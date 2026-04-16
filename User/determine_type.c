#include "determine_type.h"
#include "OLED.h"
#include "signal_gen.h"
#include "adc_measure.h"
#include "tim.h"
#include "dac.h"
#include "adc.h"



#define TEST_INPUT_VPP 1.0f  // 指定测试输入信号的Vpp基准幅值

#if (ISLUCK == 0)
FilterType Result =FILTER_UNKNOWN;
// 五点测评
const float test_freqs[5] = {1000.0f, 10000.0f, 25000.0f, 40000.0f, 50000.0f};
// 对应幅度增益 ( Gain = Vout / Vin )
float gain_results[5] = {0};

/**
 * @brief 用于判断滤波器类型
 * @retval 类型枚举
 */
FilterType Identify_Filter_Type(void) {
    //安全设置
    __HAL_TIM_DISABLE(&htim2);
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
    HAL_ADC_Stop_DMA(&hadc1);
    
    // 恒定幅值准备
    SignalGen_Start(TEST_INPUT_VPP);   
    ADC_Measure_Start();     
    __HAL_TIM_ENABLE(&htim2);

    //开始测频率
    for (int i = 0; i < 5; i++) {
        // 设置指定频率
        Set_DDS_Freq(test_freqs[i]);
        
        //等待进入稳态
        // 5τ时间暂态响应消失 99.3%，时间绰绰有余！
        //RC τ=RmaxCmax=10k*100nF=1ms，RL τ=Lmax/Rmin
        //以上是一阶的情况
        //二阶是1.6Q/f0，高Q很难稳定，不过也很少出现
        HAL_Delay(200);//!不缺时间的话也就懒得砍掉了   

        //求平均VPP以求稳
        float vpp_sum = 0.0f;
        for (int j = 0; j < 10; j++) {
            vpp_sum += Get_Vpp(); // 从ADC测幅模块拿当前的幅值
            HAL_Delay(10);        
        }
        
        //得到对应幅度列表
        gain_results[i] = (vpp_sum / 10.0f) / TEST_INPUT_VPP; // 计算增益
    }

    //大小判断滤波器类型
    int max_idx = 0;
    int min_idx = 0;
    for (int i = 1; i < 5; i++) {
        //找最大最小
        if (gain_results[i] > gain_results[max_idx]) max_idx = i;
        if (gain_results[i] < gain_results[min_idx]) min_idx = i;
    }

    // 1. 全通滤波器判断：整个频段内增益变化极其微弱 (起伏差小于最大增益值的 15%)
    if ((gain_results[max_idx] - gain_results[min_idx]) < (0.15f * gain_results[max_idx])) {
        return FILTER_ALL_PASS;
    }
    
    // 2. 其余特征类型判断
    if (max_idx == 0 && min_idx == 4) {
        //低4高0
        return FILTER_LOW_PASS;
    } 
    else if (max_idx == 4 && min_idx == 0) {
        // 低0高4
        return FILTER_HIGH_PASS;
    } 
    else if (max_idx == 1 || max_idx == 2 || max_idx == 3) {
        //中间大
        return FILTER_BAND_PASS;
    } 
    else if (min_idx == 1 || min_idx == 2 || min_idx == 3) {
        //中间小
        return FILTER_BAND_STOP;
    }

    // 保底处理(如果在边缘情况不太明显，直接比大小)
    if (gain_results[0] > gain_results[4]) {
        return FILTER_LOW_PASS;
    } else {
        return FILTER_HIGH_PASS;
    }
}

void task5_do(void){
    Result = Identify_Filter_Type();
    OLED_Clear();
    switch (Result){
        case FILTER_UNKNOWN:
            OLED_ShowCenterString("UNKNOWN");
        break;
        case FILTER_LOW_PASS:
            OLED_ShowCenterString("LOW PASS");
        break;
        case FILTER_HIGH_PASS:
            OLED_ShowCenterString("HIGH PASS");
        break;
        case FILTER_BAND_PASS:
            OLED_ShowCenterString("BAND PASS");
        break;
        case FILTER_BAND_STOP:
            OLED_ShowCenterString("BAND STOP");
        break;
        case FILTER_ALL_PASS:
            OLED_ShowCenterString("ALL PASS");
        break;
        default:        
        break;
    }
};

void scan_freq(void){

}

void task6_do(void){

}

#else
// ========== 可能用到的宏定义 ==========
#define SWEEP_START_FREQ        1000.0f     // 起始频率
#define SWEEP_FREQ_STEP         500.0f      // 步进
#define SWEEP_POINTS            99          //点数

#define BP_BS_MARGIN_RATIO      0.5f        // 带通带阻边缘区分阈值：边缘增益需小于最大值的一半或大于最小值的一倍
#define ALL_PASS_TOLERANCE      0.15f       // 全通波动容忍度 15%
#define PEAK_RESONANCE_THRES    1.2f        // 有突起的阈值（峰值/平带增益 > 1.2）
#define SQRT_1_2                0.7071f     // -3dB 衰减比例

//滤波器参数系数
typedef struct {
    FilterType type;    // 类型
    float G;            // 通带增益
    float f0;           // 中心频率/截止频率 (Hz)
    float Q;            // 品质因数
} FilterModel_t;

FilterModel_t identified_model;

// 辅助函数：线性插值找频率
// f_prev: 上一个频率, g_prev: 上一步增益 > 目标增益
// f_curr: 当前频率,   g_curr: 当前增益 < 目标增益
// target_g: 目标寻找增益 (比如0.707倍某基准)
static float Interpolate_Freq(float f_prev, float g_prev, float f_curr, float g_curr, float target_g) {
    if (g_prev == g_curr) return f_curr; // 防除零
    // Y - Y1 = ((Y2 - Y1) / (X2 - X1)) * (X - X1) 的变体求解 X
    float ratio = (target_g - g_prev) / (g_curr - g_prev);
    return f_prev + ratio * (f_curr - f_prev);
}

// 核心逻辑(在你的 ISLUCK 的 else 中调用或平替)
void Identify_Filter_Model_HighRes(void) {
    float gain_curve[SWEEP_POINTS];
    float freq_array[SWEEP_POINTS];
    
    float max_gain = 0.0f;
    float min_gain = 9999.0f;
    int max_idx = 0;
    int min_idx = 0;

    //安全设置
    __HAL_TIM_DISABLE(&htim2);
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
    HAL_ADC_Stop_DMA(&hadc1);
    
    // 恒定幅值准备
    SignalGen_Start(TEST_INPUT_VPP);   
    ADC_Measure_Start();     
    __HAL_TIM_ENABLE(&htim2);

    HAL_Delay(1000);

    //扫频
    for (int i = 0; i < SWEEP_POINTS; i++) {
        freq_array[i] = SWEEP_START_FREQ + i * SWEEP_FREQ_STEP;
        Set_DDS_Freq(freq_array[i]);
        HAL_Delay(50); //t stable
        
        float vpp = Get_Vpp();
        gain_curve[i] = vpp / TEST_INPUT_VPP;
        
        if (gain_curve[i] > max_gain) {
            max_gain = gain_curve[i];
            max_idx = i;
        }
        if (gain_curve[i] < min_gain) {
            min_gain = gain_curve[i];
            min_idx = i;
        }
    }

    // 2. 提取特征增益
    float G_dc = gain_curve[0];                        // 1kHz 视作直流增益近似
    float G_inf = gain_curve[SWEEP_POINTS - 1];        // 50kHz 视作无穷远增益近似
    
    // 3. 种类判断树
    float fluctuation = max_gain - min_gain;
    
    if (fluctuation < ALL_PASS_TOLERANCE * max_gain) {
        // 【全通】
        identified_model.type = FILTER_ALL_PASS;
        identified_model.G = (max_gain + min_gain) / 2.0f;
        identified_model.f0 = 0.0f; // 无特定截止意义
        identified_model.Q = 0.0f;  
    } 
    else if (G_dc < max_gain * BP_BS_MARGIN_RATIO && G_inf < max_gain * BP_BS_MARGIN_RATIO) {
        // 【带通】
        identified_model.type = FILTER_BAND_PASS;
        identified_model.G = max_gain;
        identified_model.f0 = freq_array[max_idx];
        
        // 向左寻找 -3dB (0.707 * max_gain)
        float target_g = max_gain * SQRT_1_2;
        float fL = freq_array[0], fH = freq_array[SWEEP_POINTS - 1]; // Fallback 防越界
        
        for (int i = max_idx; i > 0; i--) {
            if (gain_curve[i] > target_g && gain_curve[i-1] <= target_g) {
                fL = Interpolate_Freq(freq_array[i], gain_curve[i], freq_array[i-1], gain_curve[i-1], target_g);
                break;
            }
        }
        // 向右寻找 -3dB
        for (int i = max_idx; i < SWEEP_POINTS - 1; i++) {
            if (gain_curve[i] > target_g && gain_curve[i+1] <= target_g) {
                fH = Interpolate_Freq(freq_array[i], gain_curve[i], freq_array[i+1], gain_curve[i+1], target_g);
                break;
            }
        }
        
        float BW = fH - fL;
        identified_model.Q = (BW > 0.1f) ? (identified_model.f0 / BW) : 10.0f; // 保护
    }
    else if (G_dc > min_gain && G_inf > min_gain && min_gain < max_gain * BP_BS_MARGIN_RATIO) {
        // 【带阻】
        identified_model.type = FILTER_BAND_STOP;
        identified_model.G = (G_dc + G_inf) / 2.0f;
        identified_model.f0 = freq_array[min_idx];
        identified_model.Q = SQRT_1_2; // 默认 0.707 
        //? f0/BW算阻带？这样可能误差会很大，可能
    }
    else if (G_dc > G_inf) {
        // 【低通】
        identified_model.type = FILTER_LOW_PASS;
        identified_model.G = G_dc;
        
        //判断突起
        if (max_gain > PEAK_RESONANCE_THRES * G_dc) {
            identified_model.Q = max_gain / G_dc;
        } else {
            identified_model.Q = SQRT_1_2;
        }
        
        // 找0.707的点
        float target_g = G_dc * SQRT_1_2;
        identified_model.f0 = freq_array[SWEEP_POINTS - 1]; // Fallback
        for (int i = 0; i < SWEEP_POINTS - 1; i++) {
            if (gain_curve[i] > target_g && gain_curve[i+1] <= target_g) {
                identified_model.f0 = Interpolate_Freq(freq_array[i], gain_curve[i], freq_array[i+1], gain_curve[i+1], target_g);
                break;
            }
        }
    }
    else {
        // 【高通】
        identified_model.type = FILTER_HIGH_PASS;
        identified_model.G = G_inf;
        
        if (max_gain > PEAK_RESONANCE_THRES * G_inf) {
            identified_model.Q = max_gain / G_inf;
        } else {
            identified_model.Q = SQRT_1_2;
        }
        
        //同上
        float target_g = G_inf * SQRT_1_2;
        identified_model.f0 = freq_array[0]; // Fallback
        for (int i = SWEEP_POINTS - 1; i > 0; i--) {
            if (gain_curve[i] > target_g && gain_curve[i-1] <= target_g) {
                identified_model.f0 = Interpolate_Freq(freq_array[i], gain_curve[i], freq_array[i-1], gain_curve[i-1], target_g);
                break;
            }
        }
    }
}


void task5_do(void){
    OLED_Clear();
    OLED_ShowCenterString("Please wait……");
    Identify_Filter_Model_HighRes();
    OLED_ShowCenterString("Finish");
    HAL_Delay(3000);
    OLED_ShowCenterString("the answer is……");
    HAL_Delay(3000);
    switch (identified_model.type){
        case FILTER_UNKNOWN:
            OLED_ShowCenterString("UNKNOWN");
        break;
        case FILTER_LOW_PASS:
            OLED_ShowCenterString("LOW PASS");
        break;
        case FILTER_HIGH_PASS:
            OLED_ShowCenterString("HIGH PASS");
        break;
        case FILTER_BAND_PASS:
            OLED_ShowCenterString("BAND PASS");
        break;
        case FILTER_BAND_STOP:
            OLED_ShowCenterString("BAND STOP");
        break;
        case FILTER_ALL_PASS:
            OLED_ShowCenterString("ALL PASS");
        break;
        default:        
        break;
    }
};



#endif
