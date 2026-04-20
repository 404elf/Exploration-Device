#include "dsp_filter.h"
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "string.h"
#include "determine_type.h"
#include "math.h"
#include "OLED.h"
////#include "arm_math.h"   //优化IIR 函数
////static arm_biquad_casd_df1_inst_f32 iir_instance;
////static float32_t iir_state[4];  //历史状态，只有历史的，才叫历史状态
////static float32_t iir_coeffs[5]; 

////static float32_t float_in_buf[FILTER_BUF_SIZE / 2];
////static float32_t float_out_buf[FILTER_BUF_SIZE / 2];
// H(s)传递函数分母系数(A2*s^2 + A1*s + A0) (已知模型电路参数)
#define A2_COEF 2.397e-10f
#define A1_COEF 4.7e-5f
#define A0_COEF 1.0f
#define DC_GAIN 2.0f // 系统增益参数

#ifndef PI
#define PI 3.1415926535f
#endif

// 定时器触发的系统ADC/DAC采样率(Hz)
#define SAMPLE_RATE 1000000.0f


static uint16_t ADC_Buffer[FILTER_BUF_SIZE]; // DMA自动写入的ADC采集缓冲区
static uint16_t DAC_Buffer[FILTER_BUF_SIZE]; // DMA自动读出的DAC波形缓冲区域

// 差分方程系统系数：b0~b2对应分子(输入)，a1~a2对应分母(输出)
static float b0 = 0.05f, b1 = 0.10f, b2 = 0.05f;
static float a1 = -1.20f, a2 = 0.40f;

// 历史输入输出数据状态变量(Z域延时)
static float x_n1 = 0.0f, x_n2 = 0.0f;
static float y_n1 = 0.0f, y_n2 = 0.0f;

//主要的改动在这里！！！也就是该如何求解系数

/**
 * @brief 求解IIR差分的系数
 */
void Calculate_IIR_Coeffs(void) {
    if (identified_model.type == FILTER_UNKNOWN) return;

    // 1. 提取物理参数
    float G  = identified_model.G;
    float w0 = 2.0f * PI * identified_model.f0;
    float Q  = identified_model.Q;
    
    // 安全防护防止除零
    if (Q < 0.01f) Q = 0.707f; 
    if (w0 < 1.0f) w0 = 1000.0f;

    // 2. 根据公式提取S域通式：H(s) = (B2*s^2 + B1*s + B0) / (A2*s^2 + A1*s + A0)
    float A2 = 1.0f / (w0 * w0);
    float A1 = 1.0f / (Q * w0);
    float A0 = 1.0f;
    
    float B2 = 0.0f, B1 = 0.0f, B0 = 0.0f;
    
    // 分子则根据滤波器的类型变化
    switch (identified_model.type) {
        case FILTER_LOW_PASS:
            B2 = 0.0f;         B1 = 0.0f;            B0 = G;
            break;
        case FILTER_HIGH_PASS:
            B2 = G * A2;       B1 = 0.0f;            B0 = 0.0f;
            break;
        case FILTER_BAND_PASS:
            B2 = 0.0f;         B1 = G * A1;          B0 = 0.0f;
            break;
        case FILTER_BAND_STOP:
            B2 = G * A2;       B1 = 0.0f;            B0 = G;
            break;
        case FILTER_ALL_PASS: // 全通附加项(选配): H(s)=G*(A2*s^2 - A1*s + A0)/分母
            B2 = G * A2;       B1 = -G * A1;         B0 = G * A0; 
            break;
        default:
            return;
    }

    // 3. 通用双线性变换 (s = K * (z-1)/(z+1) 即 s = K * (1-z^-1)/(1+z^-1))
    // K = 2 * Fs
    float K = 2.0f * SAMPLE_RATE;
    float K_sq = K * K;
    
    // 分母多项式展开归并
    float den0 = A2 * K_sq + A1 * K + A0;
    float den1 = 2.0f * A0 - 2.0f * A2 * K_sq;
    float den2 = A2 * K_sq - A1 * K + A0;
    
    // 分子多项式展开归并
    float num0 = B2 * K_sq + B1 * K + B0;
    float num1 = 2.0f * B0 - 2.0f * B2 * K_sq;
    float num2 = B2 * K_sq - B1 * K + B0;
    
    // 4. 系数归一化，赋值给IIR差分数组
    b0 = num0 / den0;
    b1 = num1 / den0;
    b2 = num2 / den0;
    
    a1 = den1 / den0;
    a2 = den2 / den0;
}

/**
 * @brief IIR操作并更新缓冲
 * @param pIn ADC用于提供输入数据
 * @param pOut DAC用于存放滤波结果
 * @param length 处理数据数
 */
static void Process_IIR_Block(uint16_t* pIn, uint16_t* pOut, uint16_t length) {
    for (int i = 0; i < length; i++) {
        // 输入Vpp
        float x_n = (pIn[i] / 4095.0f) * 3.3f;
        
        // IIR离散差分方程
        float y_n = b0 * x_n + b1 * x_n1 + b2 * x_n2 - a1 * y_n1 - a2 * y_n2;
        
        // 状态更新（数据在Z域平移延位以备下一个点代入计算）
        x_n2 = x_n1; 
        x_n1 = x_n;
        y_n2 = y_n1; 
        y_n1 = y_n;
        
        //DAC输出
        float dac_float = (y_n / 3.3f) * 4095.0f;
        
        // 限幅操作
        if (dac_float > 4095.0f) dac_float = 4095.0f;
        if (dac_float < 0.0f) dac_float = 0.0f;
        
        pOut[i] = (uint16_t)dac_float;
    }
}

/**
 * @brief 初始化差分方程初始状态
 */
void Task4_Filter_Init(void) {
    //历史缓存清零
    x_n1 = 0.0f; x_n2 = 0.0f;
    y_n1 = 0.0f; y_n2 = 0.0f;
}

/**
 * @brief 启动系统，触发中断，系统自动执行
 */
void Task4_Filter_Start(void) {
    __HAL_TIM_DISABLE(&htim2);

    //先关后开
    //任务切换，防止表污染，切换表前要先停
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
    HAL_ADC_Stop_DMA(&hadc1);

    __HAL_TIM_SET_COUNTER(&htim2, 0);

    // 清零缓冲区
    memset(DAC_Buffer, 0, sizeof(DAC_Buffer));
    memset(ADC_Buffer, 0, sizeof(ADC_Buffer));

    // 先ADC再DAC
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Buffer, FILTER_BUF_SIZE);
    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)DAC_Buffer, FILTER_BUF_SIZE, DAC_ALIGN_12B_R);

    __HAL_TIM_ENABLE(&htim2);
}   

/**
 * @brief 暂停系统运行
 */
void Task4_Filter_Stop(void) {
    // 停用ADC采样DMA读取与DAC波形输出驱动DMA
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
}

/**
 * @brief ADC完成前半段，DAC往前半段填，和先前一样，没什么好说的
 */
void IIR_Filter_ADC_HalfCpltCallback(void) {
    Process_IIR_Block(&ADC_Buffer[0], &DAC_Buffer[0], FILTER_BUF_SIZE / 2);
}

void IIR_Filter_ADC_FullCpltCallback(void) {
    Process_IIR_Block(&ADC_Buffer[FILTER_BUF_SIZE / 2], &DAC_Buffer[FILTER_BUF_SIZE / 2], FILTER_BUF_SIZE / 2);
}

/**
 * 任务四：模拟已知模型电路
 */
void Task4_do(void){
    OLED_Clear();
    OLED_ShowCenterString("basictask4");
    // 使用宏定义的已知参数逆推出系统的谐振频率和Q值等，覆盖写入 identified_model
    identified_model.type = FILTER_LOW_PASS;
    identified_model.G = DC_GAIN;
    
    // 根据 H(s) 中 A2 = 1/w0^2 算出 w0
    float w0 = 1.0f / sqrtf(A2_COEF); 
    identified_model.f0 = w0 / (2.0f * PI);
    
    // 根据 A1 = 1/(Q*w0) 算出 Q
    identified_model.Q = 1.0f / (A1_COEF * w0);
    
    // 将模拟出的已知电路模型转化为 IIR 参数
    Calculate_IIR_Coeffs();
    
    // 启动流水线
    Task4_Filter_Init();
    Task4_Filter_Start();
}
/**
 * 发挥二：模拟未知模型电路
 * 在发挥一中已经算出了HS，这里通过HS算出差分方程系数
 */
void Task6_do(void){
    Calculate_IIR_Coeffs();
    
    Task4_Filter_Init();
    Task4_Filter_Start();
}

