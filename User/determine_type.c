#include "determine_type.h"
#include "OLED.h"
#include "signal_gen.h"
#include "adc_measure.h"
#include "tim.h"
#include "dac.h"
#include "adc.h"

#define LUCK 0      //不知道，预留两种写法，取决于发挥2进度

#if (LUCK == 0)

// 你指定的5个测试频点
const float test_freqs[5] = {1000.0f, 10000.0f, 25000.0f, 40000.0f, 50000.0f};
// 用于存放稳态均值Vpp的数组
float vpp_results[5] = {0};

FilterType Identify_Filter_Type(void) {
    // -------------------------------------------------------------
    // 1. 清除前面任务的影响，重新接管纯粹的 DDS和ADC 功能
    // -------------------------------------------------------------
    // 先停止相关定时器和DMA，防止切换时产生越界或死锁
    __HAL_TIM_DISABLE(&htim2);
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
    HAL_ADC_Stop_DMA(&hadc1);
    
    // 模拟前台准备：给定 1Vpp (或自定) 恒定幅值
    SignalGen_Start(1.0f);   
    ADC_Measure_Start();     
    __HAL_TIM_ENABLE(&htim2);

    // -------------------------------------------------------------
    // 2. 核心逻辑：发一定时间的波 -> 等待稳态 -> 中部分多次测量 -> 存入数组
    // -------------------------------------------------------------
    for (int i = 0; i < 5; i++) {
        // 设置指定频率
        Set_DDS_Freq(test_freqs[i]);
        
        // 【关键设计】：延时 200ms 放空前面的畸变期 (等待RLC电路进入稳态)
        HAL_Delay(200);   

        // 在中部时间开始多次测量Vpp（共测量 10 次）
        float vpp_sum = 0.0f;
        for (int j = 0; j < 10; j++) {
            vpp_sum += Get_Vpp(); // 从ADC测幅模块拿当前的幅值
            HAL_Delay(10);        // 每次采集间隔 10ms
        }
        
        // 计算平均值存入5个点的数组中
        vpp_results[i] = vpp_sum / 10.0f;
        
        // 发送末尾可顺便再等一下凑够总时间n（可选）
        HAL_Delay(100); 
    }

    // -------------------------------------------------------------
    // 3. 特征关系提取与类型判断
    // -------------------------------------------------------------
    // 寻找 1kHz - 50kHz 中，幅值最大和最小的点在哪一个索引
    int max_idx = 0;
    int min_idx = 0;
    for (int i = 1; i < 5; i++) {
        if (vpp_results[i] > vpp_results[max_idx]) max_idx = i;
        if (vpp_results[i] < vpp_results[min_idx]) min_idx = i;
    }

    // 根据极值点落位判断滤波类型
    if (max_idx == 0 && min_idx == 4) {
        // 最大值在1k，最小值在50k -> 随频率升高衰减 -> 低通滤波器
        return FILTER_LOW_PASS;
    } 
    else if (max_idx == 4 && min_idx == 0) {
        // 最小值在1k，最大值在50k -> 随频率升高增强 -> 高通滤波器
        return FILTER_HIGH_PASS;
    } 
    else if (max_idx == 1 || max_idx == 2 || max_idx == 3) {
        // 最大值在中间频段 (10k, 25k, 40k) -> 两头衰减，中间痛 -> 带通滤波器
        return FILTER_BAND_PASS;
    } 
    else if (min_idx == 1 || min_idx == 2 || min_idx == 3) {
        // 最小值在中间频段 -> 两头痛，中间被滤除 -> 带阻滤波器
        return FILTER_BAND_STOP;
    }

    // 保底处理(如果在边缘情况不太明显，直接比大小)
    if (vpp_results[0] > vpp_results[4]) {
        return FILTER_LOW_PASS;
    } else {
        return FILTER_HIGH_PASS;
    }
}

void task5_do(void){
    FilterType Result = Identify_Filter_Type();
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
        default:        
        break;
    }
};
#else







#endif
