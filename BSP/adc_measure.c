#include "adc_measure.h"
#include "adc.h"
#include "signal_gen.h"
#include "tim.h"
#include "string.h"

#define ADC_BUF_SIZE 512 //为了Vpp测量更准确，可以适当开大
uint16_t ADC_Value_Buffer[ADC_BUF_SIZE];    // ADC采样值缓冲区
//全局变量Vpp（需要自取）
volatile float current_measured_vpp = 0.0f; 

////uint16_t SafeBuffer[ADC_BUF_SIZE]; 
//PI计周期触发变量
volatile uint16_t cycle_cnt = 0;
/////volatile uint8_t compute_flag = 0;     源源不断更新Vpp
//测频率
volatile uint32_t now_time = 0;
volatile uint32_t last_time = 0;
//频率更新标志
volatile uint8_t  freq_update_flag = 0; // 频率更新标志，通知主应用新数据到达

/**
 * @brief 启动ADC测量及其对应的定时器
 */
void ADC_Measure_Start(void){
    HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_Value_Buffer,ADC_BUF_SIZE);
    HAL_TIM_Base_Start(&htim2); 
}

/**
 * @brief 计算输入数据的Vpp
 * @param pBuffer 指针  指向前/半部分缓冲区
 * @param length 需要处理的数据大小
 */
void ADC_Cal_Vpp(uint16_t* pBuffer, uint16_t length){
    uint16_t max=0,min=4095;

    for (int i = 0; i < length; i++){
        if(pBuffer[i] > max) max = pBuffer[i];
        if(pBuffer[i] < min) min = pBuffer[i];
    }
    // 计算得到的Vpp为MCU端ADC探测到的实际电压峰峰值
    float vpp = (max - min) * 3.3f / 4095.0f;
    current_measured_vpp = vpp;
}

/**
 * @brief 导出Vpp
 */
float Get_Vpp(void) {
    return current_measured_vpp;
}

//? 优化建议：如果有足够的开发时间，可以尝试使用定时器的从模式(复位模式)来实现更精确的捕获对齐
/**
 * @brief 过零比较器上升沿触发
 */
void is_PI(void){
    //计完数就跑，快进快出
    cycle_cnt++;
    if (cycle_cnt >= 500) {
        cycle_cnt = 0;

        // PI触发才获取周期计数值
        now_time = DWT->CYCCNT;     //每500个周期计数一次。5khz的波是0.1s，32位足够

        //统一触发
        freq_update_flag = 1;   //频率计算
        ////compute_flag = 1; // 触发PID更新标志
        //相位重置，提供一个基准
        SignalGEN_Restart(); 
    }
}

//! 需要注意：如果刚上电或刚切换到此模式，首次触发时由于last_time为0，算出的差值可能会很大导致突变
/**
 * @brief 计算输入波形频率，并实现同频输出
 */
void update_freq(void){
    if (last_time==0) {
        last_time = now_time;
        //! 更加好的方案是在切换到该模式时固定触发，防止时间滞后比较。但是估计不会来回切换吧？留个坑先
        return;
    }
    if (freq_update_flag) {
        freq_update_flag = 0;
        
        // 无符号32位整数相减自动处理溢出回绕问题 (类似 uint8_t 的 1-255=2)
        uint32_t period_cycles = now_time - last_time;
        
        last_time = now_time;

        // 计算当前输入信号的频率，这里的系统时钟频率为168MHz
        float current_freq = (168000000.0f / (float)period_cycles)*500.0f;

        if (current_freq >= 800.0f && current_freq <= 55000.0f) {
            Set_DDS_Freq(current_freq);
        }
    }
}

//与隔壁signal_gen.c的逻辑相同
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        ADC_Cal_Vpp(&ADC_Value_Buffer[0], ADC_BUF_SIZE / 2);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        ADC_Cal_Vpp(&ADC_Value_Buffer[ADC_BUF_SIZE / 2], ADC_BUF_SIZE / 2);
    }
}

