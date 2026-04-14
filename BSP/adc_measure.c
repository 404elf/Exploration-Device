#include "adc_measure.h"
#include "adc.h"
#include "signal_gen.h"
#include "tim.h"
#include "string.h"

#define ADC_BUF_SIZE 512
uint16_t ADC_Value_Buffer[ADC_BUF_SIZE];    // ADC采样值缓冲区
uint16_t SafeBuffer[ADC_BUF_SIZE]; 

volatile uint16_t cycle_cnt = 0;
volatile uint8_t compute_flag = 0;

volatile uint32_t now_time = 0;
volatile uint32_t last_time = 0;
volatile uint8_t  freq_update_flag = 0; // 频率更新标志，通知主应用新数据到达

/**
 * @brief 启动ADC测量及其对应的定时器
 */
void ADC_Measure_Start(void){
    HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_Value_Buffer,ADC_BUF_SIZE);
    HAL_TIM_Base_Start(&htim2); 
}

/**
 * @brief 计算MCU采样到的ADC数据的峰峰值(Vpp)
 */
float ADC_Cal_Vpp(void){
    uint16_t max=0,min=4095;

    //! 为了防止计算过程中DMA正在覆写数据，使用安全缓冲区备份数据
    memcpy(SafeBuffer, ADC_Value_Buffer, sizeof(SafeBuffer));   // memcpy的第三个参数是以字节为单位的大小

    for (int i=0;i<ADC_BUF_SIZE;i++){
        if(SafeBuffer[i]>max) max=SafeBuffer[i];
        if(SafeBuffer[i]<min) min=SafeBuffer[i];
    }
    // 计算得到的Vpp为MCU端ADC探测到的实际电压峰峰值
    float Vpp=(max-min)*3.3f/4095.0f;
    return Vpp;
}

//? 优化建议：如果有足够的开发时间，可以尝试使用定时器的从模式(复位模式)来实现更精确的捕获对齐
/**
 * @brief 过零点外部中断事件处理回调，由控制引脚上升沿触发
 */
void is_PI(void){
    // 使用DWT作为高压定时器，获取当前的周期计数值CYCCNT
        now_time = DWT->CYCCNT; 

        // 当输入信号从负半轴过零跳变到正半轴时触发，此处重启信号发生波形以实现相位对齐(锁相)
        SignalGEN_Restart();
        cycle_cnt++;
        if (cycle_cnt >= 500) {
            cycle_cnt = 0;
            compute_flag = 1; // 触发PID更新标志
        }
    
    freq_update_flag = 1; 
}

//! 需要注意：如果刚上电或刚切换到此模式，首次触发时由于last_time为0，算出的差值可能会很大导致突变
/**
 * @brief 更新并计算信号的实际频率，再同步设置波形输出定时器
 */
void update_freq(void){
    if (freq_update_flag) {
        freq_update_flag = 0;
        
        // 无符号32位整数相减自动处理溢出回绕问题 (类似 uint8_t 的 1-255=2)
        uint32_t period_cycles = now_time - last_time;
        
        last_time = now_time;

        // 计算当前输入信号的频率，这里的系统时钟频率为168MHz
        float current_freq = 168000000.0f / (float)period_cycles;

        // 通过改变TIM6的自动重装载寄存器(ARR)同步改变输出波形的频率
        if (current_freq >= 800.0f && current_freq <= 55000.0f) {
            uint32_t new_arr = (uint32_t)(84000000.0f / (current_freq * 200.0f)) - 1;
            __HAL_TIM_SET_AUTORELOAD(&htim6, new_arr);
        }
    }
}

