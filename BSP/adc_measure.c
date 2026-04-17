#include "adc_measure.h"
#include "adc.h"
#include "signal_gen.h"
#include "tim.h"
#include "string.h"
#include "OLED.h"

#define ADC_BUF_SIZE 4096 //为了Vpp测量更准确，可以适当开大
uint16_t ADC_Value_Buffer[ADC_BUF_SIZE];    // ADC采样值缓冲区
//全局变量Vpp（需要自取）
volatile float current_measured_vpp = 0.0f; 
//最新频率(需要自取)
volatile float current_measured_freq = 5000.0f;
////uint16_t SafeBuffer[ADC_BUF_SIZE]; 
//PI计周期触发变量
volatile uint16_t cycle_cnt = 0;
/////volatile uint8_t compute_flag = 0;     源源不断更新Vpp
//测频率
volatile uint32_t now_time = 0;
volatile uint32_t last_time = 0;

/**
 * @brief 启动ADC测量及其对应的定时器
 */
void ADC_Measure_Start(void){
    HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_Value_Buffer,ADC_BUF_SIZE);
    ////HAL_TIM_Base_Start(&htim2); 
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
 * @brief 过零比较器上升沿触发(这里由于中断触发，触发500回进行相位对齐，和开始计算频率)
 */
void is_PI(void){
    //计完数就跑，快进快出
    cycle_cnt++;
    if (cycle_cnt >= 500) {
        cycle_cnt = 0;

        // PI触发才获取周期计数值
        now_time = DWT->CYCCNT;     //每500个周期计数一次。5khz的波是0.1s，32位足够

        if (last_time != 0) { 
            uint32_t period_cycles = now_time - last_time;
            current_measured_freq = (168000000.0f / (float)period_cycles) * 500.0f;
        }
        last_time = now_time;
        ////compute_flag = 1; // 触发PID更新标志
        
        //相位重置，提供一个基准
        SignalGEN_Restart(); 
    }
}

/**
 * @brief 自取频率
 */
float Get_freq(void){
    return current_measured_freq; 
}

//! 需要注意：如果刚上电或刚切换到此模式，首次触发时由于last_time为0，算出的差值可能会很大导致突变
/**
 * @brief 更新频率（被动）
 */
//!等等，是不是出现过两个更新来着？
//哦哦 我知道了，只有一处需要用到被动更新，所以没问题了
void update_freq(void){
    if (current_measured_freq >= 800.0f && current_measured_freq <= 55000.0f) {
        Set_DDS_Freq(current_measured_freq);
    }
}


/**
 * @brief 乒乓缓存前半段完成中断(半满)：DMA仍在继续往后半段装入数据，此刻CPU趁机处理刚装好的前半段(0 ~ 99)
 */
void Task3_ADC_HalfCpltCallback(void) {
     ADC_Cal_Vpp(&ADC_Value_Buffer[0], ADC_BUF_SIZE / 2);
}

/**
 * @brief 乒乓缓存全满完成中断：DMA发生回放绕并开始重写前半段数据，此刻CPU趁机处理刚装好的后半段(100 ~ 199)
 */
void Task3_ADC_FullCpltCallback(void) {
    ADC_Cal_Vpp(&ADC_Value_Buffer[ADC_BUF_SIZE / 2], ADC_BUF_SIZE / 2);
}

void task3_do(void) {
    OLED_Clear();
    OLED_ShowCenterString("basictask3");
    // 夺回 ADC 控制权
    HAL_ADC_Stop_DMA(&hadc1);
    memset(ADC_Value_Buffer, 0, sizeof(ADC_Value_Buffer));
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Value_Buffer, ADC_BUF_SIZE);
    
    // 如果任务3也需要输出波形，这里再夺回 DAC
    //？什么叫也需要，就是需要啊啊笨蛋
    SignalGen_Resume();
}

//因为ADC走两套逻辑，所以中断就搬迁了
//与隔壁signal_gen.c的逻辑相同
/*
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
       
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        
    }
}
*/

