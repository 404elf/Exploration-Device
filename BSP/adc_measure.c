#include "adc_measure.h"
#include "adc.h"
#include "signal_gen.h"
#include "tim.h"
#include "string.h"
#include "OLED.h"
/**
 * 文件介绍：
 * 
 */


#define ADC_BUF_SIZE 256  
#define DWT_CYCCNT_CLK_HZ ((float)HAL_RCC_GetHCLKFreq()) //DWT的频率，与HCLK同频

uint16_t ADC_Value_Buffer[ADC_BUF_SIZE];    // ADC采样值缓冲区

//*markdown：可见这种计算不返回，用全局变量引出是一种很方便的方法
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
 * @brief 启动ADC
 */
void ADC_Measure_Start(void){
    HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_Value_Buffer,ADC_BUF_SIZE);
    ////HAL_TIM_Base_Start(&htim2); 
}

/**
 * @brief 从测量数据中找到Vpp
 * @param pBuffer 指向测量数据
 * @param length 需要处理的数据大小
 */
void ADC_Cal_Vpp(uint16_t* pBuffer, uint16_t length){
    // 静态变量，重复运行，到点清零，需要记忆
    static uint16_t global_max = 0;
    static uint16_t global_min = 4095;
    static uint8_t  calc_count = 0;
    
    //半中断中找极值
    for (int i = 0; i < length; i++){
    //*mark：能用本地变量，就别用外设寄存器，也别用内存
    uint16_t pBuffer_Reg=pBuffer[i];
    if(pBuffer_Reg > global_max) global_max = pBuffer_Reg;
    if(pBuffer_Reg < global_min) global_min = pBuffer_Reg;
    }
    
    calc_count++;
    
    //累计50次总结一下，涵盖更大时间范围
    //!虽然处理数据是O（N），但是中断调用有着固定开销，会使得时间占比越大
    //*MK 占用不会消失，只是另一个方向转移……
    if(calc_count >= 50) {
        // 计算最终的 Vpp
        float final_vpp = (global_max - global_min) * 3.3f / 4095.0f;
        current_measured_vpp = final_vpp;
        
        //准备重新测量
        global_max = 0;
        global_min = 4095;
        calc_count = 0;
    }
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
            // DWT 运行在 HCLK，因此这里要使用 HCLK 作为周期换算基准
            current_measured_freq = (DWT_CYCCNT_CLK_HZ / (float)period_cycles) * 500.0f;
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
void Measure_ADC_HalfCpltCallback(void) {
     ADC_Cal_Vpp(&ADC_Value_Buffer[0], ADC_BUF_SIZE / 2);
}

/**
 * @brief 乒乓缓存全满完成中断：DMA发生回放绕并开始重写前半段数据，此刻CPU趁机处理刚装好的后半段(100 ~ 199)
 */
void Measure_ADC_FullCpltCallback(void) {
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
