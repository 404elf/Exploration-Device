#include "adc_measure.h"
#include "adc.h"
#include "signal_gen.h"
#include "tim.h"
#include "string.h"
#include "OLED.h"
/**
 * 文件介绍：这里主要运用了双缓冲机制，以半中断和全中断为界，来回的对Vpp进行计算
 */

#define ADC_BUF_SIZE 256  


uint16_t ADC_Value_Buffer[ADC_BUF_SIZE];    // ADC采样值缓冲区

//*markdown：可见这种计算不返回，用全局变量引出是一种很方便的方法
//全局变量Vpp（需要自取）
volatile float current_measured_vpp = 0.0f; 


/**
 * @brief 启动ADC
 */
void ADC_Measure_Start(void){
    HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_Value_Buffer,ADC_BUF_SIZE);
    ////HAL_TIM_Base_Start(&htim2); 
}

//*mark:这里的Get，实现了处理与读写分离的操作，确保了数值自动更新，随意拿取
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

/**
 * @brief:任务三初始化
 * 任务三要用到ADC数组，以采集V来判断Vpp
 */

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
