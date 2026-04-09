#include "adc_measure.h"
#include "adc.h"
#include "signal_gen.h"
#include "tim.h"
#include "string.h"

#define ADC_BUF_SIZE 512
uint16_t ADC_Value_Buffer[ADC_BUF_SIZE];    //buffer！！！！>sample value
uint16_t SafeBuffer[ADC_BUF_SIZE]; 
/**
 * @brief start measurement
 */
void ADC_Measure_Start(void){
    HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_Value_Buffer,ADC_BUF_SIZE);
    HAL_TIM_Base_Start(&htim2); 
}

/**
 * @brief calculate real Vpp for mcu 
 */
float ADC_Cal_Vpp(void){
    uint16_t max=0,min=4095;

    //! when time is going, DMA still run.
    memcpy(SafeBuffer, ADC_Value_Buffer, sizeof(SafeBuffer));   //the thrid param is byte

    for (int i=0;i<ADC_BUF_SIZE;i++){
        if(SafeBuffer[i]>max) max=SafeBuffer[i];
        if(SafeBuffer[i]<min) min=SafeBuffer[i];
    }
    //Vpp is Voltage of adc of mcu 
    float Vpp=(max-min)*3.3f/4095.0f;
    return Vpp;
}

//?maybe try Slave Mode: Reset Mod(if have time)
/**
 * @brief calculate Vpp
 * @param GPIO_Pin:the PIN who trigger an interrupt
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if (GPIO_Pin==GPIO_PIN_0){
        //if signal rise from - to +,Restart.(is for Phase alignment)
        SignalGEN_Restart();
    }
}

