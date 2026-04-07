#include "adc_measure.h"
#include "adc.h"
#include "signal_gen.h"

#define ADC_BUF_SIZE 512
uint16_t ADC_Value_Buffer[ADC_BUF_SIZE];    //buffer！！！！>sample value

/**
 * @brief start measurement
 */
void ADC_Measure_Start(void){
    HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_Value_Buffer,ADC_BUF_SIZE);
}

/**
 * @brief calculate real Vpp for mcu 
 */
float ADC_Cal_Vpp(void){
    uint16_t max=0,min=4095;
    for (int i=0;i<ADC_BUF_SIZE;i++){
        if(ADC_Value_Buffer[i]>max) max=ADC_Value_Buffer;
        if(ADC_Value_Buffer[i]<min) min=ADC_Value_Buffer;
    }
    //Vpp is Voltage of adc of mcu 
    float Vpp=(max-min)*3.3f/4095.0f;
    return Vpp;
}

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

