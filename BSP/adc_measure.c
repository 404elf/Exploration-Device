#include "adc_measure.h"
#include "adc.h"
#include "signal_gen.h"
#include "tim.h"
#include "string.h"

#define ADC_BUF_SIZE 512
uint16_t ADC_Value_Buffer[ADC_BUF_SIZE];    //buffer！！！！>sample value
uint16_t SafeBuffer[ADC_BUF_SIZE]; 

volatile uint16_t cycle_cnt = 0;
volatile uint8_t compute_flag = 0;

volatile uint32_t now_time = 0;
volatile uint32_t last_time = 0;
volatile uint8_t  freq_update_flag = 0; //inform main when new data come

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
        //DWT is a pure timer��CYCCNT is cycle cnt.
        now_time = DWT->CYCCNT; 

        //if signal rise from - to +,Restart.(is for Phase alignment)
        SignalGEN_Restart();
        cycle_cnt++;
        if (cycle_cnt >= 500) {
            cycle_cnt = 0;
            compute_flag = 1; // pid flag
        }
    }
    freq_update_flag = 1; 
}

//! if turn to this mode, the first time is that DWT->CYCCNT - 0.TOO BIG!
/**
 * @brief update freq of wave
 */
void update_freq(void){
    if (freq_update_flag) {
        freq_update_flag = 0;
        
        //uint32 subtract  is auto mode(such as uint8,1 - 255=1+256-255)
        uint32_t period_cycles = now_time - last_time;
        
        last_time = now_time;

        // calculate freq
        float current_freq = 168000000.0f / (float)period_cycles;

        // change wave freq.thourgh change time 6 atuoreload.
        if (current_freq >= 800.0f && current_freq <= 55000.0f) {
            uint32_t new_arr = (uint32_t)(84000000.0f / (current_freq * 200.0f)) - 1;
            __HAL_TIM_SET_AUTORELOAD(&htim6, new_arr);
        }
    }
}

