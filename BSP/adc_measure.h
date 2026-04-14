#ifndef __ADC_MEASURE_H
#define __ADC_MEASURE_H

#include "main.h"
    
////extern volatile uint8_t compute_flag;

void ADC_Measure_Start(void);   // 启动ADC测量
void ADC_Cal_Vpp(uint16_t* pBuffer, uint16_t length);    // 计算Vpp峰峰值
void update_freq(void);
void is_PI(void);
float Get_Vpp(void);
#endif
