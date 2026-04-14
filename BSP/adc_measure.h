#ifndef __ADC_MEASURE_H
#define __ADC_MEASURE_H

#include "main.h"
    
extern volatile uint8_t compute_flag;

void ADC_Measure_Start(void);   // 启动ADC测量
float ADC_Cal_Vpp(void);    // 计算Vpp峰峰值
void update_freq(void);
void is_PI(void);

#endif
