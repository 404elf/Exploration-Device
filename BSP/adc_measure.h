#ifndef __ADC_MEASURE_H
#define __ADC_MEASURE_H

#include "main.h"
    
extern volatile uint8_t compute_flag;

void ADC_Measure_Start(void);   //start measurement
float ADC_Cal_Vpp(void);    //calculate Vpp
void update_freq(void);

#endif
