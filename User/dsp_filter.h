#ifndef __DSP_FILTER_H
#define __DSP_FILTER_H

#include "main.h"

//Double Buffering
#define FILTER_BUF_SIZE 200

// 外部调用的控制接口
void Task4_Filter_Init(void);  // intal
void Task4_Filter_Start(void); // start IIR filter
void Task4_Filter_Stop(void);  // stop IIR filter

//Interrupt
void Task4_ADC_HalfCpltCallback(void); 
void Task4_ADC_FullCpltCallback(void);

#endif