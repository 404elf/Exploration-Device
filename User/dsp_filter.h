#ifndef __DSP_FILTER_H
#define __DSP_FILTER_H

#include "main.h"

// 过滤器双缓冲大小
#define FILTER_BUF_SIZE 200

// 外部调用的控制接口
void Task4_Filter_Init(void);  // 初始化滤波器参数
void Task4_Filter_Start(void); // 启动IIR滤波器计算与数据流传输
void Task4_Filter_Stop(void);  // 停止IIR滤波器

// DMA中断处理回调接口
void Task4_ADC_HalfCpltCallback(void); 
void Task4_ADC_FullCpltCallback(void);

#endif
