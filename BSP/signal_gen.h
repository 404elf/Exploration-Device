#ifndef __SIGNAL_GEN_H
#define __SIGNAL_GEN_H

#include "main.h"

#define PI 3.1415926535f
#define SINE_SAMPLES 200

float Cal_Vin(float Vout,float freq);
void SignalGen_InitTable(float vpp_target);	//初始化表格
void SignalGen_Start(void);					//启动硬件输出
void SignalGen_UpdateVpp(float new_vpp);	//运行中动态改变幅度
	

#endif
