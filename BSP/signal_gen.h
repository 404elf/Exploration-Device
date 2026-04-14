#ifndef __SIGNAL_GEN_H
#define __SIGNAL_GEN_H

#include "main.h"

#define PI 3.1415926535f
#define SINE_SAMPLES 200

float Cal_Vin(float Vout,float freq);
void SignalGen_InitTable(float vpp_target);	// 初始化波形数据表
void SignalGen_Start(void);					// 启动波形输出
void SignalGEN_Restart(void);               // 重启波形输出（相位对齐用）
void SignalGen_UpdateVpp(float new_vpp);	// 在运行过程中修改Vpp


#endif
