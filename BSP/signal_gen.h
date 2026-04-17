#ifndef __SIGNAL_GEN_H
#define __SIGNAL_GEN_H

#include "main.h"

#define PI 3.1415926535f

////define SINE_SAMPLES 200     
#define DAC_BUF_SIZE 400 //上为定周法，此为定频法

float Cal_Vin(float Vout,float freq);
void Set_DDS_Freq(float freq);              // 设置DDS输出频率
void Init_SineRef(void);                    // 初始化1024点的基准正弦表
void SignalGen_Start(float init_vpp);					// 启动波形输出
void SignalGEN_Restart(void);               // 重启波形输出（相位对齐用）
void SignalGen_Resume(void);                // 抢回DAC流控制并重开
void SignalGen_UpdateVpp(float new_vpp);	// 在运行过程中修改Vpp
void task2_do(void);
void SignalGen_DAC_HalfCpltCallback(void);
void SignalGen_DAC_FullCpltCallback(void);

#endif
