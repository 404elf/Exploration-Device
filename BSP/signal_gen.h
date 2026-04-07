#ifndef __SIGNAL_GEN_H
#define __SIGNAL_GEN_H

#include "main.h"

#define PI 3.1415926535f
#define SINE_SAMPLES 200

float Cal_Vin(float Vout,float freq);
void SignalGen_InitTable(float vpp_target);	//InitTable
void SignalGen_Start(void);					//start output
void SignalGEN_Restart(void);               //restart output
void SignalGen_UpdateVpp(float new_vpp);	//Modify Vpp in RUNNING


#endif
