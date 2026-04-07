#ifndef __CONTROL_SYS_H
#define __CONTROL_SYS_H

#include "main.h"

typedef struct{

    float Target;   //goal
    float Kp;   //k of Proportion
    float Ki;   //k of Integration

    float Integral; //Integration(no k)

    float Output;   //value that want to change
}PI_Controller;

void Control_Init(void);
void Control_Task_100ms(void);

#endif
