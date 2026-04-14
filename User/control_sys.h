#ifndef __CONTROL_SYS_H
#define __CONTROL_SYS_H

#include "main.h"

typedef struct{
    float Target;   // 目标设定值
    float Kp;       // 比例系数(Kp)
    float Ki;       // 积分系数(Ki)

    float Integral; // 积分累计误差值

    float Output;   // 经过PI运算计算后的计划输出值
}PI_Controller;

void Control_Init(void);
void PI_Task(void);
void PI_compute(float vpp);

#endif
