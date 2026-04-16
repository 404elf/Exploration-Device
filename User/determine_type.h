#ifndef __DETERMINE_TYPE_H
#define __DETERMINE_TYPE_H

#include "main.h"


#define ISLUCK 1      //不知道，预留两种写法，取决于发挥2进度

// 定义滤波器类型枚举
typedef enum {
    FILTER_UNKNOWN = 0,
    FILTER_LOW_PASS,   // 低通
    FILTER_HIGH_PASS,  // 高通
    FILTER_BAND_PASS,  // 带通
    FILTER_BAND_STOP,  // 带阻
    FILTER_ALL_PASS    // 全通
} FilterType;

// 暴露给 main.c 调用的主控函数
void task5_do(void);
void task6_do(void);
#endif
