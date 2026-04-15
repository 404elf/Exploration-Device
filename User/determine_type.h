#ifndef __DETERMINE_TYPE_H
#define __DETERMINE_TYPE_H

#include "main.h"

// 定义滤波器类型枚举
typedef enum {
    FILTER_UNKNOWN = 0,
    FILTER_LOW_PASS,   // 低通
    FILTER_HIGH_PASS,  // 高通
    FILTER_BAND_PASS,  // 带通
    FILTER_BAND_STOP   // 带阻
} FilterType;

// 暴露给 main.c 调用的主控函数
void task5_do(void);

#endif
