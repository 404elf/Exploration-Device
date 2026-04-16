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

//滤波器参数系数
typedef struct {
    FilterType type;    // 类型
    float G;            // 通带增益
    float f0;           // 中心频率/截止频率 (Hz)
    float Q;            // 品质因数
} FilterModel_t;

//把系数给滤波器
extern FilterModel_t identified_model;

// 暴露给 main.c 调用的主控函数
void task5_do(void);
void task6_do(void);
#endif
