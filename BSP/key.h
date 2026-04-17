#ifndef __KEY_H
#define __KEY_H

#include "main.h"

// 定义系统运行模式枚举
typedef enum {
    SYS_MODE_INIT        = 0, // 初始上电状态或复位状态
    // -- 基础部分 --
    SYS_MODE_BASIC_2     = 2, // 基础任务2: DAC开环输出
    SYS_MODE_BASIC_3     = 3, // 基础任务3: PI闭环稳幅输出
    SYS_MODE_BASIC_4     = 4, // 基础任务4: 模拟数字滤波
    // -- 冗余/过渡标志 --
    SYS_MODE_AUTO_MSG    = 5, // 提示信息显示(过渡态)
    // -- 发挥部分 --
    SYS_MODE_ADVANCED_1  = 6, // 发挥1任务: 盲扫未知电路频响模型
    SYS_MODE_ADVANCED_2  = 7  // 发挥2任务: 根据扫出模型自动启动数字模拟
} MachineTaskMode_t;

extern volatile MachineTaskMode_t key_flag;

void Key_handler(uint16_t GPIO_Pin);

#endif
