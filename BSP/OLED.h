#ifndef __OLED_H
#define __OLED_H

#include "main.h"

/* ================== 用户配置区 ================== */
/* 
 * 1. 默认使用软件IIC通信，适配SD1315/SSD1306
 * 2. 请根据实际连接的引脚修改下面的宏定义
 * 3. 注意：如果你使用的是CubeMX，请将这两个引脚配置为:
 *    - GPIO_Output_PP (推挽输出) 
 *    - GPIO_SPEED_FREQ_HIGH 或 VERY_HIGH (本芯片为168MHz，适当提高引脚翻转速度)
 * 4. 如果引脚不在GPIOB上，还需要修改下方HAL_GPIO_WritePin传入的GPIO端口宏
 */

#define OLED_SCL_PORT  GPIOB
#define OLED_SCL_PIN   GPIO_PIN_6

#define OLED_SDA_PORT  GPIOB
#define OLED_SDA_PIN   GPIO_PIN_7

/* 引脚电平控制宏 */
#define OLED_SCL_H()   HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_SET)
#define OLED_SCL_L()   HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_RESET)

#define OLED_SDA_H()   HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_SET)
#define OLED_SDA_L()   HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_RESET)

/* ================== 函数API ================== */

/**
 * @brief  初始化OLED屏幕
 */
void OLED_Init(void);

/**
 * @brief  清除屏幕内容
 */
void OLED_Clear(void);

/**
 * @brief  在屏幕居中显示一行英文字符串 (由于只写了简易驱动，建议字符串长度不要过长，英文字符)
 * @param  str: 要显示的字符串
 */
void OLED_ShowCenterString(char *str);

/**
 * @brief  在指定行和列显示字符串
 * @param  row: 行号(1~8)，一共8行(由于是6x8字体)
 * @param  col: 起始像素列(1~128)，默认为1则表示从最左边开始
 * @param  str: 要显示的英文字符串
 */
void OLED_ShowString(uint8_t row, uint8_t col, char *str);

#endif

