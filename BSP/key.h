#ifndef __KEY_H
#define __KEY_H

#include "main.h"
extern volatile uint8_t key_flag;
void Key_handler(uint16_t GPIO_Pin);

#endif
