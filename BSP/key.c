#include "key.h"
#include "gpio.h"
#include "adc_measure.h" // 引入 is_PI 的声明

volatile uint8_t key_flag = 2;

volatile uint32_t last_press_tick = 0; // 上次有效按键的时间戳
volatile uint8_t  button_event = 0;    // 按键事件标志位

//导出给中断使用
void Key_handler(uint16_t GPIO_Pin){

	uint32_t current_tick = HAL_GetTick(); // 获取当前毫秒数

    switch (GPIO_Pin){
       case GPIO_PIN_0:
        //用于计数cnt++，加满判断
        is_PI();
        break;
//!默认让pin1和pin2作为按键触发切换模式，后面再初始化
      case GPIO_PIN_1:
        if (current_tick - last_press_tick > 200) {
            if (key_flag<6) key_flag+=1;
            else key_flag = 2;
            last_press_tick = current_tick;
            }
        break;

      case GPIO_PIN_2:
        if (current_tick - last_press_tick > 200) {
            if (key_flag>2) key_flag-=1;
            else key_flag = 6;
            last_press_tick = current_tick;
            }
        break;

      default:
          break; 
    }
}
