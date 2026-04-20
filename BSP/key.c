#include "key.h"
#include "gpio.h"
#include "signal_gen.h"

#define DWT_CYCCNT_CLK_HZ ((float)HAL_RCC_GetHCLKFreq()) //DWT的频率，与HCLK同频

volatile uint32_t last_press_tick = 0; // 上次有效按键的时间戳
volatile uint8_t  button_event = 0;    // 按键事件标志位
//最新频率(需要自取)
volatile float current_measured_freq = 5000.0f;

//测频率
volatile uint32_t now_time = 0;
volatile uint32_t last_time = 0;

volatile MachineTaskMode_t key_flag = SYS_MODE_BASIC_2;

static void measure_freq(void);
static void phase_restart(void);

//导出给中断使用
void Key_handler(uint16_t GPIO_Pin){

	uint32_t current_tick = HAL_GetTick(); // 获取当前毫秒数

    switch (GPIO_Pin){
       case GPIO_PIN_0:
        measure_freq(); //判断并计算频率
        phase_restart(); //判断并相位重置
        break;
////默认让pin1和pin2作为按键触发切换模式，后面再初始化
//!last_press_tick被共享了，可能会出现按key1快速按key2，key2被屏蔽，不过从某种角度来说也防止了连按
      case GPIO_PIN_1:
        if (current_tick - last_press_tick > 200) {
            // 仅在基础模式 2~4 循环
            if (key_flag < SYS_MODE_BASIC_4) key_flag = (MachineTaskMode_t)(key_flag + 1);
            else key_flag = SYS_MODE_BASIC_2;
            last_press_tick = current_tick;
            }
        break;

      case GPIO_PIN_2:
        if (current_tick - last_press_tick > 200) {
            // 仅在基础模式 2~4 循环
            if (key_flag > SYS_MODE_BASIC_2) key_flag = (MachineTaskMode_t)(key_flag - 1);
            else key_flag = SYS_MODE_BASIC_4;
            last_press_tick = current_tick;
            }
        break;

      case GPIO_PIN_3: // 独立的一键自动学习建（Auto键）
        if (current_tick - last_press_tick > 200) {
            key_flag = SYS_MODE_ADVANCED_1; // 强行切入一键自动学习状态 (发挥1)
            last_press_tick = current_tick;
        }
        break;

      default:
          break; 
    }
}

//? 优化建议：如果有足够的开发时间，可以尝试使用定时器的从模式(复位模式)来实现更精确的捕获对齐
/**
 * @brief 计算频率判断与执行
 * 过零比较器上升沿触发（每五百回计算一次）
 */
static void measure_freq(void){
    static uint16_t cycle_cnt=0;
  //计完数就跑，快进快出
    cycle_cnt++;
    //五百周期重置一次
    if (cycle_cnt >= 500) {
        cycle_cnt = 0;

        // PI触发才获取周期计数值
        now_time = DWT->CYCCNT;     //每500个周期计数一次。5khz的波是0.1s，32位足够

        if (last_time != 0) { 
            uint32_t period_cycles = now_time - last_time;
            // DWT 运行在 HCLK，因此这里要使用 HCLK 作为周期换算基准
            current_measured_freq = (DWT_CYCCNT_CLK_HZ / (float)period_cycles) * 500.0f;
        }
        last_time = now_time;
        ////compute_flag = 1; // 触发PID更新标志
        
    }
}
/**
 * @brief:相位重置判断与执行
 */
static void phase_restart(void){
  static uint16_t cycle_cnt=0; 
  cycle_cnt++;
    if(cycle_cnt>=10){
      cycle_cnt=0;

      //相位重置，提供一个基准
      SignalGEN_Restart(); 
    }
}

/**
 * @brief 自取频率
 */
float Get_freq(void){
    return current_measured_freq; 
}

