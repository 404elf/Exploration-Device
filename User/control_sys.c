#include "control_sys.h"
#include "adc_measure.h"
#include "signal_gen.h"
#include "tim.h"
#include "math.h"
#include "OLED.h"
#include <stdio.h>
#include "key.h"

static PI_Controller vpp_ctrl;
////static volatile uint8_t ctrl_timer=0;

/**
 * @brief 初始化PI控制器及其参数配置
 */
void Control_Init(void){
    // 初始化控制初次设定的目标参数与PID系数
    vpp_ctrl.Target=2.2f;   
    vpp_ctrl.Kp=0.15f;   
    vpp_ctrl.Ki=0.05f;   

    vpp_ctrl.Integral=0; 

    // MCU PI开环初始驱动预估输出暂定为0.1f 
    vpp_ctrl.Output=0.1f;   
}


/**
 * @brief 具体地执行针对输入实际反馈电压的单周期PI闭环公式控制计算
 * @param vpp 当前采样得到的反馈系统的实时实际Vpp
 */
void PI_compute(float vpp){
        // 经典位置式数字PI算法：首先计算当前控制误差(Error)
        float error = vpp_ctrl.Target - vpp;

        // 设置死区：如果误差绝对值足够小（认为受控基本达标），则暂停积分累积以免引起震荡
        if (fabs(error)>0.01f){
            vpp_ctrl.Integral +=error;
        }
        // 加入积分限幅保护(即抗积分系统长周期工作饱和失控策略)，上限及下限±10.0f
        if(vpp_ctrl.Integral > 10.0f) vpp_ctrl.Integral = 10.0f;
        if(vpp_ctrl.Integral < -10.0f) vpp_ctrl.Integral = -10.0f;

        // 根据标准积分位置算法计算实际控制增量输出：预期新输出 = 比例主控K*误差项 + 稳态控制K*累计积分项
        vpp_ctrl.Output=(vpp_ctrl.Kp*error)+(vpp_ctrl.Ki*vpp_ctrl.Integral);

        // 对实际所作用到信号系统的输出参数值上限下限做截断与硬限制约束以防物理设备损坏
        if (vpp_ctrl.Output>3.0f)vpp_ctrl.Output=3.0f;
        if (vpp_ctrl.Output<0.0f)vpp_ctrl.Output=0.0f;
}

/**
 * @brief 周期触发条件下处理的PI系统闭环调节核心总任务
 */
void PI_Task(void){
        static uint32_t last_time = 0;        // 记录上次运行time用于求间隔
        uint32_t current_time = HAL_GetTick();   // 系统时间
        if (last_time == 0) last_time = current_time;
        //多少时间一次PI控制 detat很重要
        if (current_time - last_time >= 100){
            // 获取Vin的Vpp
            float current_vpp=Get_Vpp();
            //我们预期让Vout改变的数值，但是很可惜，还需要通过未知模型，所以我们打算用cal逆向，天才的设计！
            //所以我把计算写死在单片机里绝对不是为了偷懒！！！
            PI_compute(current_vpp);

            float target_freq = Get_freq(); 
            float true_vin = Cal_Vin(vpp_ctrl.Output, target_freq);

            // 修正真实电压
            SignalGen_UpdateVpp(true_vin);
            //确保同频
            update_freq();
            //更新时间差 ！ 我怎么忘记了？？？？
            last_time = current_time;
        }
}

// ================= PID 调参配置宏 =================
// 调参模式宏定：0代表正在扫P，1代表使用最佳P去扫I
#define TUNE_MODE 0

// 当 TUNE_MODE 为 1 时，被固定的“最佳已知P值”
#define BEST_KP 0.15f 
// ================================================

/**
 * @brief PID 步长自动自整定调试函数 (DEBUG_MODE == 1 时调用)
 * 本函数通过宏定义的模式进行单体调试，消除等待焦虑。
 * 调P时，I=0；调I时，使用上方填写的BEST_KP。
 */
void PID_AutoTune_Task(void) {
    static uint32_t last_tune_time = 0;
    static float tune_Kp = 0.0f;
    static float tune_Ki = 0.0f;
    
    char show_buf[32];
    uint32_t current_time = HAL_GetTick();

    if (last_tune_time == 0) last_tune_time = current_time;
    //非阻塞定时，也是老朋友了
    
    //每次调节停留的时间
    if (current_time - last_tune_time >= 5000) {
#if TUNE_MODE == 0
        // Phase 0: 纯扫 P (I强制为0)
        tune_Ki = 0.0f;
        tune_Kp += 0.05f;  // P 的快速步进
        
        if (tune_Kp > 0.50f) {
            tune_Kp = 0.0f; // 扫到太发散直接归零重头来
        }
#else
        // Phase 1: 固定最佳 P, 纯扫 I (积分常数P:I差距极大，按100:1估算步长设置极小)
        tune_Kp = BEST_KP;
        tune_Ki += 0.01f; // I 的保守微调步长
        
        if (tune_Ki > 0.20f) {
            tune_Ki = 0.0f; // I 扫满也重跑
        }
#endif
        
        // 赋予实际闭环控制结构体中
        vpp_ctrl.Kp = tune_Kp;
        vpp_ctrl.Ki = tune_Ki;
        
        last_tune_time = current_time;
    }

    // 将内容打到 OLED 上同步供人视察（使用第1-3行以防文字乱码重叠）
    sprintf(show_buf, "DEBUG: PID Tune");
    OLED_ShowString(1, 1, show_buf);
    
#if TUNE_MODE == 0
    sprintf(show_buf, "[1. Sweeping Kp]");
#else
    sprintf(show_buf, "[2. Sweeping Ki]");
#endif
    OLED_ShowString(2, 1, show_buf);

    sprintf(show_buf, "Kp:%.2f Ki:%.3f", vpp_ctrl.Kp, vpp_ctrl.Ki);
    OLED_ShowString(4, 1, show_buf);
}
