#include "control_sys.h"
#include "adc_measure.h"
#include "signal_gen.h"
#include "tim.h"
#include "math.h"

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
/* ~~
    // 清除定时器更新所默认导致的初始中断标志位操作
    __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);

    // 开启定时器同时使能其中断
    HAL_TIM_Base_Start_IT(&htim7);
~~ */
}

/**
 * @brief 周期触发条件下处理的PI系统闭环调节核心总任务
 */
void PI_Task(void){
    // 判断是否有触发运算标志要求
    if (compute_flag){
        // 采集并计算得到当前实际系统电压Vpp数值
        float current_vpp=ADC_Cal_Vpp();
        
        // 调用子控制处理单元带入数据并获取下一阶段的指令需求修值
        PI_compute(current_vpp);
        
        // 将系统计算出的新修理想修正参考幅值Vout发送给波形模块以修正真实输出电压
        SignalGen_UpdateVpp(vpp_ctrl.Output);

        ////ctrl_timer=0;
        compute_flag = 0;
    }
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

//! TODO: 经系统优化后以后若确认彻底废弃此独立100ms事件更新方法可移除这代码。
// 已不再使用TIM7单独周期驱动相控而是在信号交越中断触发去协同调频修正PI策略。
/* ~~
// TIM7全局周期间定时100ms中断处理回调函数 (由CubeMX外设触发)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM7) {
        // 置更新执行标志位留给主控制循环执行操作逻辑
        ctrl_timer=1;
    }
}
~~ */

