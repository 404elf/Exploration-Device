#include "control_sys.h"
#include "adc_measure.h"
#include "signal_gen.h"
#include "tim.h"
#include "math.h"

static PI_Controller vpp_ctrl;
////static volatile uint8_t ctrl_timer=0;

/**
 * @brief PID inital£¬and open IT
 */
void Control_Init(void){
    //PI value of the first time(who need to record)
    vpp_ctrl.Target=2.2f;   
    vpp_ctrl.Kp=0.15f;   
    vpp_ctrl.Ki=0.05f;   

    vpp_ctrl.Integral=0; 

    //Voltage output of MCU 
    vpp_ctrl.Output=0.1f;   

    //avoid signal of IT_UPDATE (inital)
    __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);

    //open timer and Enable interrupts
    HAL_TIM_Base_Start_IT(&htim7);

}

/**
 * @brief calculate of PI
 */
void Control_Task_100ms(void){
    //FIFO(fast in fast out)
    if (compute_flag){
        //classic Algorithm
        float current_vpp=ADC_Cal_Vpp();
        
        float error = vpp_ctrl.Target - current_vpp;

        //if small,stop integral
        if (fabs(error)>0.01f){
            vpp_ctrl.Integral +=error;
        }
        //maybe too long
        if(vpp_ctrl.Integral > 10.0f) vpp_ctrl.Integral = 10.0f;
        if(vpp_ctrl.Integral < -10.0f) vpp_ctrl.Integral = -10.0f;

        //As above(classic Algorithm)
        vpp_ctrl.Output=(vpp_ctrl.Kp*error)+(vpp_ctrl.Ki*vpp_ctrl.Integral);

        //limit of Vout
        if (vpp_ctrl.Output>3.0f)vpp_ctrl.Output=3.0f;
        if (vpp_ctrl.Output<0.0f)vpp_ctrl.Output=0.0f;

        //decide to new output
        SignalGen_UpdateVpp(vpp_ctrl.Output);

        ////ctrl_timer=0;
        compute_flag = 0;
    }
}

//!remember to delete it
//no use it(TIM7),is for phase.
/* ~~
//Timed Interrupt 100ms(CubeMX open TIM7 global interrupt)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM7) {
        //FIFO(fast in fast out)
        ctrl_timer=1;
    }
}
~~ */

