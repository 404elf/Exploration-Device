#include "signal_gen.h"
#include "math.h"
#include "dac.h"
#include "tim.h"

// 定时器tim2触发DAC(PA4)输出波形

// 内部变量 - 存储在内存中的双缓冲波形表
////static uint16_t SineTable[SINE_SAMPLES];
uint16_t SineTable_A[SINE_SAMPLES];
uint16_t SineTable_B[SINE_SAMPLES];
uint16_t *pCurrentTable = SineTable_A; // 乒乓缓冲指针，初始指向表A

#define REF_POINTS 1024
static float SineRef[REF_POINTS]; // 基准正弦表 (0.0~1.0)
static float dds_phase = 0.0f;
static float dds_phase_step = 0.0f; // 步长决定频率

/**
 * @brief 计算理论输入电压Vin
 * @param Vout 目标输出电压
 * @param freq 信号频率(Hz)
 * @retval 理论需给定的Vin
 */
float Cal_Vin(float Vout,float freq){
    // 角频率 (rad/s)
    float omega = 2.0f * PI * freq;

    // 传递函数公式参数
    // 传递函数的分母部分：as^2+bs+1
    float a = 2.397e-10f;
    float b = 4.7e-5f;
    
    // 复数表示s=j*omega，计算其实部和虚部
    float real_part = 1.0f - a * omega*omega;
    float imag_part = b * omega;

    // 求复数的分母模长
    float Denominator_Len = sqrtf(real_part*real_part + imag_part*imag_part);
    if (Denominator_Len < 1e-6f) Denominator_Len = 1e-6f;   // 防止除0，限制极小值

    float Hs_Len=2.0f/Denominator_Len;
    
    float Vin = Vout / Hs_Len;
    // 可能需要系统校准因数进行修正，此处暂为1.0
    Vin = Vin / 1.0f;
    return Vin;
}

/**
 * @brief 以1024个检查点为基准，判断一步要走几个检查点
 * @param freq 输出波形的频率
 */
void Set_DDS_Freq(float freq) {
    //DDS（Direct Digital Synthesis）直接数字频率合成
    //相位累加器
    dds_phase_step = (freq / 1000000.0f) * REF_POINTS;  
}

/**
 * @brief 有着1024个点 的 基准sin表（值域为0，1）
 */
void Init_SineRef(void) {
    for(int i=0; i<REF_POINTS; i++) 
        SineRef[i] = (sinf(2.0f * PI * i / REF_POINTS) + 1.0f) / 2.0f;
}

/**
 * @brief 
 * @param pTable 指针->在双表切换
 * @param vpp_target 目标Vpp (0.0V-3.3V)
 */
//SineRef[(int)dds_phase] VS sinf(2.0f * PI * i/SINE_SAMPLES)
//前者查表，后者时实计算，
//查表=表+数据+地址（地址移动规则）
static void SignalGen_FillTable(uint16_t* pTable, float vpp_target) {
    float amp = (vpp_target / 3.3f) * 4095.0f;
    float offset = 2048.0f - (amp / 2.0f);
    for (int i=0; i<SINE_SAMPLES; i++) {
        /*
        是冗余吗？
        int index = (int)dds_phase;
        if(index >= REF_POINTS) index = REF_POINTS - 1;
        else if(index < 0) index = 0;

        float val = SineRef[index] * amp + offset;
        */
        float val = (uint16_t)(SineRef[(int)dds_phase] * amp + offset);
        
        //限制幅度
        if (val > 4095.0f) val = 4095.0f;
        if (val < 0.0f) val = 0.0f;

        pTable[i]=(uint16_t)val;

        dds_phase += dds_phase_step;
        if (dds_phase >= REF_POINTS) dds_phase -= REF_POINTS;
    }
}

/**
 * @brief 启动定时器及DAC的DMA数据传输开始输出
 */
void SignalGen_Start(void){
    // 启动DAC对应的DMA传输请求
    HAL_DAC_Start_DMA(&hdac,DAC1_CHANNEL_1,(uint32_t*)pCurrentTable,SINE_SAMPLES,DAC_ALIGN_12B_R);

    // 启动触发波形输出的定时器tim2
    HAL_TIM_Base_Start(&htim2);
}


//! 用于相位对齐：每次外部过零跳变触发时进行 停止->清零计数值->再启动 的操作
//? 频繁调用可能引入短暂的空窗停顿期，需留意交界处是否在高速下产生波形失真
/**
 * @brief 重新启动DAC-DMA传输（用于实现从0相位重新开始的相位对齐）
 */
void SignalGEN_Restart(void){
    // 首先停止定时器触发外设

    __HAL_TIM_DISABLE(&htim2);
    
    // 必须调用Stop指令取消当前DAC的DMA传输状态，否则单独Start将无动作
    HAL_DAC_Stop_DMA(&hdac,DAC_CHANNEL_1);
    
    // 使用已更新或原初指针再次完整启动一轮新传输
    HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1,(uint32_t*)pCurrentTable,SINE_SAMPLES,DAC_ALIGN_12B_R);
    
    // 清除定时器计数值使其从0开始计数实现相位对齐(从0度角开始输出)
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    // 恢复开放定时器启动更新事件
    __HAL_TIM_ENABLE(&htim2);
}

/**
 * @brief 在系统运行时动态更新波形输出Vpp
 * @param new_vpp 目标设定新峰峰电压(0.0V-3.3V)
 */
void SignalGen_UpdateVpp(float new_vpp) {
    // 双表切换
    uint16_t *pNextTable = (pCurrentTable == SineTable_A) ? SineTable_B : SineTable_A;
    SignalGen_FillTable(pNextTable, new_vpp);
    
    __HAL_TIM_DISABLE(&htim2); // 停止时钟

    //停表
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
    pCurrentTable = pNextTable;
    
    //切表，切了表自己会走
    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)pCurrentTable, SINE_SAMPLES, DAC_ALIGN_12B_R);
    __HAL_TIM_SET_COUNTER(&htim2, 0); // 计数清零
    __HAL_TIM_ENABLE(&htim2); // 开启
}

