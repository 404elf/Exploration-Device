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

/**
 * @brief 计算理论输入电压Vin
 * @param Vout 目标输出电压(系统最终想要的输出电压)
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
 * @brief 初始化正弦波数据表(填充表A用作初始输出)
 * @param vpp_target 目标峰峰电压Vpp (0.0V-3.3V)
 */
void SignalGen_InitTable(float vpp_target){
    float amplitude = (vpp_target / 3.3f) * 4095.0f/2.0f;
    for  (int i=0;i<SINE_SAMPLES;i++){
        // ADC/DAC偏置中心值为2048 (量程0-4095)
        float val = 2048.0f + amplitude * sinf(2.0f * PI * i/SINE_SAMPLES);

        // 防止Vpp溢出导致波形削顶失真的保护机制
        if (val>4095.0f) val = 4095.0f;
        if (val < 0.0f) val = 0.0f;

        SineTable_A[i] = (uint16_t)val;   // 转换成无符号16位整型以适配DAC数据寄存器
    }
}

/**
 * @brief 为指定的缓冲数组填充正弦波数据
 * @param pTable 指定待填充的目标波形缓冲数组
 * @param vpp_target 目标峰峰电压Vpp (0.0V-3.3V)
 */
static void SignalGen_FillTable(uint16_t* pTable, float vpp_target){
    float amplitude = (vpp_target / 3.3f) * 4095.0f/2.0f;
    for  (int i=0;i<SINE_SAMPLES;i++){
        // ADC/DAC偏置中心值为2048 (量程0-4095)
        float val = 2048.0f + amplitude * sinf(2.0f * PI * i/SINE_SAMPLES);

        // 防止Vpp溢出导致波形削顶失真的保护机制
        if (val>4095.0f) val = 4095.0f;
        if (val < 0.0f) val = 0.0f;

        pTable[i] = (uint16_t)val;   // 转换成无符号16位整型以适配DAC数据寄存器
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
void SignalGen_UpdateVpp(float new_vpp){
    // 双缓冲乒乓操作：判断当前指向表，选定处于空闲的备用表来进行数据更新装载计算处理
    uint16_t *pNextTable = (pCurrentTable == SineTable_A) ? SineTable_B : SineTable_A;

    SignalGen_FillTable(pNextTable, new_vpp);
    
    // 短暂暂停输出定时器(寄存器级屏蔽)以准备指针切换及DAC重启
    __HAL_TIM_DISABLE(&htim2);

    // 终止当前传输以防止传输冲突
    HAL_DAC_Stop_DMA(&hdac,DAC_CHANNEL_1);  // 同步停用DAC和DMA通道

    pCurrentTable = pNextTable;
    // 直接启动加载了新指针(内含新Vpp序列数据)的新DAC-DMA请求替代SignalGen_Start();
    HAL_DAC_Start_DMA(&hdac,DAC1_CHANNEL_1,(uint32_t*)pCurrentTable,SINE_SAMPLES,DAC_ALIGN_12B_R);
    
    // 清空计数器对齐复位相位
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    // 重新开启定时器触发运转
    __HAL_TIM_ENABLE(&htim2);
}

