#include "signal_gen.h"
#include "math.h"
#include "dac.h"
#include "tim.h"
//定周DDS   点准频率变  点就是波形精细度
//用指针来切表
//定频DDS   点变频率准
//一个表分上下部分流式输出


// 定时器tim2触发DAC(PA4)输出波形

// 内部变量 - 存储在内存中的双缓冲波形表
////static uint16_t SineTable[SINE_SAMPLES];
uint16_t DAC_Buffer[DAC_BUF_SIZE]; // 唯一的一段环形 DMA 缓冲

#define REF_POINTS 512
static float SineRef[REF_POINTS]; // 基准正弦表 (0.0~1.0)
static float dds_phase = 0.0f;
static float dds_phase_step = 0.0f; // 步长决定频率

//修改此即可修改vpp
static volatile float current_vpp_target = 0.0f;

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
 * @brief 有着1024个点 的 基准sin表（值域为0，1）RE
 * REF_POINTS决定表有几个点
 */
void Init_SineRef(void) {
    for(int i=0; i<REF_POINTS; i++) 
        SineRef[i] = (sinf(2.0f * PI * i / REF_POINTS) + 1.0f) / 2.0f;
}

/**
 * @brief DDS流式数据块填充（填表，但是每次只填一半（看后面调用））
 * 根据不断累加的 dds_phase 动态计算下一个数据块
 */
static void DDS_Generate_Block(uint16_t* pBuffer, uint16_t length) {
    //current_vpp_target在这里起作用影响
    float amp = (current_vpp_target / 3.3f) * 4095.0f;
    float offset = 2048.0f - (amp / 2.0f);
    //上下皆做判断，可以确保从其它函数进来改变的情况不出错
    for (int i = 0; i < length; i++) {
        //!强行取整，高频时波动大
        //todo 线性插值 or 增大REF_POINTS
        /*
        线性插值法展示
        int i = (int)dds_phase;
        float frac = dds_phase - (float)i; // 提取小数部分（0.0 ~ 0.99）

        // 计算两个相邻点的加权平均
        float val = SineRef[i] * (1.0f - frac) + SineRef[(i + 1) % REF_POINTS] * frac;
        val = val * amp + offset;
        */
        int index = (int)dds_phase;
        if(index >= REF_POINTS) index = REF_POINTS - 1;
        else if(index < 0) index = 0;

        float val = SineRef[index] * amp + offset;
        
        if (val > 4095.0f) val = 4095.0f;
        if (val < 0.0f) val = 0.0f;

        pBuffer[i] = (uint16_t)val;
        
        // 关键点：相位在此持续累加，绝对不会断层
        dds_phase += dds_phase_step;
        if (dds_phase >= REF_POINTS) dds_phase -= REF_POINTS;
    }
}

/**
 * @brief 系统启动波形输出
 */
void SignalGen_Start(float init_vpp) {

    //先关，做安全操作，防止从其它地方进来
    HAL_TIM_Base_Stop(&htim2);
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
    
    current_vpp_target = init_vpp;
    dds_phase = 0.0f; // 重置初始相位

    //!初始vpp为0，所以开启输出也为0，流式填表填的也都是0
    // 预先填满整个 DMA 缓冲区
    DDS_Generate_Block(DAC_Buffer, DAC_BUF_SIZE);

    //先填充，后开DMA，避免
    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)DAC_Buffer, DAC_BUF_SIZE, DAC_ALIGN_12B_R);
    HAL_TIM_Base_Start(&htim2);
}


/**
 * @brief 重启波形输出
 */
void SignalGEN_Restart(void) {
    //表的第0位
    dds_phase = 0.0f; 
    __HAL_TIM_SET_COUNTER(&htim2, 0); 
}

/**
 * @brief 修改Vpp。在运行时动态更新Vpp
 */
void SignalGen_UpdateVpp(float new_vpp) {
    // 现在更新 Vpp 非常简单！只需要改写这个变量。
    // CPU 在后台触发的 DMA 中断会实时读取这个新值，自动应用在下半段波形中。
    // 无需调用 __HAL_TIM_DISABLE，也无需 Stop_DMA！波形完美过渡。
    current_vpp_target = new_vpp;
}

/**
 * @brief 修改频率。以1024个检查点为基准，判断一步要走几个检查点
 * @param freq 输出波形的频率
 */
void Set_DDS_Freq(float freq) {
    //DDS（Direct Digital Synthesis）直接数字频率合成
    //相位累加器
    //1000000.0f/freq 为在freq的波里一个周期能采x个点x dot/T，倒数就是 1/x T/dac_dot。每点占据1/x个周期
    //REF_POINTS是 y base_dot/T 每周期有y基准点
    //相乘 y/x base_dot/dac_dot 每个dac点对应几个基准点
    dds_phase_step = (freq / 1000000.0f) * REF_POINTS;  
}


void task2_do(void){
    //通过Hs计算目标Vout的Vpp
    float vpp=Cal_Vin(2.0,5000.0);
    SignalGen_UpdateVpp(vpp);       //更新Vpp
    Set_DDS_Freq(5000.0);           //更新频率
}

//单表上下切换实现流式输出
// 当前半个缓冲被 DAC 发送出去时，触发回调
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
    // DMA 正在发后半段，CPU 赶紧算出接下来的连续 DDS 点填入前半段

    //DAC是会自己输出的，所以这里是改变表的内容
    DDS_Generate_Block(&DAC_Buffer[0], DAC_BUF_SIZE / 2);
}

// 当后半个缓冲也被 DAC 发送出去时，触发回调
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
    // DMA 扭头去发前半段，CPU 赶紧算出接下来的连续 DDS 点填入后半段
    DDS_Generate_Block(&DAC_Buffer[DAC_BUF_SIZE / 2], DAC_BUF_SIZE / 2);
}