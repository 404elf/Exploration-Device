#include "dsp_filter.h"
#include "adc.h"
#include "dac.h"
#include "tim.h"

// H(s)传递函数分母系数(A2*s^2 + A1*s + A0)
#define A2_COEF 2.397e-10f
#define A1_COEF 4.7e-5f
#define A0_COEF 1.0f
#define DC_GAIN 2.0f // 系统增益参数

// 定时器触发的系统ADC/DAC采样率(Hz)
#define SAMPLE_RATE 1000000.0f


static uint16_t ADC_Buffer[FILTER_BUF_SIZE]; // DMA自动写入的ADC采集缓冲区
static uint16_t DAC_Buffer[FILTER_BUF_SIZE]; // DMA自动读出的DAC波形缓冲区域

// 差分方程系统系数：b0~b2对应分子(输入)，a1~a2对应分母(输出)
static float b0 = 0.05f, b1 = 0.10f, b2 = 0.05f;
static float a1 = -1.20f, a2 = 0.40f;

// 历史输入输出数据状态变量(Z域延时)
static float x_n1 = 0.0f, x_n2 = 0.0f;
static float y_n1 = 0.0f, y_n2 = 0.0f;
/**
 * @brief 基于双线性变换法自动计算IIR数字滤波器系数
 */
void Calculate_IIR_Coeffs(void) {
    //  计算常数 K = 2 * Fs（这里使用了简化的双线性置换法实现）
    float K = 2.0f * SAMPLE_RATE;
    float K_sq = K * K;
    
    // 对s进行双线性替换(s = K*(1-z^-1)/(1+z^-1))并展开，提取合并出对应各项中间系数
    float den0 = A2_COEF * K_sq + A1_COEF * K + A0_COEF;
    float den1 = 2.0f * A0_COEF - 2.0f * A2_COEF * K_sq;
    float den2 = A2_COEF * K_sq - A1_COEF * K + A0_COEF;
    
    // 进行归一化处理(除以初项系数den0)后赋值给对应差分方程常系数
    b0 = DC_GAIN / den0;
    b1 = (2.0f * DC_GAIN) / den0;
    b2 = DC_GAIN / den0;
    
    a1 = den1 / den0;
    a2 = den2 / den0;
}


/**
 * @brief 核心DSP滤波操作：执行实时IIR数据运算并更新缓冲
 * @param pIn 指向需处理的输入ADC数据缓冲区的指针
 * @param pOut 欲存放滤波结果的输出DAC缓冲区的指针
 * @param length 单次要求处理的数据点数目
 */
static void Process_IIR_Block(uint16_t* pIn, uint16_t* pOut, uint16_t length) {
    for (int i = 0; i < length; i++) {
        // 提取被转化回实际对应的物理输入电压Vin的ADC数值
        float x_n = (pIn[i] / 4095.0f) * 3.3f;
        
        // 直接执行离散差分方程运算公式（实现IIR滤波核心运算）
        float y_n = b0 * x_n + b1 * x_n1 + b2 * x_n2 - a1 * y_n1 - a2 * y_n2;
        
        // 状态更新（数据在Z域平移延位以备下一个点代入计算）
        x_n2 = x_n1; 
        x_n1 = x_n;
        y_n2 = y_n1; 
        y_n1 = y_n;
        
        // 将数字目标运算模拟电压重转换为12位DAC寄存器控制标度值
        float dac_float = (y_n / 3.3f) * 4095.0f;
        
        // 执行幅度硬限幅处理防止数值越过最大寄存器容量引发系统严重畸变
        if (dac_float > 4095.0f) dac_float = 4095.0f;
        if (dac_float < 0.0f) dac_float = 0.0f;
        
        pOut[i] = (uint16_t)dac_float;
    }
}

/**
 * @brief 初始化滤波状态系统的缓存变量数据（不强制全0可在此设定初指）
 */
void Task4_Filter_Init(void) {
    x_n1 = 0.0f; x_n2 = 0.0f;
    y_n1 = 0.0f; y_n2 = 0.0f;
}

/**
 * @brief 启动系统流：依次打开DAC输出 -> 开启ADC采集启动更新 -> 启动触发系统的心跳源(TIM2定时器)
 */
void Task4_Filter_Start(void) {
    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)DAC_Buffer, FILTER_BUF_SIZE, DAC_ALIGN_12B_R);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Buffer, FILTER_BUF_SIZE);
    HAL_TIM_Base_Start(&htim2); 
}

/**
 * @brief 暂停处理计算系统接口：暂停控制心跳源定时器TIM2 -> 停用ADC采样DMA读取 -> 停止DAC波形输出驱动DMA
 */
void Task4_Filter_Stop(void) {
    HAL_TIM_Base_Stop(&htim2);
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
}

/**
 * @brief 乒乓缓存前半段完成中断(半满)：DMA仍在继续往后半段装入数据，此刻CPU趁机处理刚装好的前半段(0 ~ 99)
 */
void Task4_ADC_HalfCpltCallback(void) {
    Process_IIR_Block(&ADC_Buffer[0], &DAC_Buffer[0], FILTER_BUF_SIZE / 2);
}

/**
 * @brief 乒乓缓存全满完成中断：DMA发生回放绕并开始重写前半段数据，此刻CPU趁机处理刚装好的后半段(100 ~ 199)
 */
void Task4_ADC_FullCpltCallback(void) {
    Process_IIR_Block(&ADC_Buffer[FILTER_BUF_SIZE / 2], &DAC_Buffer[FILTER_BUF_SIZE / 2], FILTER_BUF_SIZE / 2);
}


