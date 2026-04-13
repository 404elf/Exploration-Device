#include "dsp_filter.h"
#include "adc.h"
#include "dac.h"
#include "tim.h"

// H(s) coefficient£¨ÏµÊý£©
#define A2_COEF 2.397e-10f
#define A1_COEF 4.7e-5f
#define A0_COEF 1.0f
#define DC_GAIN 2.0f //dB

//Sampling rate
#define SAMPLE_RATE 500000.0f


uint16_t ADC_Buffer[FILTER_BUF_SIZE]; // DMA auto
uint16_t DAC_Buffer[FILTER_BUF_SIZE]; // DMA atuo

//Placeholder default
static float b0 = 0.05f, b1 = 0.10f, b2 = 0.05f;
static float a1 = -1.20f, a2 = 0.40f;

// Past input and output
static float x_n1 = 0.0f, x_n2 = 0.0f;
static float y_n1 = 0.0f, y_n2 = 0.0f;
/**
 * @brief auto calculate Bilinear transformation
 */
void Calculate_IIR_Coeffs(void) {
    //Constant K = 2 * Fs
    float K = 2.0f * SAMPLE_RATE;
    float K_sq = K * K;
    
    //Intermediate variables = K*(1-z^-1)/(1+z^-1)
    float den0 = A2_COEF * K_sq + A1_COEF * K + A0_COEF;
    float den1 = 2.0f * A0_COEF - 2.0f * A2_COEF * K_sq;
    float den2 = A2_COEF * K_sq - A1_COEF * K + A0_COEF;
    
    //Normalize and assign
    b0 = DC_GAIN / den0;
    b1 = (2.0f * DC_GAIN) / den0;
    b2 = DC_GAIN / den0;
    
    a1 = den1 / den0;
    a2 = den2 / den0;
}


/**
 * @brief DSP£ºProcess data
 * @param pIn: input ADC Array pointer
 * @param pOut: output DAC Array pointer
 * @param length: Number of points processed
 */
static void Process_IIR_Block(uint16_t* pIn, uint16_t* pOut, uint16_t length) {
    for (int i = 0; i < length; i++) {
        //adc value transform real Vin that mcu accept
        float x_n = (pIn[i] / 4095.0f) * 3.3f;
        
        // Difference Equation Calculation
        float y_n = b0 * x_n + b1 * x_n1 + b2 * x_n2 - a1 * y_n1 - a2 * y_n2;
        
        //update value
        x_n2 = x_n1; 
        x_n1 = x_n;
        y_n2 = y_n1; 
        y_n1 = y_n;
        
        //value of DAC
        float dac_float = (y_n / 3.3f) * 4095.0f;
        
        // Amplitude(·ù¶È) Limiting
        if (dac_float > 4095.0f) dac_float = 4095.0f;
        if (dac_float < 0.0f) dac_float = 0.0f;
        
        pOut[i] = (uint16_t)dac_float;
    }
}

/**
 * @brief Initialize Filter and clear IIR coefficient
 */
void Task4_Filter_Init(void) {
    x_n1 = 0.0f; x_n2 = 0.0f;
    y_n1 = 0.0f; y_n2 = 0.0f;
}

/**
 * @brief in order:dac open->adc open-> timer2 open 
 */
void Task4_Filter_Start(void) {
    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)DAC_Buffer, FILTER_BUF_SIZE, DAC_ALIGN_12B_R);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Buffer, FILTER_BUF_SIZE);
    HAL_TIM_Base_Start(&htim2); 
}

/**
 * @brief in order:timer2 stop -> dac stop -> adc stop
 */
void Task4_Filter_Stop(void) {
    HAL_TIM_Base_Stop(&htim2);
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
}

/**
 *  @brief DMA run in Second half,cpu run in first half(0 ~ 99)
 */
void Task4_ADC_HalfCpltCallback(void) {
    Process_IIR_Block(&ADC_Buffer[0], &DAC_Buffer[0], FILTER_BUF_SIZE / 2);
}

/**
 * @brief DMA run in first half,cpu run in Second half(100 ~ 199)
 */
void Task4_ADC_FullCpltCallback(void) {
    Process_IIR_Block(&ADC_Buffer[FILTER_BUF_SIZE / 2], &DAC_Buffer[FILTER_BUF_SIZE / 2], FILTER_BUF_SIZE / 2);
}
