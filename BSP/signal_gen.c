#include "signal_gen.h"
#include "math.h"
#include "dac.h"
#include "tim.h"

//Internal variable - in Memory
////static uint16_t SineTable[SINE_SAMPLES];
uint16_t SineTable_A[SINE_SAMPLES];
uint16_t SineTable_B[SINE_SAMPLES];
uint16_t *pCurrentTable = SineTable_A; // it 's turn to A.

/**
 * @brief calculate Vin
 * @param Vout: How much Vout do you want?
 * @param freq: as if you see
 * @retval Vin
 */
float Cal_Vin(float Vout,float freq){
    // rad/s
    float omega = 2.0f * PI * freq;

    //formula param 
    //Denominator£ºas^2+bs+1
    float a = 2.397e-10f;
    float b = 4.7e-5f;
    
    //real+imag*j
    float real_part = 1.0f - a * omega*omega;
    float imag_part = b * omega;

    //Modulus length
    float Denominator_Len = sqrtf(real_part*real_part + imag_part*imag_part);
    if (Denominator_Len < 1e-6f) Denominator_Len = 1e-6f;   //maybe small

    float Hs_Len=2.0f/Denominator_Len;
    
    float Vin = Vout / Hs_Len;
    //maybe Error£¨£¨Calibration£¿£©
    Vin = Vin / 1.0f;
    return Vin;
}

/**
 *  @brief  generate table for sine of Vin
 *  @param vpp_target: goal Vpp(0.0V-3.3V)
 */
void SignalGen_InitTable(float vpp_target){
    float amplitude = (vpp_target / 3.3f) * 4095.0f/2.0f;
    for  (int i=0;i<SINE_SAMPLES;i++){
        //Center : 2048
        float val = 2048.0f + amplitude * sinf(2.0f * PI * i/SINE_SAMPLES);

        //Vpp overflow Protection
        if (val>4095.0f) val = 4095.0f;
        if (val < 0.0f) val = 0.0f;

        SineTable_A[i] = (uint16_t)val;   //RG need uint
    }
}

/**
 *  @brief  generate table for sine of Vin
 *  @param pTable: Specify the table
 *  @param vpp_target: goal Vpp(0.0V-3.3V)
 */
static void SignalGen_FillTable(uint16_t* pTable, float vpp_target){
    float amplitude = (vpp_target / 3.3f) * 4095.0f/2.0f;
    for  (int i=0;i<SINE_SAMPLES;i++){
        //Center : 2048
        float val = 2048.0f + amplitude * sinf(2.0f * PI * i/SINE_SAMPLES);

        //Vpp overflow Protection
        if (val>4095.0f) val = 4095.0f;
        if (val < 0.0f) val = 0.0f;

        pTable[i] = (uint16_t)val;   //RG need uint
    }
}

/**
 * @brief  start timer and DAC-DMA transfer
 */
void SignalGen_Start(void){
    //start DMA of DAC
    HAL_DAC_Start_DMA(&hdac,DAC1_CHANNEL_1,(uint32_t*)pCurrentTable,SINE_SAMPLES,DAC_ALIGN_12B_R);

    //Start timer
    HAL_TIM_Base_Start(&htim6);
}


//! when it 's time,stop->wait->start.
//? maybe waitting time is too long?it 's due to no output(Distortion)
/**
 * @brief restart DAC-DMA Transfer(from 0 begin)
 */
void SignalGEN_Restart(void){
    //to stop timer
    __HAL_TIM_DISABLE(&htim6);
    
    //must stop and start,or restart is no response.
    HAL_DAC_Stop_DMA(&hdac,DAC_CHANNEL_1);
    
    //andthen restart
    HAL_DAC_Start_DMA(&hdac,DAC1_CHANNEL_1,(uint32_t*)pCurrentTable,SINE_SAMPLES,DAC_ALIGN_12B_R);
    
    //clear timer
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    //let's go with timer
    __HAL_TIM_ENABLE(&htim6);
}

/**
 * @brief Modify Vpp in RUNNING
 */
void SignalGen_UpdateVpp(float new_vpp){
    //it 's turn to who?
    uint16_t *pNextTable = (pCurrentTable == SineTable_A) ? SineTable_B : SineTable_A;

    SignalGen_FillTable(pNextTable, new_vpp);
    
    //to stop timer(RG)
    __HAL_TIM_DISABLE(&htim6);

    //stop transfer
    HAL_DAC_Stop_DMA(&hdac,DAC_CHANNEL_1);  //DAC and DMA stop together

    pCurrentTable = pNextTable;
    //renew transfer
    ////SignalGen_Start();  
    //maybe try to delete "start timer" 
    HAL_DAC_Start_DMA(&hdac,DAC1_CHANNEL_1,(uint32_t*)pCurrentTable,SINE_SAMPLES,DAC_ALIGN_12B_R);
    
    //Phase Alignment
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    //start timer
    __HAL_TIM_ENABLE(&htim6);
}

