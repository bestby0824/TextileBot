/**
* 版权所有(C)
*
* ********
*
* @file Int_Ctrl.c
* @brief
* @details
* @author MuNiu
* @version 1.0.0
* @date xxxx-xx-xx
* @par Description:
* @par Change History:
*
* <日期> | <版本> | <作者> | <描述>
*
* xxxx/xx/xx | 1.0.0 | MuNiu | 创建文件
*
*/


//-------------------- include files ----------------------------------------
#include "stm32f4xx_hal.h"
#include "AngleMath.h"

#define ADC_SampleTime                              ADC_SAMPLETIME_15CYCLES

typedef enum
{
    ADC_CH_Pressure,
    ADC_CH_HW_Ver,
    ADC_CH_TMPE,
    ADC_CH_V_1,
    ADC_CH_V_2,
    ADC_CH_V_3,
    ADC_CH_V_IN,

    ADC_CH_I_1,
    ADC_CH_I_2,
    ADC_CH_I_3,
    ADC_CH_ALL_NUM,
} E_ADC_CH;
#define ADC_CH_RQ_NUM                          ADC_CH_ALL_NUM - 3

/**************************************规则组****************************************/
#define ADC_Pressure_GPIO_PORT                  GPIOA                        //Pressure_In
#define ADC_HW_Ver_GPIO_PORT                    GPIOA                        //HW_Ver_In
#define ADC_TMPE_GPIO_PORT                      GPIOC                        //TMPE_In
#define ADC_V_1_GPIO_PORT                       GPIOC                        //PHASE1_VOTAGE
#define ADC_V_2_GPIO_PORT                       GPIOC                        //PHASE2_VOTAGE
#define ADC_V_3_GPIO_PORT                       GPIOC                        //PHASE3_VOTAGE
#define ADC_V_IN_GPIO_PORT                      GPIOC                        //INPUT_VOLTAGE

#define ADC_Pressure_GPIO_PIN                   GPIO_PIN_0            //Pressure_In      ch0
#define ADC_HW_Ver_GPIO_PIN                     GPIO_PIN_4            //HW_Ver_In        ch4
#define ADC_TMPE_GPIO_PIN                       GPIO_PIN_0            //TMPE_In          ch10
#define ADC_V_1_GPIO_PIN                        GPIO_PIN_1            //PHASE1_VOTAGE    ch11
#define ADC_V_2_GPIO_PIN                        GPIO_PIN_2            //PHASE2_VOTAGE    ch12
#define ADC_V_3_GPIO_PIN                        GPIO_PIN_3            //PHASE3_VOTAGE    ch13
#define ADC_V_IN_GPIO_PIN                       GPIO_PIN_4            //INPUT_VOLTAGE    ch14

#define ADC_RG_CH_LIST  { 0, 4, 10, 11, 12, 13, 14 }        //规则组转换序列

/**************************************注入组****************************************/
#define ADC_I_1_GPIO_PORT                    GPIOB
#define ADC_I_2_GPIO_PORT                    GPIOC
#define ADC_I_3_GPIO_PORT                    GPIOB

#define ADC_I_1_GPIO_PIN                     GPIO_PIN_1
#define ADC_I_2_GPIO_PIN                     GPIO_PIN_5
#define ADC_I_3_GPIO_PIN                     GPIO_PIN_0

#define ADC_I_1_ChSQR                        9
#define ADC_I_2_ChSQR                        15
#define ADC_I_3_ChSQR                        8

#define ADC_GPIO_PORT_LIST  \
{                           \
    ADC_Pressure_GPIO_PORT,\
    ADC_HW_Ver_GPIO_PORT,\
    ADC_TMPE_GPIO_PORT,\
    ADC_V_1_GPIO_PORT,\
    ADC_V_2_GPIO_PORT,\
    ADC_V_3_GPIO_PORT,\
    ADC_V_IN_GPIO_PORT,\
                      \
    ADC_I_1_GPIO_PORT,\
    ADC_I_2_GPIO_PORT,\
    ADC_I_3_GPIO_PORT,\
}

#define ADC_GPIO_PIN_LIST   \
{                           \
    ADC_Pressure_GPIO_PIN,\
    ADC_HW_Ver_GPIO_PIN,\
    ADC_TMPE_GPIO_PIN,\
    ADC_V_1_GPIO_PIN,\
    ADC_V_2_GPIO_PIN,\
    ADC_V_3_GPIO_PIN,\
    ADC_V_IN_GPIO_PIN,\
                     \
    ADC_I_1_GPIO_PIN,\
    ADC_I_2_GPIO_PIN,\
    ADC_I_3_GPIO_PIN,\
}

#define ADC_DMA_Handle                      DMA2
#define ADC_DMA_ClK_ENABLE()                __HAL_RCC_DMA2_CLK_ENABLE()
#define ADC_DMA_STREAM                      DMA2_Stream0
#define ADC_DMA_CHANNEL                     DMA_CHANNEL_0
#define ADC_DMA_IRQ                         DMA2_Stream0_IRQn
#define ADC_DMA_IRQ_Handler                 DMA2_Stream0_IRQHandler
#define ADC_DMA_IRQ_CLEAR()                 DMA2->LIFCR |= 0x3F<<0


void ADC_Init ( void );
static void ADC_BSP_Init ( void );
uint16_t ADC_GetValue ( E_ADC_CH ch );

void ADC_InjectInit ( void );
void HAL_ADCEx_InjectedConvCpltCallback ( ADC_HandleTypeDef* AdcHandle );

extern ADC_HandleTypeDef  hadc1, hadc2;
uint32_t ADC_Get_Result_Average ( uint32_t ch, uint8_t times );

extern _iq CurrentA, CurrentB, CurrentC;










