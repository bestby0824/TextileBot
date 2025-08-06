/**
* ��Ȩ����(C)
*
* ********
*
* @file
* @brief
* @details
* @author HWW
* @version 1.0.0
* @date xxxx-xx-xx
* @par Description:
* @par Change History:
*
* <����> | <�汾> | <����> | <����>
*
* xxxx/xx/xx | 1.0.0 | HWW | �����ļ�
*
*/
//------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
#ifndef _ADC__H_
#define _ADC__H_
//-------------------- include files ----------------------------------------
#include "Driver.h"
#include "AngleMath.h"
//-------------------- public definitions -----------------------------------
#define ADC_SampleTime                              ADC_SAMPLETIME_56CYCLES

typedef enum
{
    ADC_CH_Version = 0,                             //PC0      ADC123_IN10          0
    ADC_CH_SENSOR_24V_V,                            //PC2      ADC123_IN12          1
    ADC_CH_TEMP_IN,                                 //PC3      ADC123_IN13          2
    ADC_CH_PC_24V_V,                                //PC4      ADC12_IN14           3
    ADC_CH_PC_24V_I,                                //PC5      ADC12_IN15           4
    ADC_CH_2_5V_REF,                                //PB0      ADC12_IN8            5
    ADC_CH_M_I_MCU,                                 //PB1      ADC12_IN9            6
    ADC_CH_MOTOR1_24V_V,                            //PA4      ADC12_IN4            7
    ADC_CH_MOTOR2_24V_V,                            //PA5      ADC12_IN5            8

    ADC_CH_0_Stop_V,                                //PF7       ADC3_IN5            9
    ADC_CH_SENSOR_24V_I,                            //PF8       ADC3_IN6            10
    ADC_CH_MOTOR1_24V_I,                            //PF9      ADC3_IN7             11
    ADC_CH_MOTOR2_24V_I,                            //PF10      ADC3_IN8            12
    ADC_RQ_CH_NUM,
} E_ADC_CH;


/**************************************������****************************************/
#define ADC_Version_GPIO_PORT               GPIOC                               //Ӳ���汾�ż��
#define ADC_SENSOR_24V_V_GPIO_PORT          GPIOC                               //��������Դ��ѹ���
#define ADC_TEMP_IN_GPIO_PORT               GPIOC                               //�¶ȼ��
#define ADC_PC_24V_V_GPIO_PORT              GPIOC                               //��λ����Դ��ѹ���
#define ADC_PC_24V_I_GPIO_PORT              GPIOC                               //��λ����Դ�������
#define ADC_2_5V_REF_GPIO_PORT              GPIOB                               //2.5V�ο�
#define ADC_M_I_MCU_GPIO_PORT               GPIOB                               //�������������
#define ADC_MOTOR1_24V_V_GPIO_PORT          GPIOA                               //���1��Դ��ѹ���
#define ADC_MOTOR2_24V_V_GPIO_PORT          GPIOA                               //���2��Դ��ѹ���

#define ADC_0Stop_V_GPIO_PORT               GPIOF                               //��0�༱ͣ��ѹ��⣩
#define ADC_SENSOR_24V_I_GPIO_PORT          GPIOF                               //��������Դ�������
#define ADC_MOTOR1_24V_I_GPIO_PORT          GPIOF                               //���1��Դ�������
#define ADC_MOTOR2_24V_I_GPIO_PORT          GPIOF                               //���2��Դ�������

#define ADC_Version_GPIO_PIN                GPIO_PIN_0                          //Ӳ���汾�ż��
#define ADC_SENSOR_24V_V_GPIO_PIN           GPIO_PIN_2                          //��������Դ��ѹ���
#define ADC_TEMP_IN_GPIO_PIN                GPIO_PIN_3                          //�¶ȼ��
#define ADC_PC_24V_V_GPIO_PIN               GPIO_PIN_4                          //��λ����Դ��ѹ���
#define ADC_PC_24V_I_GPIO_PIN               GPIO_PIN_5                          //��λ����Դ�������
#define ADC_2_5V_REF_GPIO_PIN               GPIO_PIN_0                          //2.5V�ο�
#define ADC_M_I_MCU_GPIO_PIN                GPIO_PIN_1                          //�������������
#define ADC_MOTOR1_24V_V_GPIO_PIN           GPIO_PIN_4                          //���1��Դ��ѹ���
#define ADC_MOTOR2_24V_V_GPIO_PIN           GPIO_PIN_5                          //���2��Դ��ѹ���

#define ADC_0Stop_V_GPIO_PIN                GPIO_PIN_7                          //��0�༱ͣ��ѹ��⣩
#define ADC_SENSOR_24V_I_GPIO_PIN           GPIO_PIN_8                          //��������Դ�������
#define ADC_MOTOR1_24V_I_GPIO_PIN           GPIO_PIN_9                          //���1��Դ�������
#define ADC_MOTOR2_24V_I_GPIO_PIN           GPIO_PIN_10                         //���2��Դ�������

#define ADC_Version_ChSQR                           (10)
#define ADC_SENSOR_24V_V_ChSQR                      (12)
#define ADC_TEMP_IN_ChSQR                           (13)
#define ADC_PC_24V_V_ChSQR                          (14)
#define ADC_PC_24V_I_ChSQR                          (15)
#define ADC_2_5V_REF_ChSQR                          (8)
#define ADC_M_I_MCU_ChSQR                           (9)
#define ADC_MOTOR1_24V_V_ChSQR                      (4)
#define ADC_MOTOR2_24V_V_ChSQR                      (5)

#define ADC_0Stop_V_ChSQR                           (5)
#define ADC_SENSOR_24V_I_ChSQR                      (6)
#define ADC_MOTOR1_24V_I_ChSQR                      (7)
#define ADC_MOTOR2_24V_I_ChSQR                      (8)

#define ADC_GPIO_PORT_LIST  \
{                           \
    ADC_Version_GPIO_PORT,\
    ADC_SENSOR_24V_V_GPIO_PORT,\
    ADC_TEMP_IN_GPIO_PORT,\
    ADC_PC_24V_V_GPIO_PORT,\
    ADC_PC_24V_I_GPIO_PORT,\
    ADC_2_5V_REF_GPIO_PORT,\
    ADC_M_I_MCU_GPIO_PORT,\
    ADC_MOTOR1_24V_V_GPIO_PORT,\
    ADC_MOTOR2_24V_V_GPIO_PORT,\
    ADC_0Stop_V_GPIO_PORT,\
    ADC_SENSOR_24V_I_GPIO_PORT,\
    ADC_MOTOR1_24V_I_GPIO_PORT,\
    ADC_MOTOR2_24V_I_GPIO_PORT,\
}

#define ADC_GPIO_PIN_LIST   \
{                           \
    ADC_Version_GPIO_PIN,\
    ADC_SENSOR_24V_V_GPIO_PIN,\
    ADC_TEMP_IN_GPIO_PIN,\
    ADC_PC_24V_V_GPIO_PIN,\
    ADC_PC_24V_I_GPIO_PIN,\
    ADC_2_5V_REF_GPIO_PIN,\
    ADC_M_I_MCU_GPIO_PIN,\
    ADC_MOTOR1_24V_V_GPIO_PIN,\
    ADC_MOTOR2_24V_V_GPIO_PIN,\
    ADC_0Stop_V_GPIO_PIN,\
    ADC_SENSOR_24V_I_GPIO_PIN,\
    ADC_MOTOR1_24V_I_GPIO_PIN,\
    ADC_MOTOR2_24V_I_GPIO_PIN,\
}

#define ADC_DMA_Handle                      DMA2
#define ADC_DMA_ClK_ENABLE()                __HAL_RCC_DMA2_CLK_ENABLE()
#define ADC_DMA_STREAM                      DMA2_Stream0
#define ADC_DMA_CHANNEL                     DMA_CHANNEL_2
#define ADC_DMA_IRQ                         DMA2_Stream0_IRQn
#define ADC_DMA_IRQ_Handler                 DMA2_Stream0_IRQHandler
#define ADC_DMA_IRQ_CLEAR()                 DMA2->LIFCR |= 0x3F<<0

#define ADC_DMA_Handle                      DMA2
#define ADC_DMA_ClK_ENABLE()                __HAL_RCC_DMA2_CLK_ENABLE()
#define ADC_DMA_STREAM_2                      DMA2_Stream4
#define ADC_DMA_CHANNEL_2                     DMA_CHANNEL_0
#define ADC_DMA_IRQ_2                         DMA2_Stream4_IRQn
#define ADC_DMA_IRQ_Handler_2                 DMA2_Stream4_IRQHandler
#define ADC_DMA_IRQ_CLEAR_2()                 DMA2->HIFCR |= 0x3F<<0


#define ADC_RG_CH_LIST  {\
    ADC_Version_ChSQR,\
    ADC_SENSOR_24V_V_ChSQR,\
    ADC_TEMP_IN_ChSQR,\
    ADC_PC_24V_V_ChSQR,\
    ADC_PC_24V_I_ChSQR,\
    ADC_2_5V_REF_ChSQR,\
    ADC_M_I_MCU_ChSQR,\
    ADC_MOTOR1_24V_V_ChSQR,\
    ADC_MOTOR2_24V_V_ChSQR,\
    ADC_0Stop_V_ChSQR,\
    ADC_SENSOR_24V_I_ChSQR,\
    ADC_MOTOR1_24V_I_ChSQR,\
    ADC_MOTOR2_24V_I_ChSQR,\
}   //������ת������

//-------------------- public functions -------------------------------------
void ADC_SWStart ( void );

void Adc_Init ( void );
uint16_t ADC_GetValue ( E_ADC_CH ch );

#endif // _XXX_H_

//-----------------------End of file------------------------------------------
/** @}*/
