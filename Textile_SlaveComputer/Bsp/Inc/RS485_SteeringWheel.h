/**
* 版权所有(C) 
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
* <日期> | <版本> | <作者> | <描述>
*
* xxxx/xx/xx | 1.0.0 | HWW | 创建文件
*
*/
//------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
#ifndef _RS485SteeringWheel__H_
#define _RS485SteeringWheel__H_
//-------------------- include files ----------------------------------------
#include "Driver.h"
//-------------------- public definitions -----------------------------------

#define RS485_SteeringWheel_MAX_COMM_LENTH            255

#define RS485_SteeringWheel_Handle                    USART6
#define RS485_SteeringWheel_CLK_ENABLE()              __HAL_RCC_USART6_CLK_ENABLE()

#define RS485_SteeringWheel_TX_PIN                    GPIO_PIN_6
#define RS485_SteeringWheel_TX_GPIO_PORT              GPIOC
#define RS485_SteeringWheel_RX_PIN                    GPIO_PIN_7
#define RS485_SteeringWheel_RX_GPIO_PORT              GPIOC
#define RS485_SteeringWheel_EN_PIN                    GPIO_PIN_8
#define RS485_SteeringWheel_EN_GPIO_PORT              GPIOC

#define RS485_SteeringWheel_GPIO_AF                   GPIO_AF8_USART6

#define RS485_SteeringWheel_IRQn                      USART6_IRQn
#define RS485_SteeringWheel_IRQHandler                USART6_IRQHandler

#define RS485_SteeringWheel_RX_DMA                    DMA2
#define RS485_SteeringWheel_RX_DMA_ClK_ENABLE()       __HAL_RCC_DMA2_CLK_ENABLE()
#define RS485_SteeringWheel_RX_DMA_STREAM             DMA2_Stream1
#define RS485_SteeringWheel_RX_DMA_CHANNEL            DMA_CHANNEL_5
#define RS485_SteeringWheel_RX_DMA_IRQ                DMA2_Stream1_IRQn
#define RS485_SteeringWheel_RX_DMA_IRQ_Handler        DMA2_Stream1_IRQHandler
#define RS485_SteeringWheel_RX_DMA_IRQ_CLEAR()        DMA2->LIFCR |= 0x3F<<16

#define RS485_SteeringWheel_TX_DMA                    DMA2
#define RS485_SteeringWheel_TX_DMA_ClK_ENABLE()       __HAL_RCC_DMA2_CLK_ENABLE()
#define RS485_SteeringWheel_TX_DMA_STREAM             DMA2_Stream6
#define RS485_SteeringWheel_TX_DMA_CHANNEL            DMA_CHANNEL_5
#define RS485_SteeringWheel_TX_DMA_IRQ                DMA2_Stream6_IRQn
#define RS485_SteeringWheel_TX_DMA_IRQ_Handler        DMA2_Stream6_IRQHandler
#define RS485_SteeringWheel_TX_DMA_IRQ_CLEAR()        DMA2->HIFCR |= 0x3F<<16
//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
uint8_t RS485_SteeringWheel_Init(uint32_t baud);
uint8_t RS485_SteeringWheel_Receive(uint8_t *P,uint16_t *len);
uint8_t RS485_SteeringWheel_Send(uint8_t *P,uint32_t len);
void Bsp_printf_RS485_SteeringWheel(char* fmt,...) ;
//-------------------- inline functions -------------------------------------

#endif // _RS485SteeringWheel_H_

//-----------------------End of file------------------------------------------
/** @}*/
