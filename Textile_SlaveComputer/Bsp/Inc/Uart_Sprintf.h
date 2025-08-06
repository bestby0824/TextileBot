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
#ifndef _Uart_Sprintf__H_
#define _Uart_Sprintf__H_
//-------------------- include files ----------------------------------------
#include "Driver.h"
//-------------------- public definitions -----------------------------------

#define Uart_Sprintf_MAX_COMM_LENTH            255

#define Uart_Sprintf_Handle                    USART2
#define Uart_Sprintf_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE()

#define Uart_Sprintf_TX_PIN                    GPIO_PIN_2
#define Uart_Sprintf_TX_GPIO_PORT              GPIOA
#define Uart_Sprintf_RX_PIN                    GPIO_PIN_3
#define Uart_Sprintf_RX_GPIO_PORT              GPIOA
#define Uart_Sprintf_GPIO_AF                   GPIO_AF7_USART2

#define Uart_Sprintf_IRQn                      USART2_IRQn
#define Uart_Sprintf_IRQHandler                USART2_IRQHandler

#define Uart_Sprintf_RX_DMA                    DMA1
#define Uart_Sprintf_RX_DMA_ClK_ENABLE()       __HAL_RCC_DMA1_CLK_ENABLE()
#define Uart_Sprintf_RX_DMA_STREAM             DMA1_Stream5
#define Uart_Sprintf_RX_DMA_CHANNEL            DMA_CHANNEL_4
#define Uart_Sprintf_RX_DMA_IRQ                DMA1_Stream5_IRQn
#define Uart_Sprintf_RX_DMA_IRQ_Handler        DMA1_Stream5_IRQHandler
#define Uart_Sprintf_RX_DMA_IRQ_CLEAR()        DMA1->HIFCR |= 0x3F<<6

#define Uart_Sprintf_TX_DMA                    DMA1
#define Uart_Sprintf_TX_DMA_ClK_ENABLE()       __HAL_RCC_DMA1_CLK_ENABLE()
#define Uart_Sprintf_TX_DMA_STREAM             DMA1_Stream6
#define Uart_Sprintf_TX_DMA_CHANNEL            DMA_CHANNEL_4
#define Uart_Sprintf_TX_DMA_IRQ                DMA1_Stream6_IRQn
#define Uart_Sprintf_TX_DMA_IRQ_Handler        DMA1_Stream6_IRQHandler
#define Uart_Sprintf_TX_DMA_IRQ_CLEAR()        DMA1->HIFCR |= 0x3F<<16
//-------------------- public data ------------------------------------------
extern uint8_t g_SprintfTxBuf[Uart_Sprintf_MAX_COMM_LENTH];
//-------------------- public functions -------------------------------------
uint8_t Uart_Sprintf_Init(uint32_t baud);
uint8_t Uart_Sprintf_Receive(uint8_t *P,uint16_t *len);
uint8_t Uart_Sprintf_Send(uint8_t *P,uint32_t len);

//-------------------- inline functions -------------------------------------

#endif // _Uart_Sprintf_H_

//-----------------------End of file------------------------------------------
/** @}*/
