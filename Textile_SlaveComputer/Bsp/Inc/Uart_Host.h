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
#ifndef _Uart_Host__H_
#define _Uart_Host__H_
//-------------------- include files ----------------------------------------
#include "Driver.h"
//-------------------- public definitions -----------------------------------

#define Uart_Host_MAX_COMM_LENTH            255

#define Uart_Host_Handle                    UART4
#define Uart_Host_CLK_ENABLE()              __HAL_RCC_UART4_CLK_ENABLE()

#define Uart_Host_TX_PIN                    GPIO_PIN_0
#define Uart_Host_TX_GPIO_PORT              GPIOA
#define Uart_Host_RX_PIN                    GPIO_PIN_1
#define Uart_Host_RX_GPIO_PORT              GPIOA
#define Uart_Host_GPIO_AF                   GPIO_AF8_UART4

#define Uart_Host_IRQn                      UART4_IRQn
#define Uart_Host_IRQHandler                UART4_IRQHandler

#define Uart_Host_RX_DMA                    DMA1
#define Uart_Host_RX_DMA_ClK_ENABLE()       __HAL_RCC_DMA1_CLK_ENABLE()
#define Uart_Host_RX_DMA_STREAM             DMA1_Stream2
#define Uart_Host_RX_DMA_CHANNEL            DMA_CHANNEL_4
#define Uart_Host_RX_DMA_IRQ                DMA1_Stream2_IRQn
#define Uart_Host_RX_DMA_IRQ_Handler        DMA1_Stream2_IRQHandler
#define Uart_Host_RX_DMA_IRQ_CLEAR()        DMA1->LIFCR |= 0x3F<<16

#define Uart_Host_TX_DMA                    DMA1
#define Uart_Host_TX_DMA_ClK_ENABLE()       __HAL_RCC_DMA1_CLK_ENABLE()
#define Uart_Host_TX_DMA_STREAM             DMA1_Stream4
#define Uart_Host_TX_DMA_CHANNEL            DMA_CHANNEL_4
#define Uart_Host_TX_DMA_IRQ                DMA1_Stream4_IRQn
#define Uart_Host_TX_DMA_IRQ_Handler        DMA1_Stream4_IRQHandler
#define Uart_Host_TX_DMA_IRQ_CLEAR()        DMA1->HIFCR |= 0x3F
//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
uint8_t Uart_Host_Init(uint32_t baud);
uint8_t Uart_Host_Receive(uint8_t *P,uint16_t *len);
uint8_t Uart_Host_Send(uint8_t *P,uint32_t len);
void Bsp_printf_Host(char* fmt,...) ;
//-------------------- inline functions -------------------------------------

#endif // _Uart_Host_H_

//-----------------------End of file------------------------------------------
/** @}*/
