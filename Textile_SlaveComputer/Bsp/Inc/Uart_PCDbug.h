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
#ifndef _Uart_PCDbug__H_
#define _Uart_PCDbug__H_
//-------------------- include files ----------------------------------------
#include "Driver.h"
//-------------------- public definitions -----------------------------------

#define Uart_PCDbug_MAX_COMM_LENTH           255

#define Uart_PCDbug_Handle                   USART3
#define Uart_PCDbug_CLK_ENABLE()             __HAL_RCC_USART3_CLK_ENABLE()

#define Uart_PCDbug_TX_PIN                    GPIO_PIN_8
#define Uart_PCDbug_TX_GPIO_PORT              GPIOD
#define Uart_PCDbug_RX_PIN                    GPIO_PIN_9
#define Uart_PCDbug_RX_GPIO_PORT              GPIOD

#define Uart_PCDbug_GPIO_AF                   GPIO_AF7_USART3

#define Uart_PCDbug_IRQn                      USART3_IRQn
#define Uart_PCDbug_IRQHandler                USART3_IRQHandler

#define Uart_PCDbug_RX_DMA                    DMA1
#define Uart_PCDbug_RX_DMA_ClK_ENABLE()       __HAL_RCC_DMA1_CLK_ENABLE()
#define Uart_PCDbug_RX_DMA_STREAM            DMA1_Stream1
#define Uart_PCDbug_RX_DMA_CHANNEL           DMA_CHANNEL_4
#define Uart_PCDbug_RX_DMA_IRQ               DMA1_Stream1_IRQn
#define Uart_PCDbug_RX_DMA_IRQ_Handler       DMA1_Stream1_IRQHandler
#define Uart_PCDbug_RX_DMA_IRQ_CLEAR()       DMA1->LIFCR |= 0x3F<<6

#define Uart_PCDbug_TX_DMA                   DMA1
#define Uart_PCDbug_TX_DMA_ClK_ENABLE()      __HAL_RCC_DMA1_CLK_ENABLE()
#define Uart_PCDbug_TX_DMA_STREAM            DMA1_Stream3
#define Uart_PCDbug_TX_DMA_CHANNEL           DMA_CHANNEL_4
#define Uart_PCDbug_TX_DMA_IRQ               DMA1_Stream3_IRQn
#define Uart_PCDbug_TX_DMA_IRQ_Handler       DMA1_Stream3_IRQHandler
#define Uart_PCDbug_TX_DMA_IRQ_CLEAR()       DMA1->LIFCR |= 0x3F<<22
//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
uint8_t Uart_PCDbug_Init(uint32_t baud);
uint8_t Uart_PCDbug_Receive(uint8_t *P,uint16_t *len);
uint8_t Uart_PCDbug_Send(uint8_t *P,uint32_t len);
void Bsp_printf_PCDbug(char* fmt,...) ;
//-------------------- inline functions -------------------------------------

#endif // _Uart_PCDbug_PCDbug_H_

//-----------------------End of file------------------------------------------
/** @}*/
