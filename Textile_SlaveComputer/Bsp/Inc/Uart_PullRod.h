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
#ifndef _Uart_Solenoid__H_
#define _Uart_Solenoid__H_
//-------------------- include files ----------------------------------------
#include "Driver.h"
//-------------------- public definitions -----------------------------------
             
#define Uart_Solenoid_MAX_COMM_LENTH            255

#define Uart_Solenoid_Handle                    USART1
#define Uart_Solenoid_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE()

#define Uart_Solenoid_TX_PIN                    GPIO_PIN_9
#define Uart_Solenoid_TX_GPIO_PORT              GPIOA
#define Uart_Solenoid_RX_PIN                    GPIO_PIN_10
#define Uart_Solenoid_RX_GPIO_PORT              GPIOA

#define Uart_Solenoid_GPIO_AF                   GPIO_AF7_USART1

#define Uart_Solenoid_IRQn                      USART1_IRQn
#define Uart_Solenoid_IRQHandler                USART1_IRQHandler

#define Uart_Solenoid_RX_DMA                    DMA2
#define Uart_Solenoid_RX_DMA_ClK_ENABLE()       __HAL_RCC_DMA2_CLK_ENABLE()
#define Uart_Solenoid_RX_DMA_STREAM             DMA2_Stream2
#define Uart_Solenoid_RX_DMA_CHANNEL            DMA_CHANNEL_4
#define Uart_Solenoid_RX_DMA_IRQ                DMA2_Stream2_IRQn
#define Uart_Solenoid_RX_DMA_IRQ_Handler        DMA2_Stream2_IRQHandler
#define Uart_Solenoid_RX_DMA_IRQ_CLEAR()        DMA2->HIFCR |= 0x3F<<6

#define Uart_Solenoid_TX_DMA                    DMA2
#define Uart_Solenoid_TX_DMA_ClK_ENABLE()       __HAL_RCC_DMA2_CLK_ENABLE()
#define Uart_Solenoid_TX_DMA_STREAM             DMA2_Stream7
#define Uart_Solenoid_TX_DMA_CHANNEL            DMA_CHANNEL_4
#define Uart_Solenoid_TX_DMA_IRQ                DMA2_Stream7_IRQn
#define Uart_Solenoid_TX_DMA_IRQ_Handler        DMA2_Stream7_IRQHandler
#define Uart_Solenoid_TX_DMA_IRQ_CLEAR()        DMA2->HIFCR |= 0x3F<<22
//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
uint8_t Uart_Solenoid_Init(uint32_t baud);
uint8_t Uart_Solenoid_Receive(uint8_t *P,uint16_t *len);
uint8_t Uart_Solenoid_Send(uint8_t *P,uint32_t len);
void Bsp_printf_Uart_Solenoid(char* fmt,...) ;
//-------------------- inline functions -------------------------------------

#endif // _Uart_Solenoid_H_

//-----------------------End of file------------------------------------------
/** @}*/
