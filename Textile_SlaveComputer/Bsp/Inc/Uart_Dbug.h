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
#ifndef _Uart_Dbug__H_
#define _Uart_Dbug__H_
//-------------------- include files ----------------------------------------
#include "Driver.h"
//-------------------- public definitions -----------------------------------

#define Uart_Dbug_MAX_COMM_LENTH            255

#define Uart_Dbug_Handle                    UART5
#define Uart_Dbug_CLK_ENABLE()              __HAL_RCC_UART5_CLK_ENABLE()

#define Uart_Dbug_TX_PIN                    GPIO_PIN_12
#define Uart_Dbug_TX_GPIO_PORT              GPIOC
#define Uart_Dbug_RX_PIN                    GPIO_PIN_2
#define Uart_Dbug_RX_GPIO_PORT              GPIOD
#define Uart_Dbug_GPIO_AF                   GPIO_AF8_UART5

#define Uart_Dbug_IRQn                      UART5_IRQn
#define Uart_Dbug_IRQHandler                UART5_IRQHandler

#define Uart_Dbug_RX_DMA                    DMA1
#define Uart_Dbug_RX_DMA_ClK_ENABLE()       __HAL_RCC_DMA1_CLK_ENABLE()
#define Uart_Dbug_RX_DMA_STREAM             DMA1_Stream0
#define Uart_Dbug_RX_DMA_CHANNEL            DMA_CHANNEL_4
#define Uart_Dbug_RX_DMA_IRQ                DMA1_Stream0_IRQn
#define Uart_Dbug_RX_DMA_IRQ_Handler        DMA1_Stream0_IRQHandler
#define Uart_Dbug_RX_DMA_IRQ_CLEAR()        DMA1->LIFCR |= 0x3F

#define Uart_Dbug_TX_DMA                    DMA1
#define Uart_Dbug_TX_DMA_ClK_ENABLE()       __HAL_RCC_DMA1_CLK_ENABLE()
#define Uart_Dbug_TX_DMA_STREAM             DMA1_Stream7
#define Uart_Dbug_TX_DMA_CHANNEL            DMA_CHANNEL_4
#define Uart_Dbug_TX_DMA_IRQ                DMA1_Stream7_IRQn
#define Uart_Dbug_TX_DMA_IRQ_Handler        DMA1_Stream7_IRQHandler
#define Uart_Dbug_TX_DMA_IRQ_CLEAR()        DMA1->HIFCR |= 0xF4<<20
//-------------------- public data ------------------------------------------
extern uint8_t g_DbugTxBuf[Uart_Dbug_MAX_COMM_LENTH];
//-------------------- public functions -------------------------------------
uint8_t Uart_Dbug_Init(uint32_t baud);
uint8_t Uart_Dbug_Receive(uint8_t *P,uint16_t *len);
uint8_t Uart_Dbug_Send(uint8_t *P,uint32_t len);
void Bsp_printf(char* fmt,...) ;

//-------------------- inline functions -------------------------------------

#endif // _Uart_Dbug_H_

//-----------------------End of file------------------------------------------
/** @}*/
