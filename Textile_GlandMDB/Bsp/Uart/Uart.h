
//------------------------------------------------------------------------------

#ifndef Uart_H_
#define Uart_H_
//-------------------- include files ----------------------------------------
#include "stm32f4xx_hal.h"

//-------------------- public definitions -----------------------------------

#define Uart_MAX_COMM_LENTH						255

#define Uart_Handle										USART2
#define Uart_CLK_ENABLE()						 	__HAL_RCC_USART2_CLK_ENABLE()

#define Uart_TX_PIN                    GPIO_PIN_2
#define Uart_TX_GPIO_PORT              GPIOA
#define Uart_RX_PIN                    GPIO_PIN_3
#define Uart_RX_GPIO_PORT              GPIOA
#define Uart_GPIO_AF                   GPIO_AF7_USART2

#define Uart_IRQn                      USART2_IRQn
#define Uart_IRQHandler                USART2_IRQHandler

#define Uart_RX_DMA										DMA1
#define Uart_RX_DMA_ClK_ENABLE()			__HAL_RCC_DMA1_CLK_ENABLE()
#define Uart_RX_DMA_STREAM            DMA1_Stream5
#define Uart_RX_DMA_CHANNEL  				  DMA_CHANNEL_4
#define Uart_RX_DMA_IRQ  							DMA1_Stream5_IRQn
#define Uart_RX_DMA_IRQ_Handler  			DMA1_Stream5_IRQHandler
#define Uart_RX_DMA_IRQ_CLEAR()       DMA1->HIFCR |= 0x3F<<6

#define Uart_TX_DMA										DMA1
#define Uart_TX_DMA_ClK_ENABLE()			__HAL_RCC_DMA1_CLK_ENABLE()
#define Uart_TX_DMA_STREAM            DMA1_Stream6
#define Uart_TX_DMA_CHANNEL  					DMA_CHANNEL_4
#define Uart_TX_DMA_IRQ  							DMA1_Stream6_IRQn
#define Uart_TX_DMA_IRQ_Handler  			DMA1_Stream6_IRQHandler
#define Uart_TX_DMA_IRQ_CLEAR()       DMA1->HIFCR |= 0x3F<<16
//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
uint8_t Uart_Init(uint32_t baud);
uint8_t Uart_Receive(uint8_t *P,uint16_t *len);
uint8_t Uart_Send(uint8_t *P,uint32_t len);
void Bsp_printf(char* fmt,...) ;
//-------------------- inline functions -------------------------------------

#endif /* Uart_H_ */
//-----------------------End of file------------------------------------------
/** @}*/
