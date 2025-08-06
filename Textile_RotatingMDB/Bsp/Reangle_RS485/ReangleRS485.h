
//------------------------------------------------------------------------------

#ifndef ReangleRS485_H_
#define ReangleRS485_H_
//-------------------- include files ----------------------------------------
//#include "Driver.h"
#include "AngleMath.h"
//-------------------- public definitions -----------------------------------
#define ReangleRS485_MAX_COMM_LENTH             255

#define ReangleRS485_Handle                     USART1
#define ReangleRS485_CLK_ENABLE()               __HAL_RCC_USART1_CLK_ENABLE()
#define ReangleRS485_CLK_DISABLE()              __HAL_RCC_USART1_CLK_DISABLE()

#define ReangleRS485_TX_PIN                    GPIO_PIN_6
#define ReangleRS485_TX_GPIO_PORT              GPIOB
#define ReangleRS485_RX_PIN                    GPIO_PIN_7
#define ReangleRS485_RX_GPIO_PORT              GPIOB
#define ReangleRS485_EN_PIN                    GPIO_PIN_12
#define ReangleRS485_EN_GPIO_PORT              GPIOC
#define ReangleRS485_GPIO_AF                   GPIO_AF7_USART1

#define ReangleRS485_IRQn                      USART1_IRQn
#define ReangleRS485_IRQHandler                USART1_IRQHandler

#define ReangleRS485_RX_DMA                    DMA2
#define ReangleRS485_RX_DMA_ClK_ENABLE()       __HAL_RCC_DMA2_CLK_ENABLE()
#define ReangleRS485_RX_DMA_STREAM             DMA2_Stream5
#define ReangleRS485_RX_DMA_CHANNEL            DMA_CHANNEL_4
#define ReangleRS485_RX_DMA_IRQ                DMA2_Stream5_IRQn
#define ReangleRS485_RX_DMA_IRQ_Handler        DMA2_Stream5_IRQHandler
#define ReangleRS485_RX_DMA_IRQ_CLEAR()        DMA2->HIFCR |= 0x3F<<6

#define ReangleRS485_TX_DMA                    DMA2
#define ReangleRS485_TX_DMA_ClK_ENABLE()       __HAL_RCC_DMA2_CLK_ENABLE()
#define ReangleRS485_TX_DMA_STREAM             DMA2_Stream7
#define ReangleRS485_TX_DMA_CHANNEL            DMA_CHANNEL_4
#define ReangleRS485_TX_DMA_IRQ                DMA2_Stream7_IRQn
#define ReangleRS485_TX_DMA_IRQ_Handler        DMA2_Stream7_IRQHandler
#define ReangleRS485_TX_DMA_IRQ_CLEAR()        DMA2->HIFCR |= 0x3F<<22

//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
uint8_t ReangleRS485_Init(uint32_t baud);
uint8_t ReangleRS485_Receive(uint8_t *P,uint16_t *len);
uint8_t ReangleRS485_Send(uint8_t *P,uint32_t len);
void ReangleRS485_Send_Process(void);
_iq GetReangleValue ( void );
//-------------------- inline functions -------------------------------------

#endif /* ReangleRS485_H_ */
//-----------------------End of file------------------------------------------
/** @}*/
