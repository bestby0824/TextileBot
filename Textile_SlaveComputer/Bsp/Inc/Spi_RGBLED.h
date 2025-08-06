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
#ifndef _Spi_RgbLed__H_
#define _Spi_RgbLed__H_
//-------------------- include files ----------------------------------------
#include "Driver.h"
//-------------------- public definitions -----------------------------------
#define Spi_DMA_EN                          0

#define RgbLed_Num                           100
#define RgbLed_IdleBytes                     ((280 *3 +50*3)>>3)
#define Spi_RgbLed_MAX_COMM_LENTH            (RgbLed_Num *3 *3 + RgbLed_IdleBytes)
//每个RGB灯三个字节(24bits)，每个总线bit需要3个串口bit

#define Spi_RgbLed_Handle                    SPI2
#define Spi_RgbLed_CLK_ENABLE()              __HAL_RCC_SPI2_CLK_ENABLE()

#define Spi_RgbLed_TX_PIN                    GPIO_PIN_15
#define Spi_RgbLed_TX_GPIO_PORT              GPIOB
#define Spi_RgbLed_RX_PIN                    GPIO_PIN_14
#define Spi_RgbLed_RX_GPIO_PORT              GPIOB
#define Spi_RgbLed_CLK_PIN                   GPIO_PIN_10
#define Spi_RgbLed_CLK_GPIO_PORT             GPIOB

#define Spi_RgbLed_GPIO_AF                   GPIO_AF5_SPI2

#define Spi_RgbLed_IRQn                      SPI2_IRQn
#define Spi_RgbLed_IRQHandler                SPI2_IRQHandler

#define Spi_RgbLed_RX_DMA                    DMA1
#define Spi_RgbLed_RX_DMA_ClK_ENABLE()       __HAL_RCC_DMA1_CLK_ENABLE()
#define Spi_RgbLed_RX_DMA_STREAM             DMA1_Stream3
#define Spi_RgbLed_RX_DMA_CHANNEL            DMA_CHANNEL_0
#define Spi_RgbLed_RX_DMA_IRQ                DMA1_Stream3_IRQn
#define Spi_RgbLed_RX_DMA_IRQ_Handler        DMA1_Stream3_IRQHandler
#define Spi_RgbLed_RX_DMA_IRQ_CLEAR()        DMA1->LIFCR |= 0x3F<<22

#define Spi_RgbLed_TX_DMA                    DMA1
#define Spi_RgbLed_TX_DMA_ClK_ENABLE()       __HAL_RCC_DMA1_CLK_ENABLE()
#define Spi_RgbLed_TX_DMA_STREAM             DMA1_Stream4
#define Spi_RgbLed_TX_DMA_CHANNEL            DMA_CHANNEL_0
#define Spi_RgbLed_TX_DMA_IRQ                DMA1_Stream4_IRQn
#define Spi_RgbLed_TX_DMA_IRQ_Handler        DMA1_Stream4_IRQHandler
#define Spi_RgbLed_TX_DMA_IRQ_CLEAR()        DMA1->HIFCR |= 0x3F
/*
    L   H   <<
    0   4   0
    1   5   6
    2   6   16
    3   7   22
*/

//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
uint8_t Spi_RgbLed_Init ( );
uint8_t Spi_RgbLed_Receive ( uint8_t *P, uint16_t *len );
uint8_t Spi_RgbLed_Send ( uint8_t *P, uint32_t len );
void Bsp_printf_Spi_Solenoid ( char* fmt, ... ) ;
//-------------------- inline functions -------------------------------------

#endif // _Uart_RgbLed_H_

//-----------------------End of file------------------------------------------
/** @}*/
