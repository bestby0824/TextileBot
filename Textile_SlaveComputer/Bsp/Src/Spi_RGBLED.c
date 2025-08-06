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
/*

T0H  0.22~0.38uS
T0L  0.58~1.0uS

T1H  0.58~1.0uS
T1L  0.22~0.38uS

Reset   280uS

1bit:    0.29uS~0.38us   取中:0.335(2985075)
0码:     100
1码:     110
Reset:   105字节 = 280/0.335/8

*/
//-------------------- include files ----------------------------------------
#include "Spi_RGBLED.h"
#include "Xint.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
static uint8_t   g_pu8SpiRGBRxBuf[1];
static uint32_t  g_u32RxLen = 0;
static uint32_t  g_u32RxCplFlag = 0;
static uint32_t  g_u32TxLen = 0;
static uint32_t  g_u32TxIndex = 0;

uint8_t   g_pu8SpiRGBTxBuf[Spi_RgbLed_MAX_COMM_LENTH];
static uint8_t   u8RgbLedTxMutex = 0;

SPI_HandleTypeDef hspi_RgbLed;
DMA_HandleTypeDef hdma_spi_RgbLed_tx;
DMA_HandleTypeDef hdma_spi_RgbLed_rx;
//-------------------- private functions declare ----------------------------
static uint16_t Spi_RgbLed_GetBrrByBaud ( USART_TypeDef*, uint32_t Baud );

static uint32_t NormalByte2BusByte ( uint8_t In );
//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------

/*! \fn				uint8_t Spi_RgbLed_Init(uint32_t baud)
 *  \brief 		Initializes the Spi_RgbLed device.
 *  \param 		baud rate
 *  \return 	0:success other:false
 */
uint8_t Spi_RgbLed_Init ()
{
    Spi_RgbLed_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct;

    hspi_RgbLed.Instance = Spi_RgbLed_Handle;
    hspi_RgbLed.Init.Mode = SPI_MODE_MASTER;
    hspi_RgbLed.Init.Direction = SPI_DIRECTION_2LINES;
    hspi_RgbLed.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi_RgbLed.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi_RgbLed.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi_RgbLed.Init.NSS = SPI_NSS_SOFT;
    hspi_RgbLed.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi_RgbLed.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi_RgbLed.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi_RgbLed.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi_RgbLed.Init.CRCPolynomial = 10;


    if ( HAL_SPI_Init ( &hspi_RgbLed ) != HAL_OK )
    {
        return 1;
    }
    Spi_RgbLed_Handle ->CR2 |= 0x01 << SPI_CR2_TXDMAEN_Pos;
    Spi_RgbLed_Handle ->CR1 |= 0x01 << SPI_CR1_SPE_Pos;
    Spi_RgbLed_Handle ->CR2 |= 0x01 << SPI_CR2_TXEIE_Pos;
    /* USER CODE BEGIN Spi_RgbLed_Handle_MspInit 0 */

    /* USER CODE END Spi_RgbLed_Handle_MspInit 0 */
    /* Peripheral clock enable */
    Spi_RgbLed_CLK_ENABLE();

    /**Spi_RgbLed_Handle GPIO Configuration
    PA10     ------> SPI1_SCK
    PA14     ------> SPI1_MISO
    PB15     ------> SPI1_MOSI
    */
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = Spi_RgbLed_GPIO_AF;

//    GPIO_InitStruct.Pin = Spi_RgbLed_TX_PIN;
//    HAL_GPIO_Init ( Spi_RgbLed_TX_GPIO_PORT, &GPIO_InitStruct );

//    GPIO_InitStruct.Pin = Spi_RgbLed_RX_PIN;
//    HAL_GPIO_Init ( Spi_RgbLed_RX_GPIO_PORT, &GPIO_InitStruct );

//    GPIO_InitStruct.Pin = Spi_RgbLed_CLK_PIN;
//    HAL_GPIO_Init ( Spi_RgbLed_CLK_GPIO_PORT, &GPIO_InitStruct );

    /* Peripheral DMA init*/

#if (Spi_DMA_EN)
    Spi_RgbLed_Handle ->CR2 |= 0x01 << SPI_CR2_TXDMAEN_Pos;
    Spi_RgbLed_Handle ->CR2 |= 0x01 << SPI_CR2_RXDMAEN_Pos;
#endif


    /* USER CODE END Spi_RgbLed_Handle_MspInit 0 */
    /* Peripheral clock enable */
    Spi_RgbLed_CLK_ENABLE();


#if (Spi_DMA_EN)
    /* Peripheral DMA init*/
    Spi_RgbLed_TX_DMA_ClK_ENABLE();
    Spi_RgbLed_RX_DMA_ClK_ENABLE();

    /***********************************RX DMA config*******************************/
    Spi_RgbLed_RX_DMA_STREAM->CR = 0;
    Spi_RgbLed_RX_DMA_STREAM->CR = Spi_RgbLed_RX_DMA_CHANNEL            //通道
                                   | 0x00 << DMA_SxCR_MBURST_Pos
                                   | 0x00 << DMA_SxCR_PBURST_Pos
                                   | 0x00 << DMA_SxCR_CT_Pos
                                   | 0x00 << DMA_SxCR_DBM_Pos
                                   | 0x01 << DMA_SxCR_PL_Pos         //通道优先级 中
                                   | 0x00 << DMA_SxCR_PINCOS_Pos
                                   | 0x00 << DMA_SxCR_MSIZE_Pos      //存储器数据宽度 8位
                                   | 0x00 << DMA_SxCR_PSIZE_Pos      //外设数据宽度 8位
                                   | 0x01 << DMA_SxCR_MINC_Pos       //存储器地址递增
                                   | 0x00 << DMA_SxCR_PINC_Pos       //外设地址不递增
                                   | 0x00 << DMA_SxCR_CIRC_Pos       //非循环模式
                                   | 0x00 << DMA_SxCR_DIR_Pos        //外设到存储器
                                   | 0x00 << DMA_SxCR_PFCTRL_Pos
                                   | 0x00 << DMA_SxCR_TCIE_Pos       //传输完成中断
                                   | 0x00 << DMA_SxCR_HTIE_Pos
                                   | 0x01 << DMA_SxCR_TEIE_Pos       //传输错误中断
                                   | 0x00 << DMA_SxCR_DMEIE_Pos
                                   | 0x00 << DMA_SxCR_EN_Pos;                   //通道不使能

    Spi_RgbLed_RX_DMA_STREAM->M0AR = ( uint32_t ) g_pu8SpiRGBRxBuf;             //存储器地址
    Spi_RgbLed_RX_DMA_STREAM->PAR = ( uint32_t ) &Spi_RgbLed_Handle->DR; //外设地址
    Spi_RgbLed_RX_DMA_STREAM->NDTR = Spi_RgbLed_MAX_COMM_LENTH;                //数据传输数量
    Spi_RgbLed_RX_DMA_STREAM->FCR =   0x01 << DMA_SxFCR_FEIE_Pos //FIFO错误中断
                                      | 0x00 << DMA_SxFCR_DMDIS_Pos; //直接模式


    HAL_NVIC_SetPriority ( Spi_RgbLed_RX_DMA_IRQ, IRQ_Priority_UART, 0 );
    HAL_NVIC_EnableIRQ ( Spi_RgbLed_RX_DMA_IRQ );

    Spi_RgbLed_RX_DMA_STREAM->CR |=  0x01; //使能通道



    /***********************************TX DMA config*******************************/

    Spi_RgbLed_TX_DMA_STREAM->CR = 0;
    Spi_RgbLed_TX_DMA_STREAM->CR = Spi_RgbLed_TX_DMA_CHANNEL            //通道
                                   | 0x00 << DMA_SxCR_MBURST_Pos
                                   | 0x00 << DMA_SxCR_PBURST_Pos
                                   | 0x00 << DMA_SxCR_CT_Pos
                                   | 0x00 << DMA_SxCR_DBM_Pos
                                   | 0x01 << DMA_SxCR_PL_Pos         //通道优先级 中
                                   | 0x00 << DMA_SxCR_PINCOS_Pos
                                   | 0x00 << DMA_SxCR_MSIZE_Pos      //存储器数据宽度 8位
                                   | 0x00 << DMA_SxCR_PSIZE_Pos      //外设数据宽度 8位
                                   | 0x01 << DMA_SxCR_MINC_Pos       //存储器地址递增
                                   | 0x00 << DMA_SxCR_PINC_Pos       //外设地址不递增
                                   | 0x00 << DMA_SxCR_CIRC_Pos       //非循环模式
                                   | 0x01 << DMA_SxCR_DIR_Pos        //存储器到外设
                                   | 0x00 << DMA_SxCR_PFCTRL_Pos
                                   | 0x01 << DMA_SxCR_TCIE_Pos       //传输完成中断
                                   | 0x00 << DMA_SxCR_HTIE_Pos
                                   | 0x01 << DMA_SxCR_TEIE_Pos       //传输错误中断
                                   | 0x00 << DMA_SxCR_DMEIE_Pos
                                   | 0x00 << DMA_SxCR_EN_Pos;                    //通道不使能

    Spi_RgbLed_TX_DMA_STREAM->M0AR = ( uint32_t ) g_pu8SpiRGBTxBuf;                //存储器地址
    Spi_RgbLed_TX_DMA_STREAM->PAR  = ( uint32_t ) &Spi_RgbLed_Handle->DR;    //外设地址
    Spi_RgbLed_TX_DMA_STREAM->NDTR = 0;                  //数据传输数量
    Spi_RgbLed_TX_DMA_STREAM->FCR  =   0x01 << DMA_SxFCR_FEIE_Pos //FIFO错误中断
                                       | 0x00 << DMA_SxFCR_DMDIS_Pos; //直接模式

    HAL_NVIC_SetPriority ( Spi_RgbLed_TX_DMA_IRQ, IRQ_Priority_UART, 0 );
    HAL_NVIC_EnableIRQ ( Spi_RgbLed_TX_DMA_IRQ );

    /* DMA controller clock enable */
    Spi_RgbLed_TX_DMA_ClK_ENABLE();

    /* DMA interrupt init */
    /* DMA2_Stream3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority ( Spi_RgbLed_TX_DMA_IRQ, 0, 0 );
    HAL_NVIC_EnableIRQ ( Spi_RgbLed_TX_DMA_IRQ );
#else
    HAL_NVIC_SetPriority ( Spi_RgbLed_IRQn, IRQ_Priority_SPI2, 0 );
    HAL_NVIC_EnableIRQ ( Spi_RgbLed_IRQn );
#endif

    return 0;
}
/*! \fn         void Spi_RgbLed_IRQHandler(void)
 *  \brief      UART interrupt processing
 *  \param      none
 *  \return     none
 */
void Spi_RgbLed_IRQHandler ( void )
{
//    if ( Spi_RgbLed_Handle ->SR & SPI_SR_RXNE )
//    {

//#if (Spi_DMA_EN)
//        g_u32RxLen = Spi_RgbLed_MAX_COMM_LENTH - Spi_RgbLed_RX_DMA_STREAM->NDTR;
//        Spi_RgbLed_RX_DMA_STREAM->CR &= ~0x01;
//        Spi_RgbLed_RX_DMA_IRQ_CLEAR();
//#else

//        g_u32RxLen ++;
//        g_pu8SpiRGBRxBuf[g_u32RxLen - 1 ] = Spi_RgbLed_Handle ->DR;
//#endif
//        g_u32RxCplFlag = 1;
//    }
//    else
    if ( Spi_RgbLed_Handle ->SR & SPI_SR_TXE )
    {
#if (Spi_DMA_EN)
        u8RgbLedTxMutex = 0;
#else
        if ( g_u32TxIndex < g_u32TxLen )
        {
            Spi_RgbLed_Handle ->DR = g_pu8SpiRGBTxBuf[g_u32TxIndex];
            g_u32TxIndex++;
        } else {
            u8RgbLedTxMutex = 0;
            Spi_RgbLed_Handle ->CR2 &= ~SPI_CR2_TXEIE;
        }
#endif
    }
    NVIC_ClearPendingIRQ ( Spi_RgbLed_IRQn );

}

#if (Spi_DMA_EN)
/*! \fn         void Spi_RgbLed_RX_DMA_IRQ_Handler(void)
 *  \brief      The DMA correspond to the UART RX interrupt processing
 *  \param      none
 *  \return     none
 */
void Spi_RgbLed_RX_DMA_IRQ_Handler ( void )
{
    Spi_RgbLed_RX_DMA_IRQ_CLEAR();

    g_u32RxCplFlag = 0;
    g_u32RxLen = 0;

    Spi_RgbLed_RX_DMA_STREAM->CR  &= ~0x01;
    Spi_RgbLed_RX_DMA_STREAM->NDTR = Spi_RgbLed_MAX_COMM_LENTH;
    Spi_RgbLed_RX_DMA_STREAM->CR  |= 0x01;
}

/*! \fn         void Spi_RgbLed_TX_DMA_IRQ_Handler(void)
 *  \brief      The DMA correspond to the UART TX interrupt processing
 *  \param      none
 *  \return     none
 */
void Spi_RgbLed_TX_DMA_IRQ_Handler ( void )
{
    Spi_RgbLed_TX_DMA_IRQ_CLEAR();
}
#endif

/*! \fn         uint8_t Spi_RgbLed_Receive(uint8_t *P,uint32_t *len)
 *  \brief      UART receive from  g_pu8SpiRGBRxBuf
 *  \param      Pointer to destination pointer
 *  \param      Pointer to lenth
 *  \return     1:One frame of data is received  0;none data
 */
uint8_t Spi_RgbLed_Receive ( uint8_t *P, uint16_t *len )
{
    uint8_t ret;

    if ( g_u32RxCplFlag )
    {
        memcpy ( P, g_pu8SpiRGBRxBuf, g_u32RxLen );

        *len = g_u32RxLen;
        g_u32RxCplFlag = 0;
        g_u32RxLen = 0;
#if (Spi_DMA_EN)
        Spi_RgbLed_RX_DMA_STREAM->CR  &= ~0x01;
        while ( ( Spi_RgbLed_RX_DMA_STREAM->CR & 0x01 ) == 1 );
        Spi_RgbLed_RX_DMA_STREAM->NDTR = Spi_RgbLed_MAX_COMM_LENTH;
        Spi_RgbLed_RX_DMA_STREAM->CR  |= 0x01;
#endif
        ret = 1;
    }
    else
    {
        *len = 0;
        ret = 0;
    }

    return ret;
}






/*! \fn				static uint8_t Spi_RgbLed_Send(uint8_t *P,uint32_t len,uint32_t tick)
 *  \brief 		UART send data
 *  \param 		Pointer to source buff
 *  \param 		Pointer to lenth
 *  \return 	0:success  other;failed
 */
uint8_t Spi_RgbLed_Send ( uint8_t *P, uint32_t len )
{
    uint16_t ret, i;
    uint32_t u32Byte_WS2812;

    if ( ( len > 0 ) && ( len * 3 <= Spi_RgbLed_MAX_COMM_LENTH ) )
    {
        if ( Spi_RgbLed_Handle->SR & ( 1 << SPI_SR_TXE_Pos ) )
        {
            //----------------数据转换，普通串口数据转换为WS2812格式------------------

            for ( i = 0; i < RgbLed_IdleBytes; i++ )
            {
                g_pu8SpiRGBTxBuf[i]  = 0;
            }
            for ( i = 0; i < len; i++ )
            {
                u32Byte_WS2812 = NormalByte2BusByte ( P[i] );
                g_pu8SpiRGBTxBuf[RgbLed_IdleBytes + i * 3]     = ( u32Byte_WS2812 >> 16 ) & 0x000000FF; //高位，先发送
                g_pu8SpiRGBTxBuf[RgbLed_IdleBytes + i * 3 + 1] = ( u32Byte_WS2812 >> 8 ) & 0x000000FF;
                g_pu8SpiRGBTxBuf[RgbLed_IdleBytes + i * 3 + 2] = ( u32Byte_WS2812 ) & 0x000000FF;
            }
            //--------------------------------------------------
            g_u32TxIndex = 0;
            g_u32TxLen = len * 3;

#if (Spi_DMA_EN)
            Spi_RgbLed_TX_DMA_STREAM->CR &=  ~0x1;
            while ( ( Spi_RgbLed_TX_DMA_STREAM->CR & 0x01 ) == 1 );
            Spi_RgbLed_TX_DMA_STREAM->NDTR = Spi_RgbLed_MAX_COMM_LENTH;  //数据传输数量
            Spi_RgbLed_TX_DMA_STREAM->CR |=  0x1;  //通道使能
#else
            Spi_RgbLed_Handle ->CR2 |= SPI_CR2_TXEIE;
#endif
            u8RgbLedTxMutex = 1;
            ret = 0;
        }
        else
        {
            ret = 1;
            u8RgbLedTxMutex = 0;
        }
    }
    else
    {
        ret = 2;
    }
    ret = 0;
    return ret;
}


//-------------------- private functions ------------------------------------

static uint32_t NormalByte2BusByte ( uint8_t In )
{
    uint8_t i;
    uint32_t Out;

    for ( i = 0, Out = 0; i < 8; i++ )
    {
        Out |= ( ( In << i ) & 0x80 ) ? 0b110 : 0b100;
        if ( i < 7 )
        {
            Out <<= 3;
        }
    }
    return Out;
}
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


