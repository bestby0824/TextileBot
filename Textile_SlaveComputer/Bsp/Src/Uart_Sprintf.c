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

//-------------------- include files ----------------------------------------
#include "Uart_Sprintf.h"
#include "Xint.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
static uint8_t   pu8RxBuf[Uart_Sprintf_MAX_COMM_LENTH];
static uint32_t  g_u32RxLen = 0;
static uint32_t  g_u32RxCplFlag = 0;

static uint8_t   g_pu8TxBuf[Uart_Sprintf_MAX_COMM_LENTH];
static uint8_t   u8DbugTxMutex = 0;

//-------------------- private functions declare ----------------------------
static uint16_t Uart_Sprintf_GetBrrByBaud ( USART_TypeDef*, uint32_t Baud );
static void DMA_Init ( void );
//-------------------- public data ------------------------------------------

uint8_t g_SprintfTxBuf[Uart_Sprintf_MAX_COMM_LENTH];
//-------------------- public functions -------------------------------------


/*! \fn				uint8_t Uart_Sprintf_Init(uint32_t baud)
 *  \brief 		Initializes the Uart_Sprintf device.
 *  \param 		baud rate
 *  \return 	0:success other:false
 */
uint8_t Uart_Sprintf_Init ( uint32_t baud )
{
    GPIO_InitTypeDef            GPIO_InitStruct;
    uint8_t TXBuf[32];
    Uart_Sprintf_CLK_ENABLE();

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin       = Uart_Sprintf_TX_PIN;
    GPIO_InitStruct.Alternate = Uart_Sprintf_GPIO_AF;
    HAL_GPIO_Init ( Uart_Sprintf_TX_GPIO_PORT, &GPIO_InitStruct );

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = Uart_Sprintf_RX_PIN;
    GPIO_InitStruct.Alternate = Uart_Sprintf_GPIO_AF;
    HAL_GPIO_Init ( Uart_Sprintf_RX_GPIO_PORT, &GPIO_InitStruct );


    Uart_Sprintf_Handle->CR1 = 0;
    Uart_Sprintf_Handle->BRR  = Uart_Sprintf_GetBrrByBaud ( Uart_Sprintf_Handle, baud );
    Uart_Sprintf_Handle->CR1 &= 0xFFFF0000;
    Uart_Sprintf_Handle->CR2 &= 0xFFFF8090;
    Uart_Sprintf_Handle->CR3 &= 0xFFFFF000;
    Uart_Sprintf_Handle->CR1 |= ( 1 << 2 ) | ( 1 << 3 ) | ( 1 << 4 ) | ( 1 << 6 ); //2:接收使能 3:发送使能 4:空闲中断 6:发送完成中断
    Uart_Sprintf_Handle->CR3 |= ( 1 << 6 ) | ( 1 << 7 ); //6:DMA接收使能 7:DMA发送使能
    Uart_Sprintf_Handle->CR1 |= ( 1 << 13 ); // 13:串口使能

    HAL_NVIC_SetPriority ( Uart_Sprintf_IRQn, IRQ_Priority_UART, 0 );
    HAL_NVIC_EnableIRQ ( Uart_Sprintf_IRQn );

    DMA_Init();

    //sprintf((char *)TXBuf,"Uart_Sprintf_Send\r\n");
    //Uart_Sprintf_Send((uint8_t *) TXBuf, 16);

    return 0;
}

/*! \fn				void Uart_Sprintf_IRQHandler(void)
 *  \brief 		UART interrupt processing
 *  \param 		none
 *  \return 	none
 */
void Uart_Sprintf_IRQHandler ( void )
{
    uint32_t sr;
    uint32_t data;

    NVIC_ClearPendingIRQ ( Uart_Sprintf_IRQn );

    sr = Uart_Sprintf_Handle->SR;
    data = Uart_Sprintf_Handle->DR;
    ( void ) data;

    if ( sr & ( 1 << 4 ) ) //总线空闲(接收完成)
    {
        g_u32RxCplFlag = 1;
        g_u32RxLen = Uart_Sprintf_MAX_COMM_LENTH - Uart_Sprintf_RX_DMA_STREAM->NDTR;
        Uart_Sprintf_RX_DMA_STREAM->CR &= ~0x01;
        Uart_Sprintf_RX_DMA_IRQ_CLEAR();
    }

    if ( sr & ( 1 << 6 ) ) //发送完成
    {
        Uart_Sprintf_Handle->SR &= ~ ( 1 << 6 );
        u8DbugTxMutex = 0;
    }
}

/*! \fn				void Uart_Sprintf_RX_DMA_IRQ_Handler(void)
 *  \brief 		The DMA correspond to the UART RX interrupt processing
 *  \param 		none
 *  \return 	none
 */
void Uart_Sprintf_RX_DMA_IRQ_Handler ( void )
{
    Uart_Sprintf_RX_DMA_IRQ_CLEAR();

    g_u32RxCplFlag = 0;
    g_u32RxLen = 0;

    Uart_Sprintf_RX_DMA_STREAM->CR  &= ~0x01;
    Uart_Sprintf_RX_DMA_STREAM->NDTR = Uart_Sprintf_MAX_COMM_LENTH;
    Uart_Sprintf_RX_DMA_STREAM->CR  |= 0x01;
}

/*! \fn				void Uart_Sprintf_TX_DMA_IRQ_Handler(void)
 *  \brief 		The DMA correspond to the UART TX interrupt processing
 *  \param 		none
 *  \return 	none
 */
void Uart_Sprintf_TX_DMA_IRQ_Handler ( void )
{
    Uart_Sprintf_TX_DMA_IRQ_CLEAR();
}

/*! \fn				uint8_t Uart_Sprintf_Receive(uint8_t *P,uint32_t *len)
 *  \brief 		UART receive from  pu8RxBuf
 *  \param 		Pointer to destination pointer
 *  \param 		Pointer to lenth
 *  \return 	1:One frame of data is received  0;none data
 */
uint8_t Uart_Sprintf_Receive ( uint8_t *P, uint16_t *len )
{
    uint8_t ret;

    if ( g_u32RxCplFlag )
    {
        if ( g_u32RxLen > Uart_Sprintf_MAX_COMM_LENTH ) g_u32RxLen = Uart_Sprintf_MAX_COMM_LENTH;
        memcpy ( P, pu8RxBuf, g_u32RxLen );
        *len = g_u32RxLen;

        g_u32RxCplFlag = 0;
        g_u32RxLen = 0;

        Uart_Sprintf_RX_DMA_STREAM->CR  &= ~0x01;
        while ( ( Uart_Sprintf_RX_DMA_STREAM->CR & 0x01 ) == 1 );
        Uart_Sprintf_RX_DMA_STREAM->NDTR = Uart_Sprintf_MAX_COMM_LENTH;
        Uart_Sprintf_RX_DMA_STREAM->CR  |= 0x01;

        ret = 1;
    }
    else
    {
        *len = 0;
        ret = 0;
    }

    return ret;
}

/*! \fn				static uint8_t Uart_Sprintf_Send(uint8_t *P,uint32_t len,uint32_t tick)
 *  \brief 		UART send data
 *  \param 		Pointer to source buff
 *  \param 		Pointer to lenth
 *  \return 	0:success  other;failed
 */
uint8_t Uart_Sprintf_Send ( uint8_t *P, uint32_t len )
{
    uint8_t ret;

    if ( ( len > 0 ) || ( len <= Uart_Sprintf_MAX_COMM_LENTH ) )
    {
        if ( u8DbugTxMutex == 0 )
        {
            memcpy ( g_pu8TxBuf, P, len );

            Uart_Sprintf_TX_DMA_STREAM->CR &=  ~0x1;
            while ( ( Uart_Sprintf_TX_DMA_STREAM->CR & 0x01 ) == 1 );
            Uart_Sprintf_TX_DMA_STREAM->NDTR = len;  //数据传输数量
            Uart_Sprintf_TX_DMA_STREAM->CR |=  0x1;  //通道使能

            u8DbugTxMutex = 1;
            ret = 0;
        }
        else
        {
            ret = 1;
            u8DbugTxMutex = 0;
        }
    }
    else
    {
        ret = 2;
    }

    return ret;
}



//-------------------- private functions ------------------------------------

static void DMA_Init ( void )
{
    Uart_Sprintf_TX_DMA_ClK_ENABLE();
    Uart_Sprintf_RX_DMA_ClK_ENABLE();

    /***********************************RX DMA config*******************************/
    Uart_Sprintf_RX_DMA_STREAM->CR = 0;
    Uart_Sprintf_RX_DMA_STREAM->CR = Uart_Sprintf_RX_DMA_CHANNEL            //通道
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
                                  | 0x00 << DMA_SxCR_EN_Pos;        //通道不使能

    Uart_Sprintf_RX_DMA_STREAM->M0AR = ( uint32_t ) pu8RxBuf;          //存储器地址
    Uart_Sprintf_RX_DMA_STREAM->PAR = ( uint32_t ) &Uart_Sprintf_Handle->DR;	//外设地址
    Uart_Sprintf_RX_DMA_STREAM->NDTR = Uart_Sprintf_MAX_COMM_LENTH;       //数据传输数量
    Uart_Sprintf_RX_DMA_STREAM->FCR =   0x01 << DMA_SxFCR_FEIE_Pos //FIFO错误中断
                                     | 0x00 << DMA_SxFCR_DMDIS_Pos; //直接模式


    HAL_NVIC_SetPriority ( Uart_Sprintf_RX_DMA_IRQ, IRQ_Priority_UART, 0 );
    HAL_NVIC_EnableIRQ ( Uart_Sprintf_RX_DMA_IRQ );

    Uart_Sprintf_RX_DMA_STREAM->CR |=  0x01; //使能通道

    /***********************************TX DMA config*******************************/

    Uart_Sprintf_TX_DMA_STREAM->CR = 0;
    Uart_Sprintf_TX_DMA_STREAM->CR = Uart_Sprintf_RX_DMA_CHANNEL            //通道
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
                                  | 0x00 << DMA_SxCR_EN_Pos;//通道不使能

    Uart_Sprintf_TX_DMA_STREAM->M0AR = ( uint32_t ) g_pu8TxBuf;//存储器地址
    Uart_Sprintf_TX_DMA_STREAM->PAR  = ( uint32_t ) &Uart_Sprintf_Handle->DR;	//外设地址
    Uart_Sprintf_TX_DMA_STREAM->NDTR = 0;                  //数据传输数量
    Uart_Sprintf_TX_DMA_STREAM->FCR  =   0x01 << DMA_SxFCR_FEIE_Pos //FIFO错误中断
                                      | 0x00 << DMA_SxFCR_DMDIS_Pos; //直接模式

    HAL_NVIC_SetPriority ( Uart_Sprintf_TX_DMA_IRQ, IRQ_Priority_UART, 0 );
    HAL_NVIC_EnableIRQ ( Uart_Sprintf_TX_DMA_IRQ );

//	Uart_Sprintf_TX_DMA_STREAM->CR |=  0x01; //使能通道
}

/*! \fn				static uint16_t Uart_Sprintf_GetBrrByBaud(USART_TypeDef* UsartType,uint32_t Baud)
 *  \brief 		Get brr data by baud
 *  \param 		USARTx channel
 *  \param 		baud
 *  \return 	brr data
 */
static uint16_t Uart_Sprintf_GetBrrByBaud ( USART_TypeDef* UsartType, uint32_t Baud )
{
    uint16_t u16Brr;

    if ( ( UsartType == USART1 ) || ( UsartType == USART6 ) ) //APB2总线上的串口
    {
        u16Brr = SystemCoreClock / 2 / Baud;
    }
    else
    {
        u16Brr = SystemCoreClock / 4 / Baud;
    }

    return u16Brr;
}
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


