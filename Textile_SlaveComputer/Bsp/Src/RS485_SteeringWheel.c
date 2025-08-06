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
#include "RS485_SteeringWheel.h"
#include "Xint.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "Monitor.h"
#include "StateCtrl.h"
//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
static uint8_t   g_pu8RxBuf[RS485_SteeringWheel_MAX_COMM_LENTH];
static uint32_t  g_u32RxLen = 0;
static uint32_t  g_u32RxCplFlag = 0;

static uint8_t   g_pu8TxBuf[RS485_SteeringWheel_MAX_COMM_LENTH];
static uint8_t	 u8WheelTxMutex = 0;
int16_t  m_nSteeringLife = -1;                   //转向电机板通信心跳

//-------------------- private functions declare ----------------------------
static uint16_t RS485_SteeringWheel_GetBrrByBaud ( USART_TypeDef*, uint32_t Baud );
static void DMA_Init ( void );
//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------

/*! \fn				uint8_t RS485_SteeringWheel_Init(uint32_t baud)
 *  \brief 		Initializes the RS485_SteeringWheel device.
 *  \param 		baud rate
 *  \return 	0:success other:false
 */
uint8_t RS485_SteeringWheel_Init ( uint32_t baud )
{
    GPIO_InitTypeDef  		GPIO_InitStruct;
    uint8_t TXBuf[32];
    RS485_SteeringWheel_CLK_ENABLE();

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin       = RS485_SteeringWheel_TX_PIN;
    GPIO_InitStruct.Alternate = RS485_SteeringWheel_GPIO_AF;
    HAL_GPIO_Init ( RS485_SteeringWheel_TX_GPIO_PORT, &GPIO_InitStruct );

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin       = RS485_SteeringWheel_RX_PIN;
    GPIO_InitStruct.Alternate = RS485_SteeringWheel_GPIO_AF;
    HAL_GPIO_Init ( RS485_SteeringWheel_RX_GPIO_PORT, &GPIO_InitStruct );

    GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin       = RS485_SteeringWheel_EN_PIN;

    HAL_GPIO_Init ( RS485_SteeringWheel_EN_GPIO_PORT, &GPIO_InitStruct );

    HAL_GPIO_WritePin ( RS485_SteeringWheel_EN_GPIO_PORT, RS485_SteeringWheel_EN_PIN, GPIO_PIN_RESET );

    RS485_SteeringWheel_Handle->CR1 = 0;
    RS485_SteeringWheel_Handle->BRR  = RS485_SteeringWheel_GetBrrByBaud ( RS485_SteeringWheel_Handle, baud );
    RS485_SteeringWheel_Handle->CR1 &= 0xFFFF0000;
    RS485_SteeringWheel_Handle->CR2 &= 0xFFFF8090;
    RS485_SteeringWheel_Handle->CR3 &= 0xFFFFF000;
    RS485_SteeringWheel_Handle->CR1 |= ( 1 << 2 ) | ( 1 << 3 ) | ( 1 << 4 ) | ( 1 << 6 ); //2:接收使能 3:发送使能 4:空闲中断 6:发送完成中断
    RS485_SteeringWheel_Handle->CR3 |= ( 1 << 6 ) | ( 1 << 7 ); //6:DMA接收使能 7:DMA发送使能
    RS485_SteeringWheel_Handle->CR1 |= ( 1 << 13 ); 				// 13:串口使能

    HAL_NVIC_SetPriority ( RS485_SteeringWheel_IRQn, IRQ_Priority_UART, 0 );
    HAL_NVIC_EnableIRQ ( RS485_SteeringWheel_IRQn );

    DMA_Init();

    sprintf ( ( char * ) TXBuf, "RS485_SteeringWheel_Send\r\n" );
    RS485_SteeringWheel_Send ( ( uint8_t * ) TXBuf, 16 );

    return 0;
}

/*! \fn				void RS485_SteeringWheel_IRQHandler(void)
 *  \brief 		UART interrupt processing
 *  \param 		none
 *  \return 	none
 */
void RS485_SteeringWheel_IRQHandler ( void )
{
    uint32_t sr;
    uint32_t data;

    NVIC_ClearPendingIRQ ( RS485_SteeringWheel_IRQn );

    sr = RS485_SteeringWheel_Handle->SR;
    data = RS485_SteeringWheel_Handle->DR;
    ( void ) data;

    if ( sr & ( 1 << 4 ) ) //总线空闲(接收完成)
    {
        g_u32RxCplFlag = 1;
        g_u32RxLen = RS485_SteeringWheel_MAX_COMM_LENTH - RS485_SteeringWheel_RX_DMA_STREAM->NDTR;
        RS485_SteeringWheel_RX_DMA_STREAM->CR &= ~0x01;
        RS485_SteeringWheel_RX_DMA_IRQ_CLEAR();
    }

    if ( sr & ( 1 << 6 ) ) //发送完成
    {
        RS485_SteeringWheel_Handle->SR &= ~ ( 1 << 6 );
        u8WheelTxMutex = 0;
//        HAL_GPIO_WritePin ( RS485_SteeringWheel_EN_GPIO_PORT, RS485_SteeringWheel_EN_PIN, GPIO_PIN_RESET );
    }
}

/*! \fn				void RS485_SteeringWheel_RX_DMA_IRQ_Handler(void)
 *  \brief 		The DMA correspond to the UART RX interrupt processing
 *  \param 		none
 *  \return 	none
 */
void RS485_SteeringWheel_RX_DMA_IRQ_Handler ( void )
{
    RS485_SteeringWheel_RX_DMA_IRQ_CLEAR();

    g_u32RxCplFlag = 0;
    g_u32RxLen = 0;

    RS485_SteeringWheel_RX_DMA_STREAM->CR  &= ~0x01;
    RS485_SteeringWheel_RX_DMA_STREAM->NDTR = RS485_SteeringWheel_MAX_COMM_LENTH;
    RS485_SteeringWheel_RX_DMA_STREAM->CR  |= 0x01;
}

/*! \fn				void RS485_SteeringWheel_TX_DMA_IRQ_Handler(void)
 *  \brief 		The DMA correspond to the UART TX interrupt processing
 *  \param 		none
 *  \return 	none
 */
void RS485_SteeringWheel_TX_DMA_IRQ_Handler ( void )
{
    RS485_SteeringWheel_TX_DMA_IRQ_CLEAR();
}

/*! \fn			uint8_t RS485_SteeringWheel_Receive(uint8_t *P,uint32_t *len)
 *  \brief 		UART receive from  g_pu8RxBuf
 *  \param 		Pointer to destination pointer
 *  \param 		Pointer to lenth
 *  \return 	1:One frame of data is received  0;none data
 */
uint8_t RS485_SteeringWheel_Receive ( uint8_t *P, uint16_t *len )
{
    uint8_t ret;
    if ( g_u32RxCplFlag )
    {
        memcpy ( P, g_pu8RxBuf, g_u32RxLen );
        *len = g_u32RxLen;

        g_u32RxCplFlag = 0;
        g_u32RxLen = 0;

        RS485_SteeringWheel_RX_DMA_STREAM->CR  &= ~0x01;
        while ( ( RS485_SteeringWheel_RX_DMA_STREAM->CR & 0x01 ) == 1 );
        RS485_SteeringWheel_RX_DMA_STREAM->NDTR = RS485_SteeringWheel_MAX_COMM_LENTH;
        RS485_SteeringWheel_RX_DMA_STREAM->CR  |= 0x01;

        ret = 1;
//        if ( FaultCtrl.bit.SteerLinkLose )
//        {
//            FaultCtrl.bit.SteerLinkLose = 0;//自动清除
//            m_nSteeringLife = 5000;
//            StateResetSystem();
//        }
    }
    else
    {
        *len = 0;
        ret = 0;
    }

    return ret;
}

/*! \fn			static uint8_t RS485_SteeringWheel_Send(uint8_t *P,uint32_t len,uint32_t tick)
 *  \brief 		UART send data
 *  \param 		Pointer to source buff
 *  \param 		Pointer to lenth
 *  \return 	0:success  other;failed
 */
uint8_t RS485_SteeringWheel_Send ( uint8_t *P, uint32_t len )
{
    uint8_t ret;

    if ( ( len > 0 ) || ( len <= RS485_SteeringWheel_MAX_COMM_LENTH ) )
    {
        if ( u8WheelTxMutex == 0 )
        {
            memcpy ( g_pu8TxBuf, P, len );
            RS485_SteeringWheel_TX_DMA_STREAM->CR &=  ~0x1;
            while ( ( RS485_SteeringWheel_TX_DMA_STREAM->CR & 0x01 ) == 1 );
            RS485_SteeringWheel_TX_DMA_STREAM->NDTR = len;  //数据传输数量

//            HAL_GPIO_WritePin ( RS485_SteeringWheel_EN_GPIO_PORT, RS485_SteeringWheel_EN_PIN, GPIO_PIN_SET );
            RS485_SteeringWheel_TX_DMA_STREAM->CR |=  0x1;  //通道使能

            u8WheelTxMutex = 1;
            ret = 0;
        }
        else
        {
            ret = 1;
            u8WheelTxMutex = 1;
        }
    }
    else
    {
        ret = 2;
    }

    return ret;
}

void Bsp_printf_RS485_SteeringWheel ( char* fmt, ... )
{

    char buf[128];
    uint8_t len;
#if (Output_Mode == Mode_Normal)
    {
        va_list ap;
        va_start ( ap, fmt );
        vsprintf ( ( char* ) buf, fmt, ap );
        va_end ( ap );

        len = strlen ( buf );
        RS485_SteeringWheel_Send ( ( uint8_t* ) buf, len );
    }
#else
    {
        len = 8;
        RS485_SteeringWheel_Send ( ( uint8_t* ) buf, len );
    }
#endif

}
//-------------------- private functions ------------------------------------

static void DMA_Init ( void )
{
    RS485_SteeringWheel_TX_DMA_ClK_ENABLE();
    RS485_SteeringWheel_RX_DMA_ClK_ENABLE();

    /***********************************RX DMA config*******************************/
    RS485_SteeringWheel_RX_DMA_STREAM->CR = 0;
    RS485_SteeringWheel_RX_DMA_STREAM->CR = RS485_SteeringWheel_RX_DMA_CHANNEL            //通道
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
                                            | 0x00 << DMA_SxCR_EN_Pos;					//通道不使能

    RS485_SteeringWheel_RX_DMA_STREAM->M0AR = ( uint32_t ) g_pu8RxBuf;				//存储器地址
    RS485_SteeringWheel_RX_DMA_STREAM->PAR = ( uint32_t ) &RS485_SteeringWheel_Handle->DR;	//外设地址
    RS485_SteeringWheel_RX_DMA_STREAM->NDTR = RS485_SteeringWheel_MAX_COMM_LENTH;				//数据传输数量
    RS485_SteeringWheel_RX_DMA_STREAM->FCR =   0x01 << DMA_SxFCR_FEIE_Pos //FIFO错误中断
            | 0x00 << DMA_SxFCR_DMDIS_Pos; //直接模式


    HAL_NVIC_SetPriority ( RS485_SteeringWheel_RX_DMA_IRQ, IRQ_Priority_UART, 0 );
    HAL_NVIC_EnableIRQ ( RS485_SteeringWheel_RX_DMA_IRQ );

    RS485_SteeringWheel_RX_DMA_STREAM->CR |=  0x01; //使能通道

    /***********************************TX DMA config*******************************/

    RS485_SteeringWheel_TX_DMA_STREAM->CR = 0;
    RS485_SteeringWheel_TX_DMA_STREAM->CR = RS485_SteeringWheel_RX_DMA_CHANNEL            //通道
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
                                            | 0x00 << DMA_SxCR_EN_Pos;					//通道不使能

    RS485_SteeringWheel_TX_DMA_STREAM->M0AR = ( uint32_t ) g_pu8TxBuf;				//存储器地址
    RS485_SteeringWheel_TX_DMA_STREAM->PAR  = ( uint32_t ) &RS485_SteeringWheel_Handle->DR;	//外设地址
    RS485_SteeringWheel_TX_DMA_STREAM->NDTR = 0;				                  //数据传输数量
    RS485_SteeringWheel_TX_DMA_STREAM->FCR  =   0x01 << DMA_SxFCR_FEIE_Pos //FIFO错误中断
            | 0x00 << DMA_SxFCR_DMDIS_Pos; //直接模式

    HAL_NVIC_SetPriority ( RS485_SteeringWheel_TX_DMA_IRQ, IRQ_Priority_UART, 0 );
    HAL_NVIC_EnableIRQ ( RS485_SteeringWheel_TX_DMA_IRQ );

//	RS485_SteeringWheel_TX_DMA_STREAM->CR |=  0x01; //使能通道

}

/*! \fn				static uint16_t RS485_SteeringWheel_GetBrrByBaud(USART_TypeDef* UsartType,uint32_t Baud)
 *  \brief 		Get brr data by baud
 *  \param 		USARTx channel
 *  \param 		baud
 *  \return 	brr data
 */
static uint16_t RS485_SteeringWheel_GetBrrByBaud ( USART_TypeDef* UsartType, uint32_t Baud )
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


