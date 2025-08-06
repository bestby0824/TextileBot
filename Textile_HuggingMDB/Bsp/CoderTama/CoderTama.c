
//------------------------------------------------------------------------------

//-------------------- include files ----------------------------------------
#include "CoderTama.h"
#include <string.h>
#include "main.h"
#include "Int_Ctrl.h"
#include "MODBusCRC.h"
#include "Monitor.h"

//-------------------- local definitions ------------------------------------
#define TAMA_REPLY_LENTH   7
//-------------------- private data -----------------------------------------
static _iq  _iqAngleM = 0;
WheelAngle WheelAngleHandle1 = WheelAngleDefault;

static uint8_t   g_pu8RxBuf[TAMA_MAX_COMM_LENTH];
static uint32_t  g_u32RxLen = 0;

static uint8_t   g_pu8TxBuf[TAMA_MAX_COMM_LENTH];
static uint8_t   g_u8TxMutex = 0;

static uint16_t  g_u16TamaCheckFlags = 0xffff;

//-------------------- private functions declare ----------------------------
static uint16_t Uart_GetBrrByBaud ( USART_TypeDef*, uint32_t Baud );
static void DMA_Init ( void );
static uint8_t TAMA_Send ( uint8_t *P, uint32_t len );
static uint16_t EcoderTxCnt = 0;
//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
/*! \fn				void CoderTama_Init(uint32_t baud)
 *  \brief 		Initializes the TAMA device.
 *  \param 		baud rate
 *  \return 	0:success other:false
 */
void CoderTama_Init ( uint32_t baud )
{
    GPIO_InitTypeDef  		GPIO_InitStruct;

    TAMA_CLK_ENABLE();

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pin       = TAMA_TX_PIN;
    GPIO_InitStruct.Alternate = TAMA_GPIO_AF;
    HAL_GPIO_Init ( TAMA_TX_GPIO_PORT, &GPIO_InitStruct );

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pin       = TAMA_RX_PIN;
    GPIO_InitStruct.Alternate = TAMA_GPIO_AF;
    HAL_GPIO_Init ( TAMA_RX_GPIO_PORT, &GPIO_InitStruct );

    GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pin       = TAMA_EN_PIN;
    HAL_GPIO_Init ( TAMA_EN_GPIO_PORT, &GPIO_InitStruct );

    HAL_GPIO_WritePin ( TAMA_EN_GPIO_PORT, TAMA_EN_PIN, GPIO_PIN_RESET );

    TAMA_Set->CR1 = 0;
    TAMA_Set->BRR  = Uart_GetBrrByBaud ( TAMA_Set, baud );
    TAMA_Set->CR1 &= 0xFFFF0000;
    TAMA_Set->CR2 &= 0xFFFF8090;
    TAMA_Set->CR3 &= 0xFFFFF000;
    TAMA_Set->CR1 |= ( 1 << 2 ) | ( 1 << 3 ) | ( 1 << 4 ) | ( 1 << 6 ) | ( 0 << 12 ); //2:接收使能 3:发送使能 4:空闲中断 6:发送完成中断  12:起始位
    TAMA_Set->CR3 |= ( 1 << 6 ) | ( 1 << 7 ); //6:DMA接收使能 7:DMA发送使能
    TAMA_Set->CR1 |= ( 1 << 13 ); 				// 13:串口使能

    HAL_NVIC_SetPriority ( TAMA_IRQn, IRQ_Priority_TAMA_CODER, 0 );
    HAL_NVIC_EnableIRQ ( TAMA_IRQn );

    DMA_Init();

}

/*! \fn				void CoderTama_SendQuery(void)
 *  \brief 		Send query to read code
 *  \param 		none
 *  \return 	none
 */
void CoderTama_SendQuery ( void )
{
    uint16_t EcoderID, u16Crc;
    uint8_t u8Data[64];

    EcoderTxCnt++;
    EcoderID = ( EcoderTxCnt % 2 );
    u8Data[0] = EcoderID + 1;
    u8Data[1] = 0x03;
    u8Data[2] = 0x00;
    u8Data[3] = 0x00;
    u8Data[4] = 0x00;
    u8Data[5] = 0x01;
    u16Crc = Modbus_GetCRC16 ( u8Data, 6 );
//    u8Data[6] = 0x84;
//    u8Data[7] = 0x0A;//01 03 00 00 00 01 (84 0A)
    u8Data[6] = ( u16Crc & 0x00FF );
    u8Data[7] = ( ( u16Crc >> 8 ) & 0x00FF );
    TAMA_Send ( u8Data, 8 );
    g_u16TamaCheckFlags = g_u16TamaCheckFlags << 1;
}
/*! \fn				_iq CoderTama_GetCode(void)
 *  \brief 		get code
 *  \param 		none
 *  \return 	code
 */
_iq CoderTama_GetCode ( void )
{
    return ( _iqAngleM );
//    return ( WheelAngleHandle1._iqLeft );
}

uint16_t CoderTama_GetCheckFlags ( void )
{
    return g_u16TamaCheckFlags;
}

/*! \fn				void TAMA_IRQHandler(void)
 *  \brief 		UART interrupt processing
 *  \param 		none
 *  \return 	none
 */

_iq _iqAngleTem = 0;
void TAMA_IRQHandler ( void ) //588nS
{
    uint32_t sr;
    uint32_t data;
    uint16_t u16Crc;

    NVIC_ClearPendingIRQ ( TAMA_IRQn );

    sr = TAMA_Set->SR;
    data = TAMA_Set->DR;
    ( void ) data;

    if ( sr & ( 1 << 4 ) ) //总线空闲(接收完成)
    {
        g_u32RxLen = TAMA_MAX_COMM_LENTH - TAMA_RX_DMA_STREAM->NDTR;
        TAMA_RX_DMA_STREAM->CR &= ~0x01;
        TAMA_RX_DMA_IRQ_CLEAR();
//        g_pu8RxBuf[0] = 0x01;
//        g_pu8RxBuf[1] = 0x03;
//        g_pu8RxBuf[2] = 0x02;
//        g_pu8RxBuf[3] = 0x01;
//        g_pu8RxBuf[4] = 0x42;
//        g_pu8RxBuf[5] = 0x39;
//        g_pu8RxBuf[6] = 0xE5;
        if ( g_u32RxLen == TAMA_REPLY_LENTH )
        {
            u16Crc = g_pu8RxBuf[6];
            u16Crc = ( u16Crc << 8 ) + g_pu8RxBuf[5];
            if ( u16Crc == Modbus_GetCRC16 ( g_pu8RxBuf, TAMA_REPLY_LENTH - 2 ) ) //CRC校验待添加
            {
                if ( g_pu8RxBuf[0] == 1 )
                {
                    s16CoderTama1Life = FullLife_2s;
                    WheelAngleHandle1._iqLeft = ( ( uint32_t ) g_pu8RxBuf[4] | ( ( uint32_t ) g_pu8RxBuf[3] << 8 ) ) << 1; //原值0~32767,相当于IQ15
                    WheelAngleHandle1._iqLSin = _IQsinPU ( WheelAngleHandle1._iqLeft );
                    g_u16TamaCheckFlags |= 0x0001;

                } else if ( g_pu8RxBuf[0] == 2 )
                {
                    s16CoderTama2Life = FullLife_2s;
                    WheelAngleHandle1._iqRight = ( ( uint32_t ) g_pu8RxBuf[4] | ( ( uint32_t ) g_pu8RxBuf[3] << 8 ) ) << 1;
                    WheelAngleHandle1._iqRSin = _IQsinPU ( WheelAngleHandle1._iqRight );
                    g_u16TamaCheckFlags |= 0x0001;
                }
                if ( ( g_pu8RxBuf[0] == 1 ) || ( g_pu8RxBuf[0] == 2 ) )
                {
                    _iq _iqTem;
                    _iqTem = WheelAngleHandle1._iqRight + WheelAngleHandle1._iqLeft;
                    Angle_IQ0_IQ1 ( &_iqTem );
                    WheelAngleHandle1._iqR_LSin = _IQsinPU ( _iqTem );

                    Angle_IQ0_IQ1_Avg ( &WheelAngleHandle1._iqLeft, &WheelAngleHandle1._iqRight, &WheelAngleHandle1._iqMean_RL );
                    _iqAngleTem = WheelAngleHandle1._iqMean_RL;
                    Angle_IQ0_5 ( &_iqAngleTem );
                    Angle_IQ0_IQ1 ( &WheelAngleHandle1._iqMean_RL );
//                    if ( WheelAngleHandle1._iqR_LSin > _IQ ( 0.01 ) ) {

                    WheelAngleHandle1._iqM_Tan = 2 * _IQdiv ( _IQmpy ( WheelAngleHandle1._iqLSin, WheelAngleHandle1._iqRSin ), WheelAngleHandle1._iqR_LSin );
                    _iqAngleM = _IQdiv ( _IQatan ( WheelAngleHandle1._iqM_Tan ), _IQ ( 6.2832 ) ); //弧度制->pu
                    if ( _IQabs ( _iqAngleTem ) < _IQ ( 0.003 ) ) //( _IQabs ( _iqAngleM - WheelAngleHandle1._iqMean_RL ) > _IQ ( 0.014 ) )
                    {
                        //阿克曼模型与均值计算结果相差大于5度 //切换会造成抖动
                        _iqAngleM = WheelAngleHandle1._iqMean_RL;
                        
//                        SetCoderAKM_Warn();//同时报警
                    } else {
//                        ReSetCoderAKM_Warn();//消除报警
                    }
//                    } else {
//                        Angle_IQ0_IQ1_Avg ( &WheelAngleHandle1._iqLeft, &WheelAngleHandle1._iqRight, &_iqAngleM );

//                        ReSetCoderAKM_Warn();//消除报警
//                    }
                }
                _iqAngleM = _iqAngleM + _IQ ( 0.5 );
                Angle_IQ0_IQ1 ( &_iqAngleM );
            }
        }
        
//        if ( g_pu8RxBuf[1] == 0x06 )
//        {
//            u16Crc = g_pu8RxBuf[7];
//            u16Crc = ( u16Crc << 8 ) + g_pu8RxBuf[6];
//            if ( u16Crc == Modbus_GetCRC16 ( g_pu8RxBuf, g_u32RxLen - 2 ) )
//            {
//                memcpy ( u8EcoderCalRxData, g_pu8RxBuf, g_u32RxLen );
//            }
//        }

        while ( ( TAMA_RX_DMA_STREAM->CR & 0x01 ) != 0 );
        TAMA_RX_DMA_STREAM->NDTR = TAMA_MAX_COMM_LENTH;
        TAMA_RX_DMA_STREAM->CR  |= 0x01;
    }

    if ( sr & ( 1 << 6 ) ) //发送完成
    {
        TAMA_Set->SR &= ~ ( 1 << 6 );
        HAL_GPIO_WritePin ( TAMA_EN_GPIO_PORT, TAMA_EN_PIN, GPIO_PIN_RESET );
        g_u8TxMutex = 0;
    }

}


/*! \fn				void TAMA_RX_DMA_IRQ_Handler(void)
 *  \brief 		The DMA correspond to the UART RX interrupt processing
 *  \param 		none
 *  \return 	none
 */
void TAMA_RX_DMA_IRQ_Handler ( void )
{
    TAMA_RX_DMA_IRQ_CLEAR();

    g_u32RxLen = 0;

    TAMA_RX_DMA_STREAM->CR  &= ~0x01;
    TAMA_RX_DMA_STREAM->NDTR = TAMA_MAX_COMM_LENTH;
    TAMA_RX_DMA_STREAM->CR  |= 0x01;
}


/*! \fn				void TAMA_TX_DMA_IRQ_Handler(void)
 *  \brief 		The DMA correspond to the UART TX interrupt processing
 *  \param 		none
 *  \return 	none
 */
void TAMA_TX_DMA_IRQ_Handler ( void )
{
    TAMA_TX_DMA_IRQ_CLEAR();
}


//-------------------- private functions ------------------------------------

/*! \fn				static uint8_t TAMA_Send(uint8_t *P,uint32_t len)
 *  \brief 		UART send data
 *  \param 		Pointer to source buff
 *  \param 		Pointer to lenth
 *  \return 	0:success  other;failed
 */
static uint8_t TAMA_Send ( uint8_t *P, uint32_t len )
{
    uint8_t ret;

    if ( ( len > 0 ) || ( len <= TAMA_MAX_COMM_LENTH ) )
    {
        if ( g_u8TxMutex == 0 )
        {
            HAL_GPIO_WritePin ( TAMA_EN_GPIO_PORT, TAMA_EN_PIN, GPIO_PIN_SET );
            memcpy ( g_pu8TxBuf, P, len );
            TAMA_TX_DMA_STREAM->CR &=  ~0x1;
            while ( ( TAMA_TX_DMA_STREAM->CR & 0x01 ) == 1 );
            TAMA_TX_DMA_STREAM->NDTR = len;                     //数据传输数量
            TAMA_TX_DMA_STREAM->CR |=  0x1;                     //通道使能
            g_u8TxMutex = 1;
            ret = 0;
        }
        else
        {
            ret = 1;
        }
    }
    else
    {
        ret = 2;
    }

    return ret;
}

static void DMA_Init ( void )
{
    TAMA_TX_DMA_ClK_ENABLE();
    TAMA_RX_DMA_ClK_ENABLE();

    /***********************************RX DMA config*******************************/
    TAMA_RX_DMA_STREAM->CR = 0;
    TAMA_RX_DMA_STREAM->CR = TAMA_RX_DMA_CHANNEL                //通道
                             | 0x00 << DMA_SxCR_MBURST_Pos
                             | 0x00 << DMA_SxCR_PBURST_Pos
                             | 0x00 << DMA_SxCR_CT_Pos
                             | 0x00 << DMA_SxCR_DBM_Pos
                             | 0x01 << DMA_SxCR_PL_Pos          //通道优先级 中
                             | 0x00 << DMA_SxCR_PINCOS_Pos
                             | 0x00 << DMA_SxCR_MSIZE_Pos       //存储器数据宽度 8位
                             | 0x00 << DMA_SxCR_PSIZE_Pos       //外设数据宽度 8位
                             | 0x01 << DMA_SxCR_MINC_Pos        //存储器地址递增
                             | 0x00 << DMA_SxCR_PINC_Pos        //外设地址不递增
                             | 0x00 << DMA_SxCR_CIRC_Pos        //非循环模式
                             | 0x00 << DMA_SxCR_DIR_Pos         //外设到存储器
                             | 0x00 << DMA_SxCR_PFCTRL_Pos
                             | 0x00 << DMA_SxCR_TCIE_Pos        //传输完成中断
                             | 0x00 << DMA_SxCR_HTIE_Pos
                             | 0x01 << DMA_SxCR_TEIE_Pos        //传输错误中断
                             | 0x00 << DMA_SxCR_DMEIE_Pos
                             | 0x00 << DMA_SxCR_EN_Pos;         //通道不使能

    TAMA_RX_DMA_STREAM->M0AR = ( uint32_t ) g_pu8RxBuf;         //存储器地址
    TAMA_RX_DMA_STREAM->PAR = ( uint32_t ) &TAMA_Set->DR;    //外设地址
    TAMA_RX_DMA_STREAM->NDTR = TAMA_MAX_COMM_LENTH;             //数据传输数量
    TAMA_RX_DMA_STREAM->FCR =   0x01 << DMA_SxFCR_FEIE_Pos      //FIFO错误中断
                                | 0x00 << DMA_SxFCR_DMDIS_Pos;  //直接模式


    HAL_NVIC_SetPriority ( TAMA_RX_DMA_IRQ, IRQ_Priority_TAMA_CODER, 0 );
    HAL_NVIC_EnableIRQ ( TAMA_RX_DMA_IRQ );

    TAMA_RX_DMA_STREAM->CR |=  0x01; //使能通道

    /***********************************TX DMA config*******************************/

    TAMA_TX_DMA_STREAM->CR = 0;
    TAMA_TX_DMA_STREAM->CR = TAMA_RX_DMA_CHANNEL                //通道
                             | 0x00 << DMA_SxCR_MBURST_Pos
                             | 0x00 << DMA_SxCR_PBURST_Pos
                             | 0x00 << DMA_SxCR_CT_Pos
                             | 0x00 << DMA_SxCR_DBM_Pos
                             | 0x01 << DMA_SxCR_PL_Pos          //通道优先级 中
                             | 0x00 << DMA_SxCR_PINCOS_Pos
                             | 0x00 << DMA_SxCR_MSIZE_Pos       //存储器数据宽度 8位
                             | 0x00 << DMA_SxCR_PSIZE_Pos       //外设数据宽度 8位
                             | 0x01 << DMA_SxCR_MINC_Pos        //存储器地址递增
                             | 0x00 << DMA_SxCR_PINC_Pos        //外设地址不递增
                             | 0x00 << DMA_SxCR_CIRC_Pos        //非循环模式
                             | 0x01 << DMA_SxCR_DIR_Pos         //存储器到外设
                             | 0x00 << DMA_SxCR_PFCTRL_Pos
                             | 0x01 << DMA_SxCR_TCIE_Pos        //传输完成中断
                             | 0x00 << DMA_SxCR_HTIE_Pos
                             | 0x01 << DMA_SxCR_TEIE_Pos        //传输错误中断
                             | 0x00 << DMA_SxCR_DMEIE_Pos
                             | 0x00 << DMA_SxCR_EN_Pos;         //通道不使能

    TAMA_TX_DMA_STREAM->M0AR = ( uint32_t ) g_pu8TxBuf;         //存储器地址
    TAMA_TX_DMA_STREAM->PAR  = ( uint32_t ) &TAMA_Set->DR;   //外设地址
    TAMA_TX_DMA_STREAM->NDTR = 0;                               //数据传输数量
    TAMA_TX_DMA_STREAM->FCR  =   0x01 << DMA_SxFCR_FEIE_Pos     //FIFO错误中断
                                 | 0x00 << DMA_SxFCR_DMDIS_Pos; //直接模式

    HAL_NVIC_SetPriority ( TAMA_TX_DMA_IRQ, IRQ_Priority_TAMA_CODER, 0 );
    HAL_NVIC_EnableIRQ ( TAMA_TX_DMA_IRQ );

//	TAMA_TX_DMA_STREAM->CR |=  0x01; //使能通道

}

/*! \fn				static uint16_t Uart_GetBrrByBaud(USART_TypeDef* UsartType,uint32_t Baud)
 *  \brief 		Get brr data by baud
 *  \param 		USARTx channel
 *  \param 		baud
 *  \return 	brr data
 */
static uint16_t Uart_GetBrrByBaud ( USART_TypeDef* UsartType, uint32_t Baud )
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
