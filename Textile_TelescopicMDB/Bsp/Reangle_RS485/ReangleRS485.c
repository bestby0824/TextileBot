
//------------------------------------------------------------------------------

//-------------------- include files ----------------------------------------
#include "ReangleRS485.h"
#include "TimerBase.h"
#include "Int_Ctrl.h"
#include <string.h>
#include "Int_Ctrl.h"
#include "Monitor.h"
//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
static uint8_t   g_pu8RxBuf[ReangleRS485_MAX_COMM_LENTH];
static uint32_t  g_u32RxLen = 0;
static uint32_t  g_u32RxCplFlag = 0;
static uint32_t  g_u32RxTick = 0;

static uint8_t   g_pu8TxBuf[ReangleRS485_MAX_COMM_LENTH];
static uint8_t   u8ReangleTxMutex = 0;
static uint8_t   g_u8TxFlag = 0;
static uint32_t  g_u32TxTick = 0;
_iq Reangle_Value = 0;
//-------------------- private functions declare ----------------------------
static uint16_t Uart_GetBrrByBaud ( USART_TypeDef*, uint32_t Baud );
static void DMA_Init ( void );
static uint8_t Get_XOR_CRC ( uint8_t *P, uint16_t len );
//-------------------- public data ------------------------------------------
extern uint8_t g_u8ModbusAddr;
//-------------------- public functions -------------------------------------

/*! \fn                uint8_t ReangleRS485_Init(uint32_t baud)
 *  \brief         Initializes the ReangleRS485 device.
 *  \param         baud rate
 *  \return     0:success other:false
 */
uint8_t ReangleRS485_Init ( uint32_t baud )
{
    GPIO_InitTypeDef          GPIO_InitStruct;

    ReangleRS485_CLK_ENABLE();

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pin             = ReangleRS485_TX_PIN;
    GPIO_InitStruct.Alternate = ReangleRS485_GPIO_AF;
    HAL_GPIO_Init ( ReangleRS485_TX_GPIO_PORT, &GPIO_InitStruct );

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pin             = ReangleRS485_RX_PIN;
    GPIO_InitStruct.Alternate = ReangleRS485_GPIO_AF;
    HAL_GPIO_Init ( ReangleRS485_RX_GPIO_PORT, &GPIO_InitStruct );

    GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pin       = ReangleRS485_EN_PIN;
    HAL_GPIO_Init ( ReangleRS485_EN_GPIO_PORT, &GPIO_InitStruct );

    HAL_GPIO_WritePin ( ReangleRS485_EN_GPIO_PORT, ReangleRS485_EN_PIN, GPIO_PIN_RESET );

    ReangleRS485_Handle->CR1 = 0;
    ReangleRS485_Handle->BRR  = Uart_GetBrrByBaud ( ReangleRS485_Handle, baud );
    ReangleRS485_Handle->CR1 &= 0xFFFF0000;
    ReangleRS485_Handle->CR2 &= 0xFFFF8090;
    ReangleRS485_Handle->CR3 &= 0xFFFFF000;
    ReangleRS485_Handle->CR1 |= ( 1 << 2 ) | ( 1 << 3 ) | ( 1 << 4 ) | ( 1 << 6 ); //2:����ʹ�� 3:����ʹ�� 4:�����ж� 6:��������ж�
    ReangleRS485_Handle->CR3 |= ( 1 << 6 ) | ( 1 << 7 ); //6:DMA����ʹ�� 7:DMA����ʹ��
    ReangleRS485_Handle->CR1 |= ( 1 << 13 );               // 13:����ʹ��

    HAL_NVIC_SetPriority ( ReangleRS485_IRQn, IRQ_Priority_Reangle_CODER, 0 );
    HAL_NVIC_EnableIRQ ( ReangleRS485_IRQn );

    DMA_Init();

    return 0;
}

/*! \fn         uint8_t Get_XOR_CRC(uint8_t *P,uint16_t len)
 *  \brief      ���ֽ����У�������
 *  \param      bufָ�� ����
 *  \return     У����
 */
static uint8_t Get_XOR_CRC ( uint8_t *P, uint16_t len )
{
    uint8_t CRCret;
    uint16_t i;
    CRCret = P[0];
    for ( i = 1; i < len; i++ )
    {
        CRCret ^= P[i];
    }
    return CRCret;
}
_iq GetReangleValue ( void )
{
    return (Reangle_Value>>1);  //ת��Ϊ16λ����
}

/*! \fn                void ReangleRS485_IRQHandler(void)
 *  \brief         UART interrupt processing
 *  \param         none
 *  \return     none
 */
uint32_t u32RXCnt1 = 0,u32RXCnt2 = 0;

uint8_t XORTest1, XORTest2;
void ReangleRS485_IRQHandler ( void )
{
    uint32_t sr;
    uint32_t data;

    NVIC_ClearPendingIRQ ( ReangleRS485_IRQn );

    sr = ReangleRS485_Handle->SR;
    data = ReangleRS485_Handle->DR;
    ( void ) data;

    if ( sr & ( 1 << 4 ) ) //���߿���(�������)
    {
        uint32_t u32Temp;
        u32RXCnt1++;
        g_u32RxLen = ReangleRS485_MAX_COMM_LENTH - ReangleRS485_RX_DMA_STREAM->NDTR;

        if ( ( g_pu8RxBuf[0] == 0x02 ) )
        {
            u32RXCnt2++;
            XORTest1 = g_pu8RxBuf[5];
            XORTest2 = Get_XOR_CRC ( g_pu8RxBuf, 5 );
            if ( XORTest1 == XORTest2 )
            {
                //����17bit
                Reangle_Value = ( ( uint32_t ) g_pu8RxBuf[4] << 16 ) + ( ( uint32_t ) g_pu8RxBuf[3] << 8 ) + g_pu8RxBuf[2];
                s16ReangleLife = FullLife_100ms;
            }
        }
        ReangleRS485_RX_DMA_STREAM->CR  &= ~0x01;
        while ( ( ReangleRS485_RX_DMA_STREAM->CR & 0x01 ) != 0 )
        {
            ReangleRS485_RX_DMA_STREAM->CR &= ~0x01;
            ReangleRS485_RX_DMA_IRQ_CLEAR();
        }
        ReangleRS485_RX_DMA_STREAM->NDTR = ReangleRS485_MAX_COMM_LENTH;
        ReangleRS485_RX_DMA_STREAM->CR  |= 0x01;
    }

    if ( sr & ( 1 << 6 ) ) //�������
    {
        ReangleRS485_Handle->SR &= ~ ( 1 << 6 );
        u8ReangleTxMutex = 0;
        HAL_GPIO_WritePin ( ReangleRS485_EN_GPIO_PORT, ReangleRS485_EN_PIN, GPIO_PIN_RESET );
    }

}

/*! \fn                void ReangleRS485_RX_DMA_IRQ_Handler(void)
 *  \brief         The DMA correspond to the UART RX interrupt processing
 *  \param         none
 *  \return     none
 */
void ReangleRS485_RX_DMA_IRQ_Handler ( void )
{
    ReangleRS485_RX_DMA_IRQ_CLEAR();

    g_u32RxCplFlag = 0;
    g_u32RxLen = 0;

    ReangleRS485_RX_DMA_STREAM->CR  &= ~0x01;
    ReangleRS485_RX_DMA_STREAM->NDTR = ReangleRS485_MAX_COMM_LENTH;
    ReangleRS485_RX_DMA_STREAM->CR  |= 0x01;
}

/*! \fn                void ReangleRS485_TX_DMA_IRQ_Handler(void)
 *  \brief         The DMA correspond to the UART TX interrupt processing
 *  \param         none
 *  \return     none
 */
void ReangleRS485_TX_DMA_IRQ_Handler ( void )
{
    ReangleRS485_TX_DMA_IRQ_CLEAR();
}



/*! \fn                static uint8_t ReangleRS485_Send(uint8_t *P,uint32_t len,uint32_t basetick)
 *  \brief         UART send data
 *  \param         Pointer to source buff
 *  \param         Pointer to lenth
 *  \return     0:success  other;failed
 */
uint8_t ReangleRS485_Send ( uint8_t *P, uint32_t len )
{
    uint8_t ret;

    if ( ( len > 0 ) || ( len <= ReangleRS485_MAX_COMM_LENTH ) )
    {
        if ( u8ReangleTxMutex == 0 )
        {
            HAL_GPIO_WritePin ( ReangleRS485_EN_GPIO_PORT, ReangleRS485_EN_PIN, GPIO_PIN_SET );
            memcpy ( g_pu8TxBuf, P, len );

            ReangleRS485_TX_DMA_STREAM->CR &=  ~0x1;
            while ( ( ReangleRS485_TX_DMA_STREAM->CR & 0x01 ) != 0 );
            ReangleRS485_TX_DMA_STREAM->NDTR = len;                        //���ݴ�������

            ReangleRS485_TX_DMA_STREAM->CR |=  0x1;                            //ͨ��ʹ��
            g_u8TxFlag = 0;
            u8ReangleTxMutex = 1;
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
void ReangleRS485_Send_Process ( void )
{
    g_pu8TxBuf[0] = 2;
    ReangleRS485_Send ( g_pu8TxBuf, 1 );
    u8ReangleTxMutex = 0;
}

//-------------------- private functions ------------------------------------
static void DMA_Init ( void )
{
    ReangleRS485_TX_DMA_ClK_ENABLE();
    ReangleRS485_RX_DMA_ClK_ENABLE();

    /***********************************RX DMA config*******************************/
    ReangleRS485_RX_DMA_STREAM->CR = 0;
    ReangleRS485_RX_DMA_IRQ_CLEAR();
    while ( ( ReangleRS485_RX_DMA_STREAM->CR & 0x01 ) != 0 );

    ReangleRS485_RX_DMA_STREAM->CR = ReangleRS485_RX_DMA_CHANNEL            //ͨ��
                                     | 0x00 << DMA_SxCR_MBURST_Pos
                                     | 0x00 << DMA_SxCR_PBURST_Pos
                                     | 0x00 << DMA_SxCR_CT_Pos
                                     | 0x00 << DMA_SxCR_DBM_Pos
                                     | 0x01 << DMA_SxCR_PL_Pos         //ͨ�����ȼ� ��
                                     | 0x00 << DMA_SxCR_PINCOS_Pos
                                     | 0x00 << DMA_SxCR_MSIZE_Pos      //�洢�����ݿ�� 8λ
                                     | 0x00 << DMA_SxCR_PSIZE_Pos      //�������ݿ�� 8λ
                                     | 0x01 << DMA_SxCR_MINC_Pos       //�洢����ַ����
                                     | 0x00 << DMA_SxCR_PINC_Pos       //�����ַ������
                                     | 0x01 << DMA_SxCR_CIRC_Pos       //��ѭ��ģʽ
                                     | 0x00 << DMA_SxCR_DIR_Pos        //���赽�洢��
                                     | 0x00 << DMA_SxCR_PFCTRL_Pos
                                     | 0x00 << DMA_SxCR_TCIE_Pos       //��������ж�
                                     | 0x00 << DMA_SxCR_HTIE_Pos
                                     | 0x01 << DMA_SxCR_TEIE_Pos       //��������ж�
                                     | 0x00 << DMA_SxCR_DMEIE_Pos
                                     | 0x00 << DMA_SxCR_EN_Pos;                   //ͨ����ʹ��

    ReangleRS485_RX_DMA_STREAM->M0AR = ( uint32_t ) g_pu8RxBuf;             //�洢����ַ
    ReangleRS485_RX_DMA_STREAM->PAR = ( uint32_t ) &ReangleRS485_Handle->DR; //�����ַ
    ReangleRS485_RX_DMA_STREAM->NDTR = ReangleRS485_MAX_COMM_LENTH;                //���ݴ�������
    ReangleRS485_RX_DMA_STREAM->FCR =   0x01 << DMA_SxFCR_FEIE_Pos //FIFO�����ж�
                                        | 0x00 << DMA_SxFCR_DMDIS_Pos; //ֱ��ģʽ

    ReangleRS485_RX_DMA_STREAM->CR |=  0x01; //ʹ��ͨ��

    HAL_NVIC_SetPriority ( ReangleRS485_RX_DMA_IRQ, IRQ_Priority_485, 0 );
    HAL_NVIC_EnableIRQ ( ReangleRS485_RX_DMA_IRQ );

    /***********************************TX DMA config*******************************/

    ReangleRS485_TX_DMA_STREAM->CR = 0;
    ReangleRS485_TX_DMA_IRQ_CLEAR();
    while ( ( ReangleRS485_TX_DMA_STREAM->CR & 0x01 ) != 0 );

    ReangleRS485_TX_DMA_STREAM->CR = ReangleRS485_RX_DMA_CHANNEL            //ͨ��
                                     | 0x00 << DMA_SxCR_MBURST_Pos
                                     | 0x00 << DMA_SxCR_PBURST_Pos
                                     | 0x00 << DMA_SxCR_CT_Pos
                                     | 0x00 << DMA_SxCR_DBM_Pos
                                     | 0x01 << DMA_SxCR_PL_Pos         //ͨ�����ȼ� ��
                                     | 0x00 << DMA_SxCR_PINCOS_Pos
                                     | 0x00 << DMA_SxCR_MSIZE_Pos      //�洢�����ݿ�� 8λ
                                     | 0x00 << DMA_SxCR_PSIZE_Pos      //�������ݿ�� 8λ
                                     | 0x01 << DMA_SxCR_MINC_Pos       //�洢����ַ����
                                     | 0x00 << DMA_SxCR_PINC_Pos       //�����ַ������
                                     | 0x00 << DMA_SxCR_CIRC_Pos       //��ѭ��ģʽ
                                     | 0x01 << DMA_SxCR_DIR_Pos        //�洢��������
                                     | 0x00 << DMA_SxCR_PFCTRL_Pos
                                     | 0x01 << DMA_SxCR_TCIE_Pos       //��������ж�
                                     | 0x00 << DMA_SxCR_HTIE_Pos
                                     | 0x01 << DMA_SxCR_TEIE_Pos       //��������ж�
                                     | 0x00 << DMA_SxCR_DMEIE_Pos
                                     | 0x00 << DMA_SxCR_EN_Pos;                   //ͨ����ʹ��

    ReangleRS485_TX_DMA_STREAM->M0AR = ( uint32_t ) g_pu8TxBuf;             //�洢����ַ
    ReangleRS485_TX_DMA_STREAM->PAR  = ( uint32_t ) &ReangleRS485_Handle->DR; //�����ַ
    ReangleRS485_TX_DMA_STREAM->NDTR = 0;                                  //���ݴ�������
    ReangleRS485_TX_DMA_STREAM->FCR  =   0x01 << DMA_SxFCR_FEIE_Pos //FIFO�����ж�
                                         | 0x00 << DMA_SxFCR_DMDIS_Pos; //ֱ��ģʽ

    HAL_NVIC_SetPriority ( ReangleRS485_TX_DMA_IRQ, IRQ_Priority_485, 0 );
    HAL_NVIC_EnableIRQ ( ReangleRS485_TX_DMA_IRQ );

//    ReangleRS485_TX_DMA_STREAM->CR |=  0x01; //ʹ��ͨ��

}

/*! \fn                static uint16_t Uart_GetBrrByBaud(USART_TypeDef* UsartType,uint32_t Baud)
 *  \brief         Get brr data by baud
 *  \param         USARTx channel
 *  \param         baud
 *  \return     brr data
 */
static uint16_t Uart_GetBrrByBaud ( USART_TypeDef* UsartType, uint32_t Baud )
{
    uint16_t u16Brr;

    if ( ( UsartType == USART1 ) || ( UsartType == USART6 ) ) //APB2�����ϵĴ���
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
