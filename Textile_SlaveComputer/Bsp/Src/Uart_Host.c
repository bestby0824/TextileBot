/**
* ��Ȩ����(C)
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
* <����> | <�汾> | <����> | <����>
*
* xxxx/xx/xx | 1.0.0 | HWW | �����ļ�
*
*/
//------------------------------------------------------------------------------

//-------------------- include files ----------------------------------------
#include "Uart_Host.h"
#include "Xint.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
uint8_t   g_pu8RxBuf[Uart_Host_MAX_COMM_LENTH];
static uint32_t  g_u32RxLen = 0;
static uint32_t  g_u32RxCplFlag = 0;

static uint8_t   g_pu8TxBuf[Uart_Host_MAX_COMM_LENTH];
static uint8_t	 u8HostTxMutex = 0;

//-------------------- private functions declare ----------------------------
static uint16_t Uart_Host_GetBrrByBaud ( USART_TypeDef*, uint32_t Baud );
static void DMA_Init ( void );
//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------

/*! \fn				uint8_t Uart_Host_Init(uint32_t baud)---��λ�����ڳ�ʼ��
 *  \brief 		Initializes the Uart_Host device.
 *  \param 		baud rate
 *  \return 	0:success other:false
 */
uint8_t Uart_Host_Init ( uint32_t baud )
{
    GPIO_InitTypeDef  		GPIO_InitStruct;
    uint8_t TXBuf[32];
    Uart_Host_CLK_ENABLE();

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin 			= Uart_Host_TX_PIN;
    GPIO_InitStruct.Alternate = Uart_Host_GPIO_AF;
    HAL_GPIO_Init ( Uart_Host_TX_GPIO_PORT, &GPIO_InitStruct );

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin 			= Uart_Host_RX_PIN;
    GPIO_InitStruct.Alternate = Uart_Host_GPIO_AF;
    HAL_GPIO_Init ( Uart_Host_RX_GPIO_PORT, &GPIO_InitStruct );

    Uart_Host_Handle->CR1 = 0;
    Uart_Host_Handle->BRR  = Uart_Host_GetBrrByBaud ( Uart_Host_Handle, baud );
    Uart_Host_Handle->CR1 &= 0xFFFF0000;
    Uart_Host_Handle->CR2 &= 0xFFFF8090;
    Uart_Host_Handle->CR3 &= 0xFFFFF000;
    Uart_Host_Handle->CR1 |= ( 1 << 2 ) | ( 1 << 3 ) | ( 1 << 4 ) | ( 1 << 6 ); //2:����ʹ�� 3:����ʹ�� 4:�����ж� 6:��������ж�
    Uart_Host_Handle->CR3 |= ( 1 << 6 ) | ( 1 << 7 ); //6:DMA����ʹ�� 7:DMA����ʹ��
    Uart_Host_Handle->CR1 |= ( 1 << 13 ); 				// 13:����ʹ��
    HAL_NVIC_SetPriority ( Uart_Host_IRQn, IRQ_Priority_UART, 0 );
    HAL_NVIC_EnableIRQ ( Uart_Host_IRQn );

    DMA_Init();

//    sprintf((char *)TXBuf,"Uart_Host_Send");
//    Uart_Host_Send((uint8_t *) TXBuf, 16);

    return 0;
}

/*! \fn				void Uart_Host_IRQHandler(void)
 *  \brief 		UART interrupt processing
 *  \param 		none
 *  \return 	none
 */
void Uart_Host_IRQHandler ( void )
{
    uint32_t sr;
    uint32_t data;

    NVIC_ClearPendingIRQ ( Uart_Host_IRQn );

    sr = Uart_Host_Handle->SR;
    data = Uart_Host_Handle->DR;
    ( void ) data;

    if ( sr & ( 1 << 4 ) ) //���߿���(�������)
    {
        g_u32RxCplFlag = 1;
        g_u32RxLen = Uart_Host_MAX_COMM_LENTH - Uart_Host_RX_DMA_STREAM->NDTR;//���յ���
        Uart_Host_RX_DMA_STREAM->CR &= ~0x01;//�� DMA ������ɺ󣬽��� DMA �������ͷ�ϵͳ��Դ
        Uart_Host_RX_DMA_IRQ_CLEAR();
    }

    if ( sr & ( 1 << 6 ) ) //�������
    {
        Uart_Host_Handle->SR &= ~ ( 1 << 6 );
        u8HostTxMutex = 0;
    }
}

/*! \fn				void Uart_Host_RX_DMA_IRQ_Handler(void)
 *  \brief 		The DMA correspond to the UART RX interrupt processing
 *  \param 		none
 *  \return 	none
 */
void Uart_Host_RX_DMA_IRQ_Handler ( void )
{
    Uart_Host_RX_DMA_IRQ_CLEAR();

    g_u32RxCplFlag = 0;
    g_u32RxLen = 0;

    Uart_Host_RX_DMA_STREAM->CR  &= ~0x01;//���� DMA ������� CR �Ĵ����� EN λ��
    Uart_Host_RX_DMA_STREAM->NDTR = Uart_Host_MAX_COMM_LENTH;
    Uart_Host_RX_DMA_STREAM->CR  |= 0x01;
}

/*! \fn				void Uart_Host_TX_DMA_IRQ_Handler(void)
 *  \brief 		The DMA correspond to the UART TX interrupt processing
 *  \param 		none
 *  \return 	none
 */
void Uart_Host_TX_DMA_IRQ_Handler ( void )
{
    Uart_Host_TX_DMA_IRQ_CLEAR();
}

/*! \fn				uint8_t Uart_Host_Receive(uint8_t *P,uint32_t *len)
 *  \brief 		UART receive from  g_pu8RxBuf
 *  \param 		Pointer to destination pointer
 *  \param 		Pointer to lenth
 *  \return 	1:One frame of data is received  0;none data
 */
uint8_t Uart_Host_Receive ( uint8_t *P, uint16_t *len )
{
    uint8_t ret;

    if ( g_u32RxCplFlag )
    {
        memcpy ( P, g_pu8RxBuf, g_u32RxLen );
        *len = g_u32RxLen;

        g_u32RxCplFlag = 0;
        g_u32RxLen = 0;

        Uart_Host_RX_DMA_STREAM->CR  &= ~0x01;
        while ( ( Uart_Host_RX_DMA_STREAM->CR & 0x01 ) == 1 );//�ȴ� DMA��ֱ���ڴ���ʣ��������
        Uart_Host_RX_DMA_STREAM->NDTR = Uart_Host_MAX_COMM_LENTH;
        Uart_Host_RX_DMA_STREAM->CR  |= 0x01;

        ret = 1;
    }
    else
    {
        *len = 0;
        ret = 0;
    }

    return ret;
}

/*! \fn				static uint8_t Uart_Host_Send(uint8_t *P,uint32_t len,uint32_t tick)
 *  \brief 		UART send data
 *  \param 		Pointer to source buff
 *  \param 		Pointer to lenth
 *  \return 	0:success  other;failed
 */
uint8_t Uart_Host_Send ( uint8_t *P, uint32_t len )
{
    uint8_t ret;

    if ( ( len > 0 ) || ( len <= Uart_Host_MAX_COMM_LENTH ) )
    {
        if ( u8HostTxMutex == 0 )
        {
            memcpy ( g_pu8TxBuf, P, len );

            Uart_Host_TX_DMA_STREAM->CR &=  ~0x1;
            while ( ( Uart_Host_TX_DMA_STREAM->CR & 0x01 ) == 1 );
            Uart_Host_TX_DMA_STREAM->NDTR = len;  //���ݴ�������
            Uart_Host_TX_DMA_STREAM->CR |=  0x1;  //ͨ��ʹ��

            u8HostTxMutex = 1;
            ret = 0;
        }
        else
        {
            ret = 1;
            u8HostTxMutex = 0;
        }
    }
    else
    {
        ret = 2;
    }

    return ret;
}

void Bsp_printf_Host ( char* fmt, ... )
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
        Uart_Host_Send ( ( uint8_t* ) buf, len );
    }
#else
    {
        len = 8;
        Uart_Host_Send ( ( uint8_t* ) buf, len );
    }
#endif

}
//-------------------- private functions ------------------------------------

static void DMA_Init ( void )
{
    Uart_Host_TX_DMA_ClK_ENABLE();
    Uart_Host_RX_DMA_ClK_ENABLE();

    /***********************************RX DMA config*******************************/
    Uart_Host_RX_DMA_STREAM->CR = 0;
    Uart_Host_RX_DMA_STREAM->CR = Uart_Host_RX_DMA_CHANNEL            //ͨ��
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
                                  | 0x00 << DMA_SxCR_DIR_Pos        //���赽�洢��
                                  | 0x00 << DMA_SxCR_PFCTRL_Pos
                                  | 0x00 << DMA_SxCR_TCIE_Pos       //��������ж�
                                  | 0x00 << DMA_SxCR_HTIE_Pos
                                  | 0x01 << DMA_SxCR_TEIE_Pos       //��������ж�
                                  | 0x00 << DMA_SxCR_DMEIE_Pos
                                  | 0x00 << DMA_SxCR_EN_Pos;					//ͨ����ʹ��

    Uart_Host_RX_DMA_STREAM->M0AR = ( uint32_t ) g_pu8RxBuf;				//�洢����ַ
    Uart_Host_RX_DMA_STREAM->PAR = ( uint32_t ) &Uart_Host_Handle->DR;	//�����ַ
    Uart_Host_RX_DMA_STREAM->NDTR = Uart_Host_MAX_COMM_LENTH;				//���ݴ�������
    Uart_Host_RX_DMA_STREAM->FCR =   0x01 << DMA_SxFCR_FEIE_Pos //FIFO�����ж�
                                     | 0x00 << DMA_SxFCR_DMDIS_Pos; //ֱ��ģʽ


    HAL_NVIC_SetPriority ( Uart_Host_RX_DMA_IRQ, IRQ_Priority_UART, 0 );
    HAL_NVIC_EnableIRQ ( Uart_Host_RX_DMA_IRQ );

    Uart_Host_RX_DMA_STREAM->CR |=  0x01; //ʹ��ͨ��

    /***********************************TX DMA config*******************************/

    Uart_Host_TX_DMA_STREAM->CR = 0;
    Uart_Host_TX_DMA_STREAM->CR = Uart_Host_RX_DMA_CHANNEL            //ͨ��
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
                                  | 0x00 << DMA_SxCR_EN_Pos;					//ͨ����ʹ��

    Uart_Host_TX_DMA_STREAM->M0AR = ( uint32_t ) g_pu8TxBuf;				//�洢����ַ
    Uart_Host_TX_DMA_STREAM->PAR  = ( uint32_t ) &Uart_Host_Handle->DR;	//�����ַ
    Uart_Host_TX_DMA_STREAM->NDTR = 0;				                  //���ݴ�������
    Uart_Host_TX_DMA_STREAM->FCR  =   0x01 << DMA_SxFCR_FEIE_Pos //FIFO�����ж�
                                      | 0x00 << DMA_SxFCR_DMDIS_Pos; //ֱ��ģʽ

    HAL_NVIC_SetPriority ( Uart_Host_TX_DMA_IRQ, IRQ_Priority_UART, 0 );
    HAL_NVIC_EnableIRQ ( Uart_Host_TX_DMA_IRQ );

//	Uart_Host_TX_DMA_STREAM->CR |=  0x01; //ʹ��ͨ��

}

/*! \fn				static uint16_t Uart_Host_GetBrrByBaud(USART_TypeDef* UsartType,uint32_t Baud)
 *  \brief 		Get brr data by baud
 *  \param 		USARTx channel
 *  \param 		baud
 *  \return 	brr data
 */
static uint16_t Uart_Host_GetBrrByBaud ( USART_TypeDef* UsartType, uint32_t Baud )
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


