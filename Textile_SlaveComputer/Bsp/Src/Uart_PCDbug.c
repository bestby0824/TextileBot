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
#include "Uart_PCDbug.h"
#include "Xint.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
static uint8_t   pu8_PCDbug_RxBuf[Uart_PCDbug_MAX_COMM_LENTH];
static uint32_t  u32_PCDbug_RxLen = 0;
static uint32_t  u32_PCDbug_RxCplFlag = 0;

static uint8_t   pu8_PCDbug_TxBuf[Uart_PCDbug_MAX_COMM_LENTH];
static uint8_t   u8_PCDbug_TxMutex = 0;

//-------------------- private functions declare ----------------------------
static uint16_t Uart_PCDbug_GetBrrByBaud ( USART_TypeDef*, uint32_t Baud );
static void DMA_Init ( void );
//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------

/*! \fn				uint8_t Uart_PCDbug_Init(uint32_t baud)
 *  \brief 		Initializes the Uart_PCDbug device.
 *  \param 		baud rate
 *  \return 	0:success other:false
 */
uint8_t Uart_PCDbug_Init ( uint32_t baud )
{
    GPIO_InitTypeDef  		GPIO_InitStruct;
    uint8_t TXBuf[32];
    Uart_PCDbug_CLK_ENABLE();

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin 			= Uart_PCDbug_TX_PIN;
    GPIO_InitStruct.Alternate = Uart_PCDbug_GPIO_AF;
    HAL_GPIO_Init ( Uart_PCDbug_TX_GPIO_PORT, &GPIO_InitStruct );

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin 			= Uart_PCDbug_RX_PIN;
    GPIO_InitStruct.Alternate = Uart_PCDbug_GPIO_AF;
    HAL_GPIO_Init ( Uart_PCDbug_RX_GPIO_PORT, &GPIO_InitStruct );


    Uart_PCDbug_Handle->CR1 = 0;
    Uart_PCDbug_Handle->BRR  = Uart_PCDbug_GetBrrByBaud ( Uart_PCDbug_Handle, baud );
    Uart_PCDbug_Handle->CR1 &= 0xFFFF0000;
    Uart_PCDbug_Handle->CR2 &= 0xFFFF8090;
    Uart_PCDbug_Handle->CR3 &= 0xFFFFF000;
    Uart_PCDbug_Handle->CR1 |= ( 1 << 2 ) | ( 1 << 3 ) | ( 1 << 4 ) | ( 1 << 6 ); //2:����ʹ�� 3:����ʹ�� 4:�����ж� 6:��������ж�
    Uart_PCDbug_Handle->CR3 |= ( 1 << 6 ) | ( 1 << 7 ); //6:DMA����ʹ�� 7:DMA����ʹ��
    Uart_PCDbug_Handle->CR1 |= ( 1 << 13 ); 				// 13:����ʹ��

    HAL_NVIC_SetPriority ( Uart_PCDbug_IRQn, IRQ_Priority_UART, 0 );
    HAL_NVIC_EnableIRQ ( Uart_PCDbug_IRQn );

    DMA_Init();

//    sprintf((char *)TXBuf,"Uart_PCDbug_Send");
//    Uart_PCDbug_Send((uint8_t *) TXBuf, 16);

    return 0;
}

/*! \fn				void Uart_PCDbug_IRQHandler(void)
 *  \brief 		UART interrupt processing
 *  \param 		none
 *  \return 	none
 */
void Uart_PCDbug_IRQHandler ( void )
{
    uint32_t sr;
    uint32_t data;

    NVIC_ClearPendingIRQ ( Uart_PCDbug_IRQn );

    sr = Uart_PCDbug_Handle->SR;
    data = Uart_PCDbug_Handle->DR;
    ( void ) data;

    if ( sr & ( 1 << 4 ) ) //���߿���(�������)
    {
        u32_PCDbug_RxCplFlag = 1;
        u32_PCDbug_RxLen = Uart_PCDbug_MAX_COMM_LENTH - Uart_PCDbug_RX_DMA_STREAM->NDTR;
        Uart_PCDbug_RX_DMA_STREAM->CR &= ~0x01;
        Uart_PCDbug_RX_DMA_IRQ_CLEAR();
    }


    if ( sr & ( 1 << 6 ) ) //�������
    {
        Uart_PCDbug_Handle->SR &= ~ ( 1 << 6 );
        u8_PCDbug_TxMutex = 0;
    }
}

/*! \fn				void Uart_PCDbug_RX_DMA_IRQ_Handler(void)
 *  \brief 		The DMA correspond to the UART RX interrupt processing
 *  \param 		none
 *  \return 	none
 */
void Uart_PCDbug_RX_DMA_IRQ_Handler ( void )
{
    Uart_PCDbug_RX_DMA_IRQ_CLEAR();
    u32_PCDbug_RxCplFlag = 0;
    u32_PCDbug_RxLen = 0;

    Uart_PCDbug_RX_DMA_STREAM->CR  &= ~0x01;
    Uart_PCDbug_RX_DMA_STREAM->NDTR = Uart_PCDbug_MAX_COMM_LENTH;
    Uart_PCDbug_RX_DMA_STREAM->CR  |= 0x01;
}

/*! \fn				void Uart_PCDbug_TX_DMA_IRQ_Handler(void)
 *  \brief 		The DMA correspond to the UART TX interrupt processing
 *  \param 		none
 *  \return 	none
 */
void Uart_PCDbug_TX_DMA_IRQ_Handler ( void )
{
    Uart_PCDbug_TX_DMA_IRQ_CLEAR();
}

/*! \fn				uint8_t Uart_PCDbug_Receive(uint8_t *P,uint32_t *len)
 *  \brief 		UART receive from  pu8_PCDbug_RxBuf
 *  \param 		Pointer to destination pointer
 *  \param 		Pointer to lenth
 *  \return 	1:One frame of data is received  0;none data
 */
uint8_t Uart_PCDbug_Receive ( uint8_t *P, uint16_t *len )
{
    uint8_t ret;

    if ( u32_PCDbug_RxCplFlag )
    {
        if ( u32_PCDbug_RxLen > *len ) u32_PCDbug_RxLen = *len; //��ֹ���
        memcpy ( P, pu8_PCDbug_RxBuf, u32_PCDbug_RxLen );
        *len = u32_PCDbug_RxLen;

        u32_PCDbug_RxCplFlag = 0;
        u32_PCDbug_RxLen = 0;

        Uart_PCDbug_RX_DMA_STREAM->CR  &= ~0x01;
        while ( ( Uart_PCDbug_RX_DMA_STREAM->CR & 0x01 ) == 1 );
        Uart_PCDbug_RX_DMA_STREAM->NDTR = Uart_PCDbug_MAX_COMM_LENTH;
        Uart_PCDbug_RX_DMA_STREAM->CR  |= 0x01;

        ret = 1;
    }
    else
    {
        *len = 0;
        ret = 0;
    }

    return ret;
}

/*! \fn				static uint8_t Uart_PCDbug_Send(uint8_t *P,uint32_t len,uint32_t tick)
 *  \brief 		UART send data
 *  \param 		Pointer to source buff
 *  \param 		Pointer to lenth
 *  \return 	0:success  other;failed
 */
uint8_t Uart_PCDbug_Send ( uint8_t *P, uint32_t len )
{
    uint8_t ret;

    if ( ( len > 0 ) || ( len <= Uart_PCDbug_MAX_COMM_LENTH ) )
    {
        if ( u8_PCDbug_TxMutex == 0 )
        {
            memcpy ( pu8_PCDbug_TxBuf, P, len );

            Uart_PCDbug_TX_DMA_STREAM->CR &=  ~0x1;
            while ( ( Uart_PCDbug_TX_DMA_STREAM->CR & 0x01 ) == 1 );
            Uart_PCDbug_TX_DMA_STREAM->NDTR = len;						//���ݴ�������
            Uart_PCDbug_TX_DMA_STREAM->CR |=  0x1;							//ͨ��ʹ��

            u8_PCDbug_TxMutex = 1;
            ret = 0;
        }
        else
        {
            ret = 1;
            u8_PCDbug_TxMutex = 0;
        }
    }
    else
    {
        ret = 2;
    }

    return ret;
}

void Bsp_printf_PCDbug ( char* fmt, ... )
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
        Uart_PCDbug_Send ( ( uint8_t* ) buf, len );
    }
#else
    {
        len = 8;
        Uart_PCDbug_Send ( ( uint8_t* ) buf, len );
    }
#endif

}
//-------------------- private functions ------------------------------------

static void DMA_Init ( void )
{
    Uart_PCDbug_TX_DMA_ClK_ENABLE();
    Uart_PCDbug_RX_DMA_ClK_ENABLE();

    /***********************************RX DMA config*******************************/
    Uart_PCDbug_RX_DMA_STREAM->CR = 0;
    Uart_PCDbug_RX_DMA_STREAM->CR = Uart_PCDbug_RX_DMA_CHANNEL            //ͨ��
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

    Uart_PCDbug_RX_DMA_STREAM->M0AR = ( uint32_t ) pu8_PCDbug_RxBuf;				//�洢����ַ
    Uart_PCDbug_RX_DMA_STREAM->PAR = ( uint32_t ) &Uart_PCDbug_Handle->DR;	//�����ַ
    Uart_PCDbug_RX_DMA_STREAM->NDTR = Uart_PCDbug_MAX_COMM_LENTH;				//���ݴ�������
    Uart_PCDbug_RX_DMA_STREAM->FCR =   0x01 << DMA_SxFCR_FEIE_Pos //FIFO�����ж�
                                       | 0x00 << DMA_SxFCR_DMDIS_Pos; //ֱ��ģʽ


    HAL_NVIC_SetPriority ( Uart_PCDbug_RX_DMA_IRQ, IRQ_Priority_UART, 0 );
    HAL_NVIC_EnableIRQ ( Uart_PCDbug_RX_DMA_IRQ );

    Uart_PCDbug_RX_DMA_STREAM->CR |=  0x01; //ʹ��ͨ��

    /***********************************TX DMA config*******************************/

    Uart_PCDbug_TX_DMA_STREAM->CR = 0;
    Uart_PCDbug_TX_DMA_STREAM->CR = Uart_PCDbug_RX_DMA_CHANNEL            //ͨ��
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

    Uart_PCDbug_TX_DMA_STREAM->M0AR = ( uint32_t ) pu8_PCDbug_TxBuf;				//�洢����ַ
    Uart_PCDbug_TX_DMA_STREAM->PAR  = ( uint32_t ) &Uart_PCDbug_Handle->DR;	//�����ַ
    Uart_PCDbug_TX_DMA_STREAM->NDTR = 0;				                  //���ݴ�������
    Uart_PCDbug_TX_DMA_STREAM->FCR  =   0x01 << DMA_SxFCR_FEIE_Pos //FIFO�����ж�
                                        | 0x00 << DMA_SxFCR_DMDIS_Pos; //ֱ��ģʽ

    HAL_NVIC_SetPriority ( Uart_PCDbug_TX_DMA_IRQ, IRQ_Priority_UART, 0 );
    HAL_NVIC_EnableIRQ ( Uart_PCDbug_TX_DMA_IRQ );

//	Uart_PCDbug_TX_DMA_STREAM->CR |=  0x01; //ʹ��ͨ��

}

/*! \fn				static uint16_t Uart_PCDbug_GetBrrByBaud(USART_TypeDef* UsartType,uint32_t Baud)
 *  \brief 		Get brr data by baud
 *  \param 		USARTx channel
 *  \param 		baud
 *  \return 	brr data
 */
static uint16_t Uart_PCDbug_GetBrrByBaud ( USART_TypeDef* UsartType, uint32_t Baud )
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


