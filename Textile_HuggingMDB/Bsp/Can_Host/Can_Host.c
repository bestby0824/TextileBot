/**
* ��Ȩ����(C)
*
* ********
*
* @file
* @brief
* @details
* @author MUNIU
* @version 1.0.0
* @date xxxx-xx-xx
* @par Description:
* @par Change History:
*
* <����> | <�汾> | <����> | <����>
*
* xxxx/xx/xx | 1.0.0 | MUNIU | �����ļ�
*
*/
//------------------------------------------------------------------------------
//-------------------- pragmas ----------------------------------------------
//-------------------- include files ----------------------------------------
#include "Can_Host.h"
#include "main.h"
#include "Int_Ctrl.h"
#include "HostProcess.h"
#include "Monitor.h"

//-------------------- public data ------------------------------------------
CAN_HandleTypeDef     Can_HostHandle;     /* Can_Host��� */

CAN_BAUD_Struct Can_HostBaudTab[] =
{
    /*0*/ {125,  CAN_SJW_1TQ, CAN_BS1_4TQ, CAN_BS2_2TQ, 48},
    /*1*/ {250,  CAN_SJW_1TQ, CAN_BS1_4TQ, CAN_BS2_2TQ, 24},
    /*2*/ {500,  CAN_SJW_1TQ, CAN_BS1_4TQ, CAN_BS2_2TQ, 12},
    /*3*/ {1000, CAN_SJW_1TQ, CAN_BS1_4TQ, CAN_BS2_2TQ, 6}
};

//-------------------- public functions -------------------------------------
void CanBusOff_Reset ( void );

/**
 * @brief  Configures the Can_Host.
 * @param  None
 * @retval None
 *   @note      ����4������, �ں����ڲ����1, ����, �κ�һ�����������ܵ���0
 *              CAN����APB1����, ������ʱ��Ƶ��Ϊ Fpclk1 = PCLK1 = 42Mhz
 *              tq     = brp * tpclk1;
 *              ������ = Fpclk1 / ((tbs1 + tbs2 + 1) * brp);
 *              ��CAN������Ϊ:42M / ((4 + 2 + 1) * 6) = 1000Kbps
 */
uint8_t Can_Host_Config ( uint32_t baud )
{
    uint8_t can_baud_index = 0;
    CAN_FilterTypeDef     sFilterConfig;

    /*##-1- Configure the Can_Host peripheral #######################################*/
    Can_HostHandle.Instance = Can_Host;

    Can_HostHandle.Init.TimeTriggeredMode = DISABLE;
    Can_HostHandle.Init.AutoBusOff = DISABLE;
    Can_HostHandle.Init.AutoWakeUp = DISABLE;
    Can_HostHandle.Init.AutoRetransmission = ENABLE;
    Can_HostHandle.Init.ReceiveFifoLocked = DISABLE;
    Can_HostHandle.Init.TransmitFifoPriority = DISABLE;
    Can_HostHandle.Init.Mode = CAN_MODE_NORMAL;

    for ( can_baud_index = 0; can_baud_index < 4; can_baud_index++ )
    {
        if ( Can_HostBaudTab[can_baud_index].BPS == baud ) break;
    }
    Can_HostHandle.Init.SyncJumpWidth = Can_HostBaudTab[can_baud_index].SJW;
    Can_HostHandle.Init.TimeSeg1 = Can_HostBaudTab[can_baud_index].BS1;
    Can_HostHandle.Init.TimeSeg2 = Can_HostBaudTab[can_baud_index].BS2;
    Can_HostHandle.Init.Prescaler = Can_HostBaudTab[can_baud_index].Prescaler;


    if ( HAL_CAN_Init ( &Can_HostHandle ) != HAL_OK )
    {
        /* Initialization Error */
        return 1;
    }

    /*##-2- Configure the CAN Filter ###########################################*/
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if ( HAL_CAN_ConfigFilter ( &Can_HostHandle, &sFilterConfig ) != HAL_OK )
    {
        /* Filter configuration Error */
        return 2;
    }

    /*##-3- Start the CAN peripheral ###########################################*/
    if ( HAL_CAN_Start ( &Can_HostHandle ) != HAL_OK )
    {
        /* Start Error */
        return 3;
    }

    /*##-4- Activate CAN RX notification #######################################*/
    if ( HAL_CAN_ActivateNotification ( &Can_HostHandle, CAN_IT_RX_FIFO0_MSG_PENDING ) != HAL_OK )
    {
        /* Notification Error */
        return 4;
    }

    return 0;
}

/**
 * @brief       CAN�ײ��������������ã�ʱ�����ã��ж�����
                �˺����ᱻHAL_CAN_Init()����
 * @param       hcan:CAN���
 * @retval      ��
 */
void HAL_CAN_MspInit ( CAN_HandleTypeDef *hcan )
{
    GPIO_InitTypeDef   GPIO_InitStruct;

    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* CAN1 Periph clock enable */
    Can_Host_CLK_ENABLE();
    /* Enable GPIO clock ****************************************/
    Can_Host_GPIO_CLK_ENABLE();

    /*##-2- Configure peripheral GPIO ##########################################*/
    /* CAN1 TX GPIO pin configuration */
    GPIO_InitStruct.Pin = Can_Host_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Alternate =  Can_Host_TX_AF;

    HAL_GPIO_Init ( Can_Host_TX_GPIO_PORT, &GPIO_InitStruct );

    /* CAN1 RX GPIO pin configuration */
    GPIO_InitStruct.Pin = Can_Host_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Alternate =  Can_Host_RX_AF;

    HAL_GPIO_Init ( Can_Host_RX_GPIO_PORT, &GPIO_InitStruct );

    /*##-3- Configure the NVIC #################################################*/
    /* NVIC configuration for CAN1 Reception complete interrupt */
    HAL_NVIC_SetPriority ( Can_Host_RX_IRQn, IRQ_Priority_CAN, 0 );
    HAL_NVIC_EnableIRQ ( Can_Host_RX_IRQn );
}

/**
 * @brief       CAN RX0 �жϷ�����
 *   @note      ����CAN FIFO0�Ľ����ж�
 * @param       ��
 * @retval      ��
 */
void Can_Host_RX_IRQHandler ( void )
{
    HAL_CAN_IRQHandler ( &Can_HostHandle );

    Can_Host_Receive();
}
/**
 * @brief       CAN ����һ������
 *   @note      ���͸�ʽ�̶�Ϊ: ��׼ID, ����֡
 * @param       id      : ��׼ID(11λ)
 * @param       msg     : ����ָ��
 * @param       len     : ���ݳ���
 * @retval      ����״̬ 0, �ɹ�; 1, ʧ��;
 */
uint32_t TX_1 = 0,TX_2 = 0,TX_3 = 0;
uint8_t Can_Host_send_msg ( uint32_t id, uint8_t *msg, uint8_t len )
{
    uint16_t t = 0;
    uint32_t TxMailbox = CAN_TX_MAILBOX0;
    CAN_TxHeaderTypeDef   TxHeader;      /* ���Ͳ������ */

    TxHeader.StdId = id;         /* ��׼��ʶ�� */
    TxHeader.ExtId = id;         /* ��չ��ʶ��(29λ) */
    TxHeader.IDE = CAN_ID_STD;   /* ʹ�ñ�׼֡ */
    TxHeader.RTR = CAN_RTR_DATA; /* ����֡ */
    TxHeader.DLC = len;

    TX_1++;
    t = 0;
//    while ( HAL_CAN_GetTxMailboxesFreeLevel ( &Can_HostHandle ) != 3 ) /* �ȴ��������,��������Ϊ�� */
    while ( HAL_CAN_GetTxMailboxesFreeLevel ( &Can_HostHandle ) == 0 ) /* �ȴ��������,һ������Ϊ�� */
    {
        t++;
        if ( t > 0xF )
        {
            HAL_CAN_AbortTxRequest ( &Can_HostHandle, TxMailbox );  /* ��ʱ��ֱ����ֹ����ķ������� */
            return 1;
        }
    }

    TX_3++;
    if ( HAL_CAN_AddTxMessage ( &Can_HostHandle, &TxHeader, msg, &TxMailbox ) != HAL_OK ) /* ������Ϣ */
    {
        return 1;
    }
    else
    {
        TX_2++;
    }

    return 0;
}

/**
 * @brief       CAN �������ݲ�ѯ
 *   @note      �������ݸ�ʽ�̶�Ϊ: ��׼ID, ����֡
 * @param       id      : Ҫ��ѯ�� ��׼ID(11λ)
 * @param       buf     : ���ݻ�����
 * @retval      ���ս��
 *   @arg       0   , �����ݱ����յ�;
 *   @arg       ����, ���յ����ݳ���
 */
uint32_t Can_Host_receive_msg ( uint8_t *buf )
{
    CAN_RxHeaderTypeDef   RxHeader;      /* ���ղ������ */

    if ( HAL_CAN_GetRxFifoFillLevel ( &Can_HostHandle, CAN_RX_FIFO0 ) == 0 ) /* û�н��յ����� */
    {
        return 0;
    }

    if ( HAL_CAN_GetRxMessage ( &Can_HostHandle, CAN_RX_FIFO0, &RxHeader, buf ) != HAL_OK ) /* ��ȡ���� */
    {
        return 0;
    }

    if ( RxHeader.DLC != 8 || RxHeader.IDE != CAN_ID_STD || RxHeader.RTR != CAN_RTR_DATA )    /* ���յ����ݳ��Ȳ���8λ / ���Ǳ�׼֡ / ��������֡ */
    {
        return 0;
    }

    return RxHeader.StdId;
}
uint32_t  u32Can1RsetTimes = 0; //��λ����
/**
 * @brief        CAN BUSOFF��⴦����
 *   @note      BUSOFFʱ����λCAN����������
 * @param       ��
 * @retval      ��
 */
void CanBusOff_Reset ( void )
{
    static uint32_t u32Can1ErrFlag = 0, u32Can2ErrFlag = 0;

    u32Can1ErrFlag = ( Can_Host->ESR ) & 0x00000007;

    if ( u32Can1ErrFlag == 0x7 )
    {
        
        if ( WarnCheckEnable & BIT ( 6 ) )
        {
            g_u32WarningNum.bit.CAN_Board_BUSOFF = 1;
            if ( g_u32FaultNum.bit.CANComError == 0 ) u32Can1RsetTimes++;
            if ( ( FaultCheckEnable & ErrBit ( 12 ) ) && ( u32Can1RsetTimes > 5 ) )
            {
//                g_u32FaultNum.bit.CANComError = 1;
                u32Can1RsetTimes = 0;
            }
        }
        HAL_CAN_AbortTxRequest ( &Can_HostHandle, CAN_TX_MAILBOX0 ); //��ֹ���䷢������
        HAL_CAN_AbortTxRequest ( &Can_HostHandle, CAN_TX_MAILBOX1 );
        HAL_CAN_AbortTxRequest ( &Can_HostHandle, CAN_TX_MAILBOX2 );

        Can_Host->MCR |= 0x00008000;  //��λ
//        Can_Host->IER |= 0x00000000;  //����ж�
//        Can_Host->ESR |= 0x00000000;  //CAN_ESR_REC_Msk | CAN_ESR_TEC_Msk;  //��REC��TEC�Ĵ�����ֵ����
        Can_Host->MCR &= 0xFFFFFFFC;  //���Sleep��־

//        Can_Host_Config ( 1000 );
    }
}
