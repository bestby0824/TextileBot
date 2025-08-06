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
#include "CAN_VCU.h"
#include "main.h"
#include "VCUProcess.h"
#include "CAN_Node.h"

//-------------------- public data ------------------------------------------
CAN_HandleTypeDef     Can_VCUHandle;     /* Can_VCU��� */

CAN_BAUD_Struct CAN_VCUBaudTab[] =
{
    /*0*/ {125,  CAN_SJW_1TQ, CAN_BS1_4TQ, CAN_BS2_2TQ, 48},
    /*1*/ {250,  CAN_SJW_1TQ, CAN_BS1_4TQ, CAN_BS2_2TQ, 24},
    /*2*/ {500,  CAN_SJW_1TQ, CAN_BS1_4TQ, CAN_BS2_2TQ, 12},
    /*3*/ {1000, CAN_SJW_1TQ, CAN_BS1_4TQ, CAN_BS2_2TQ, 6}
};

//-------------------- public functions -------------------------------------
void CanBusOff_Reset ( void );

/**
 * @brief  Configures the Can_VCU.
 * @param  None
 * @retval None
 *   @note      ����4������, �ں����ڲ����1, ����, �κ�һ�����������ܵ���0
 *              CAN����APB1����, ������ʱ��Ƶ��Ϊ Fpclk1 = PCLK1 = 42Mhz
 *              tq     = brp * tpclk1;
 *              ������ = Fpclk1 / ((tbs1 + tbs2 + 1) * brp);
 *              ��CAN������Ϊ:42M / ((4 + 2 + 1) * 6) = 1000Kbps
 */
uint8_t Can_VCU_Config ( uint32_t baud )
{
    uint8_t can_baud_index = 0;
    CAN_FilterTypeDef     sFilterConfig;

    /*##-1- Configure the Can_VCU peripheral #######################################*/
    Can_VCUHandle.Instance = Can_VCU;

    Can_VCUHandle.Init.TimeTriggeredMode = DISABLE;
    Can_VCUHandle.Init.AutoBusOff = DISABLE;
    Can_VCUHandle.Init.AutoWakeUp = DISABLE;
    Can_VCUHandle.Init.AutoRetransmission = ENABLE;
    Can_VCUHandle.Init.ReceiveFifoLocked = DISABLE;
    Can_VCUHandle.Init.TransmitFifoPriority = DISABLE;
    Can_VCUHandle.Init.Mode = CAN_MODE_NORMAL;

//    Can_VCUHandle.Init.SyncJumpWidth = CAN_SJW_1TQ;
//    Can_VCUHandle.Init.TimeSeg1 = CAN_BS1_4TQ;
//    Can_VCUHandle.Init.TimeSeg2 = CAN_BS2_2TQ;
//    Can_VCUHandle.Init.Prescaler = 24;
    for ( can_baud_index = 0; can_baud_index < 4; can_baud_index++ )
    {
        if ( CAN_VCUBaudTab[can_baud_index].BPS == baud ) break;
    }
    Can_VCUHandle.Init.SyncJumpWidth = CAN_VCUBaudTab[can_baud_index].SJW;
    Can_VCUHandle.Init.TimeSeg1 = CAN_VCUBaudTab[can_baud_index].BS1;
    Can_VCUHandle.Init.TimeSeg2 = CAN_VCUBaudTab[can_baud_index].BS2;
    Can_VCUHandle.Init.Prescaler = CAN_VCUBaudTab[can_baud_index].Prescaler;


    if ( HAL_CAN_Init ( &Can_VCUHandle ) != HAL_OK )
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

    if ( HAL_CAN_ConfigFilter ( &Can_VCUHandle, &sFilterConfig ) != HAL_OK )
    {
        /* Filter configuration Error */
        return 2;
    }

    /*##-3- Start the CAN peripheral ###########################################*/
    if ( HAL_CAN_Start ( &Can_VCUHandle ) != HAL_OK )
    {
        /* Start Error */
        return 3;
    }

    /*##-4- Activate CAN RX notification #######################################*/
    if ( HAL_CAN_ActivateNotification ( &Can_VCUHandle, CAN_IT_RX_FIFO0_MSG_PENDING ) != HAL_OK )
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

    if ( hcan->Instance == Can_VCU )
    {
        /*##-1- Enable peripherals and GPIO Clocks #################################*/
        /* CAN1 Periph clock enable */
        Can_VCU_CLK_ENABLE();
        /* Enable GPIO clock ****************************************/
        Can_VCU_GPIO_CLK_ENABLE();

        /*##-2- Configure peripheral GPIO ##########################################*/
        /* CAN1 TX GPIO pin configuration */
        GPIO_InitStruct.Pin = Can_VCU_TX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Alternate =  Can_VCU_TX_AF;

        HAL_GPIO_Init ( Can_VCU_TX_GPIO_PORT, &GPIO_InitStruct );

        /* CAN1 RX GPIO pin configuration */
        GPIO_InitStruct.Pin = Can_VCU_RX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Alternate =  Can_VCU_RX_AF;

        HAL_GPIO_Init ( Can_VCU_RX_GPIO_PORT, &GPIO_InitStruct );

        /*##-3- Configure the NVIC #################################################*/
        /* NVIC configuration for CAN1 Reception complete interrupt */
        HAL_NVIC_SetPriority ( Can_VCU_RX_IRQn, IRQ_Priority_CAN, 0 );
        HAL_NVIC_EnableIRQ ( Can_VCU_RX_IRQn );
    }
    else
    {
        /*##-1- Enable peripherals and GPIO Clocks #################################*/
        /* CAN2 Periph clock enable */
        Can_VCU_CLK_ENABLE();
        Can_Node_CLK_ENABLE();
        /* Enable GPIO clock ****************************************/
        Can_Node_GPIO_CLK_ENABLE();

        /*##-2- Configure peripheral GPIO ##########################################*/
        /* CAN2 TX GPIO pin configuration */
        GPIO_InitStruct.Pin = Can_Node_TX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Alternate =  Can_Node_TX_AF;

        HAL_GPIO_Init ( Can_Node_TX_GPIO_PORT, &GPIO_InitStruct );

        /* CAN2 RX GPIO pin configuration */
        GPIO_InitStruct.Pin = Can_Node_RX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Alternate =  Can_Node_RX_AF;

        HAL_GPIO_Init ( Can_Node_RX_GPIO_PORT, &GPIO_InitStruct );

        /*##-3- Configure the NVIC #################################################*/
        /* NVIC configuration for CAN2 Reception complete interrupt */
        HAL_NVIC_SetPriority ( Can_Node_RX_IRQn, IRQ_Priority_CAN, 0 );
        HAL_NVIC_EnableIRQ ( Can_Node_RX_IRQn );
    }
}

/**
 * @brief       CAN RX0 �жϷ�����
 *   @note      ����CAN FIFO0�Ľ����ж�
 * @param       ��
 * @retval      ��
 */
void Can_VCU_RX_IRQHandler ( void )
{
    HAL_CAN_IRQHandler ( &Can_VCUHandle );

    Can_VCU_Receive();
}
/**
 * @brief       CAN ����һ������
 *   @note      ���͸�ʽ�̶�Ϊ: ��׼ID, ����֡
 * @param       id      : ��׼ID(11λ)
 * @param       msg     : ����ָ��
 * @param       len     : ���ݳ���
 * @retval      ����״̬ 0, �ɹ�; 1, ʧ��;
 */
uint8_t Can_VCU_send_msg ( uint32_t id, uint8_t *msg, uint8_t len )
{
    uint16_t t = 0;
    uint32_t TxMailbox = CAN_TX_MAILBOX0;
    CAN_TxHeaderTypeDef   TxHeader;      /* ���Ͳ������ */

    TxHeader.StdId = id;         /* ��׼��ʶ�� */
    TxHeader.ExtId = id;         /* ��չ��ʶ��(29λ) */
    TxHeader.IDE = CAN_ID_STD;   /* ʹ�ñ�׼֡ */
    TxHeader.RTR = CAN_RTR_DATA; /* ����֡ */
    TxHeader.DLC = len;

    t = 0;

    while ( HAL_CAN_GetTxMailboxesFreeLevel ( &Can_VCUHandle ) != 3 ) /* �ȴ��������,��������Ϊ�� */
    {
        t++;
        if ( t > 0xF )
        {
            HAL_CAN_AbortTxRequest ( &Can_VCUHandle, TxMailbox );  /* ��ʱ��ֱ����ֹ����ķ������� */
            return 1;
        }
    }

    if ( HAL_CAN_AddTxMessage ( &Can_VCUHandle, &TxHeader, msg, &TxMailbox ) != HAL_OK ) /* ������Ϣ */
    {
        return 1;
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
uint32_t Can_VCU_receive_msg ( uint8_t *buf )
{
    CAN_RxHeaderTypeDef   RxHeader;      /* ���ղ������ */

    if ( HAL_CAN_GetRxFifoFillLevel ( &Can_VCUHandle, CAN_RX_FIFO0 ) == 0 ) /* û�н��յ����� */
    {
        return 0;
    }

    if ( HAL_CAN_GetRxMessage ( &Can_VCUHandle, CAN_RX_FIFO0, &RxHeader, buf ) != HAL_OK ) /* ��ȡ���� */
    {
        return 0;
    }

    if ( RxHeader.IDE != CAN_ID_STD || RxHeader.RTR != CAN_RTR_DATA )    /* ���յ����ݳ��Ȳ���8λ / ���Ǳ�׼֡ / ��������֡ RxHeader.DLC != 8 || */
    {
        return 0;
    }

    return RxHeader.StdId;
}

uint32_t  u32Can1RsetTimes = 0, u32Can2RsetTimes = 0; //��λ����
/**
 * @brief        CAN BUSOFF��⴦����
 *   @note      BUSOFFʱ����λCAN����������
 * @param       ��
 * @retval      ��
 */
void CanBusOff_Reset ( void )
{
    static uint32_t u32Can1ErrFlag = 0, u32Can2ErrFlag = 0;

    u32Can1ErrFlag = ( Can_VCU->ESR ) & 0x00000007;
    u32Can2ErrFlag = ( Can_Node->ESR ) & 0x00000007;

    if ( u32Can1ErrFlag == 0x7 )
    {
        u32Can1RsetTimes++;
        HAL_CAN_AbortTxRequest ( &Can_VCUHandle, CAN_TX_MAILBOX0 ); //��ֹ���䷢������
        HAL_CAN_AbortTxRequest ( &Can_VCUHandle, CAN_TX_MAILBOX1 );
        HAL_CAN_AbortTxRequest ( &Can_VCUHandle, CAN_TX_MAILBOX2 );

        Can_VCU->MCR |= 0x00008000;  //��λ
//        Can_VCU->IER |= 0x00000000;  //����ж�
//        Can_VCU->ESR |= 0x00000000;  //CAN_ESR_REC_Msk | CAN_ESR_TEC_Msk;  //��REC��TEC�Ĵ�����ֵ����
        Can_VCU->MCR &= 0xFFFFFFFC;  //���Sleep��־

//        Can_VCU_Config ( 250 );
    }

    if ( u32Can2ErrFlag == 0x7 )
    {
        u32Can2RsetTimes++;
        HAL_CAN_AbortTxRequest ( &Can_NodeHandle, CAN_TX_MAILBOX0 ); //��ֹ���䷢������
        HAL_CAN_AbortTxRequest ( &Can_NodeHandle, CAN_TX_MAILBOX1 );
        HAL_CAN_AbortTxRequest ( &Can_NodeHandle, CAN_TX_MAILBOX2 );

        Can_Node->MCR |= 0x00008000;  //��λ
//        Can_Node->IER |= 0x00000000;  //����ж�
//        Can_Node->ESR |= 0x00000000;  //CAN_ESR_REC_Msk | CAN_ESR_TEC_Msk;  //��REC��TEC�Ĵ�����ֵ����
        Can_Node->MCR &= 0xFFFFFFFC;  //���Sleep��־

//        Can_Node_Config ( 1000 );
    }
}
