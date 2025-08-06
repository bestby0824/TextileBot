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
#include "CAN_Node.h"
#include "IMU_Com.h"
#include "CAN_VCU.h"
#include "NodeProcess.h"

CAN_HandleTypeDef     Can_NodeHandle;     /* CANx��� */

CAN_BAUD_Struct CAN_NodeBaudTab[] =
{
    /*0*/ {125,  CAN_SJW_1TQ, CAN_BS1_4TQ, CAN_BS2_2TQ, 48},
    /*1*/ {250,  CAN_SJW_1TQ, CAN_BS1_4TQ, CAN_BS2_2TQ, 24},
    /*2*/ {500,  CAN_SJW_1TQ, CAN_BS1_4TQ, CAN_BS2_2TQ, 12},
    /*3*/ {1000, CAN_SJW_1TQ, CAN_BS1_4TQ, CAN_BS2_2TQ, 6}
};

/**
 * @brief  Configures the CAN.
 * @param  None
 * @retval None
 *   @note      ����4������, �ں����ڲ����1, ����, �κ�һ�����������ܵ���0
 *              CAN����APB1����, ������ʱ��Ƶ��Ϊ Fpclk1 = PCLK1 = 42Mhz
 *              tq     = brp * tpclk1;
 *              ������ = Fpclk1 / ((tbs1 + tbs2 + 1) * brp);
 *              ��CAN������Ϊ:42M / ((4 + 2 + 1) * 6) = 1000Kbps
 */
uint8_t Can_Node_Config ( uint32_t baud )
{
    uint8_t can_baud_index = 0;
    CAN_FilterTypeDef  sFilterConfig;

    /*##-1- Configure the CAN peripheral #######################################*/
    Can_NodeHandle.Instance = Can_Node;

    Can_NodeHandle.Init.TimeTriggeredMode = DISABLE;
    Can_NodeHandle.Init.AutoBusOff = DISABLE;
    Can_NodeHandle.Init.AutoWakeUp = DISABLE;
    Can_NodeHandle.Init.AutoRetransmission = ENABLE;
    Can_NodeHandle.Init.ReceiveFifoLocked = DISABLE;
    Can_NodeHandle.Init.TransmitFifoPriority = DISABLE;
    Can_NodeHandle.Init.Mode = CAN_MODE_NORMAL;

    for ( can_baud_index = 0; can_baud_index < 4; can_baud_index++ )
    {
        if ( CAN_NodeBaudTab[can_baud_index].BPS == baud ) break;
    }
    Can_NodeHandle.Init.SyncJumpWidth = CAN_NodeBaudTab[can_baud_index].SJW;
    Can_NodeHandle.Init.TimeSeg1 = CAN_NodeBaudTab[can_baud_index].BS1;
    Can_NodeHandle.Init.TimeSeg2 = CAN_NodeBaudTab[can_baud_index].BS2;
    Can_NodeHandle.Init.Prescaler = CAN_NodeBaudTab[can_baud_index].Prescaler;

    if ( HAL_CAN_Init ( &Can_NodeHandle ) != HAL_OK )
    {
        /* Initialization Error */
        return 1;
    }

    /*##-2- Configure the CAN Filter ###########################################*/
    sFilterConfig.FilterBank = 14;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if ( HAL_CAN_ConfigFilter ( &Can_NodeHandle, &sFilterConfig ) != HAL_OK )
    {
        /* Filter configuration Error */
        return 2;
    }

    /*##-3- Start the CAN peripheral ###########################################*/
    if ( HAL_CAN_Start ( &Can_NodeHandle ) != HAL_OK )
    {
        /* Start Error */
        return 3;
    }

    /*##-4- Activate CAN RX notification #######################################*/
    if ( HAL_CAN_ActivateNotification ( &Can_NodeHandle, CAN_IT_RX_FIFO1_MSG_PENDING ) != HAL_OK )
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
//void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
//{
//    GPIO_InitTypeDef   GPIO_InitStruct;

//    /*##-1- Enable peripherals and GPIO Clocks #################################*/
//    /* CAN2 Periph clock enable */
//    Can_Node_CLK_ENABLE();
//    /* Enable GPIO clock ****************************************/
//    Can_Node_GPIO_CLK_ENABLE();

//    /*##-2- Configure peripheral GPIO ##########################################*/
//    /* CAN2 TX GPIO pin configuration */
//    GPIO_InitStruct.Pin = Can_Node_TX_PIN;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Pull = GPIO_PULLUP;
//    GPIO_InitStruct.Alternate =  Can_Node_TX_AF;

//    HAL_GPIO_Init(Can_Node_TX_GPIO_PORT, &GPIO_InitStruct);

//    /* CAN2 RX GPIO pin configuration */
//    GPIO_InitStruct.Pin = Can_Node_RX_PIN;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Pull = GPIO_PULLUP;
//    GPIO_InitStruct.Alternate =  Can_Node_RX_AF;

//    HAL_GPIO_Init(Can_Node_RX_GPIO_PORT, &GPIO_InitStruct);

//    /*##-3- Configure the NVIC #################################################*/
//    /* NVIC configuration for CAN2 Reception complete interrupt */
//    HAL_NVIC_SetPriority(Can_Node_RX_IRQn, IRQ_Priority_CAN, 0);
//    HAL_NVIC_EnableIRQ(Can_Node_RX_IRQn);
//}

/**
 * @brief       CAN RX0 �жϷ�����
 *   @note      ����CAN FIFO0�Ľ����ж�
 * @param       ��
 * @retval      ��
 */
void Can_Node_RX_IRQHandler ( void )
{
    HAL_CAN_IRQHandler ( &Can_NodeHandle );
    Can_Node_Receive();
//    IMUData_Receive ( );
}/**
 * @brief       CAN ����һ������
 *   @note      ���͸�ʽ�̶�Ϊ: ��׼ID, ����֡
 * @param       id      : ��׼ID(11λ)
 * @param       msg     : ����ָ��
 * @param       len     : ���ݳ���
 * @retval      ����״̬ 0, �ɹ�; 1, ʧ��;
 */
uint8_t Can_Node_send_msg ( uint32_t id, uint8_t *msg, uint8_t len )
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

    while ( HAL_CAN_GetTxMailboxesFreeLevel ( &Can_NodeHandle ) != 3 ) /* �ȴ��������,��������Ϊ�� */
    {
        t++;
        if ( t > 0xF )
        {
            HAL_CAN_AbortTxRequest ( &Can_NodeHandle, TxMailbox );  /* ��ʱ��ֱ����ֹ����ķ������� */
            return 1;
        }
    }

    if ( HAL_CAN_AddTxMessage ( &Can_NodeHandle, &TxHeader, msg, &TxMailbox ) != HAL_OK ) /* ������Ϣ */
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
uint32_t Can_Node_receive_msg ( uint8_t *buf )
{
    CAN_RxHeaderTypeDef   RxHeader;      /* ���ղ������ */

    if ( HAL_CAN_GetRxFifoFillLevel ( &Can_NodeHandle, CAN_RX_FIFO1 ) == 0 ) /* û�н��յ����� */
    {
        return 0;
    }

    if ( HAL_CAN_GetRxMessage ( &Can_NodeHandle, CAN_RX_FIFO1, &RxHeader, buf ) != HAL_OK ) /* ��ȡ���� */
    {
        return 0;
    }

    if ( RxHeader.IDE != CAN_ID_STD || RxHeader.RTR != CAN_RTR_DATA )    /* ���յ����ݳ��Ȳ���8λRxHeader.DLC != 8 ||  / ���Ǳ�׼֡ / ��������֡ */
    {
        return 0;
    }

    return RxHeader.StdId;
}
