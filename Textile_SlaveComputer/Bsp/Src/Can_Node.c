/**
* 版权所有(C)
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
* <日期> | <版本> | <作者> | <描述>
*
* xxxx/xx/xx | 1.0.0 | MUNIU | 创建文件
*
*/
//------------------------------------------------------------------------------
//-------------------- pragmas ----------------------------------------------
//-------------------- include files ----------------------------------------
#include "CAN_Node.h"
#include "IMU_Com.h"
#include "CAN_VCU.h"
#include "NodeProcess.h"

CAN_HandleTypeDef     Can_NodeHandle;     /* CANx句柄 */

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
 *   @note      以上4个参数, 在函数内部会减1, 所以, 任何一个参数都不能等于0
 *              CAN挂在APB1上面, 其输入时钟频率为 Fpclk1 = PCLK1 = 42Mhz
 *              tq     = brp * tpclk1;
 *              波特率 = Fpclk1 / ((tbs1 + tbs2 + 1) * brp);
 *              则CAN波特率为:42M / ((4 + 2 + 1) * 6) = 1000Kbps
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
 * @brief       CAN底层驱动，引脚配置，时钟配置，中断配置
                此函数会被HAL_CAN_Init()调用
 * @param       hcan:CAN句柄
 * @retval      无
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
 * @brief       CAN RX0 中断服务函数
 *   @note      处理CAN FIFO0的接收中断
 * @param       无
 * @retval      无
 */
void Can_Node_RX_IRQHandler ( void )
{
    HAL_CAN_IRQHandler ( &Can_NodeHandle );
    Can_Node_Receive();
//    IMUData_Receive ( );
}/**
 * @brief       CAN 发送一组数据
 *   @note      发送格式固定为: 标准ID, 数据帧
 * @param       id      : 标准ID(11位)
 * @param       msg     : 数据指针
 * @param       len     : 数据长度
 * @retval      发送状态 0, 成功; 1, 失败;
 */
uint8_t Can_Node_send_msg ( uint32_t id, uint8_t *msg, uint8_t len )
{
    uint16_t t = 0;
    uint32_t TxMailbox = CAN_TX_MAILBOX0;
    CAN_TxHeaderTypeDef   TxHeader;      /* 发送参数句柄 */

    TxHeader.StdId = id;         /* 标准标识符 */
    TxHeader.ExtId = id;         /* 扩展标识符(29位) */
    TxHeader.IDE = CAN_ID_STD;   /* 使用标准帧 */
    TxHeader.RTR = CAN_RTR_DATA; /* 数据帧 */
    TxHeader.DLC = len;

    t = 0;

    while ( HAL_CAN_GetTxMailboxesFreeLevel ( &Can_NodeHandle ) != 3 ) /* 等待发送完成,所有邮箱为空 */
    {
        t++;
        if ( t > 0xF )
        {
            HAL_CAN_AbortTxRequest ( &Can_NodeHandle, TxMailbox );  /* 超时，直接中止邮箱的发送请求 */
            return 1;
        }
    }

    if ( HAL_CAN_AddTxMessage ( &Can_NodeHandle, &TxHeader, msg, &TxMailbox ) != HAL_OK ) /* 发送消息 */
    {
        return 1;
    }

    return 0;
}

/**
 * @brief       CAN 接收数据查询
 *   @note      接收数据格式固定为: 标准ID, 数据帧
 * @param       id      : 要查询的 标准ID(11位)
 * @param       buf     : 数据缓存区
 * @retval      接收结果
 *   @arg       0   , 无数据被接收到;
 *   @arg       其他, 接收的数据长度
 */
uint32_t Can_Node_receive_msg ( uint8_t *buf )
{
    CAN_RxHeaderTypeDef   RxHeader;      /* 接收参数句柄 */

    if ( HAL_CAN_GetRxFifoFillLevel ( &Can_NodeHandle, CAN_RX_FIFO1 ) == 0 ) /* 没有接收到数据 */
    {
        return 0;
    }

    if ( HAL_CAN_GetRxMessage ( &Can_NodeHandle, CAN_RX_FIFO1, &RxHeader, buf ) != HAL_OK ) /* 读取数据 */
    {
        return 0;
    }

    if ( RxHeader.IDE != CAN_ID_STD || RxHeader.RTR != CAN_RTR_DATA )    /* 接收到数据长度不是8位RxHeader.DLC != 8 ||  / 不是标准帧 / 不是数据帧 */
    {
        return 0;
    }

    return RxHeader.StdId;
}
