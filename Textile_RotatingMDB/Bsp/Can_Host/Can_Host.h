/**
* 版权所有(C)
*
* ********
*
* @file
* @brief
* @details
* @author   MUNIU
* @version  1.0.0
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _CAN_HOST_H_
#define _CAN_HOST_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor Can_Host instance used and associated
   resources */
/* Definition for Can_Host clock resources */
#define Can_Host                            CAN1
#define Can_Host_CLK_ENABLE()               __HAL_RCC_CAN1_CLK_ENABLE()
#define Can_Host_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOA_CLK_ENABLE()
     
#define Can_Host_FORCE_RESET()              __HAL_RCC_CAN1_FORCE_RESET()
#define Can_Host_RELEASE_RESET()            __HAL_RCC_CAN1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define Can_Host_TX_PIN                    GPIO_PIN_12
#define Can_Host_TX_GPIO_PORT              GPIOA  
#define Can_Host_TX_AF                     GPIO_AF9_CAN1
#define Can_Host_RX_PIN                    GPIO_PIN_11
#define Can_Host_RX_GPIO_PORT              GPIOA
#define Can_Host_RX_AF                     GPIO_AF9_CAN1

/* Definition for CAN's NVIC */
#define Can_Host_RX_IRQn                   CAN1_RX0_IRQn
#define Can_Host_RX_IRQHandler             CAN1_RX0_IRQHandler

/* Exported macro ------------------------------------------------------------*/
/*波特率设置*/
typedef struct
{
    unsigned short int BPS;
    unsigned int SJW;
    unsigned int BS1;
    unsigned int BS2;
    unsigned short int Prescaler;
} CAN_BAUD_Struct;

/* Exported functions ------------------------------------------------------- */
uint8_t Can_Host_Config( uint32_t baud );
uint8_t Can_Host_send_msg ( uint32_t id, uint8_t *msg, uint8_t len );
uint32_t Can_Host_receive_msg ( uint8_t *buf );
void CanBusOff_Reset ( void );

#endif /* _CAN_HOST_H_ */







