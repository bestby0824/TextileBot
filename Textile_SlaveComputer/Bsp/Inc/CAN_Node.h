/**
* 版权所有(C)
*
* ********
*
* @file
* @brief
* @details
* @author   HWW
* @version  1.0.0
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _CAN_NODE_H_
#define _CAN_NODE_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor Can_Node instance used and associated
   resources */
/* Definition for Can_Node clock resources */
#define Can_Node                            CAN2
#define Can_Node_CLK_ENABLE()               __HAL_RCC_CAN2_CLK_ENABLE()
#define Can_Node_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOB_CLK_ENABLE()
     
#define Can_Node_FORCE_RESET()              __HAL_RCC_CAN2_FORCE_RESET()
#define Can_Node_RELEASE_RESET()            __HAL_RCC_CAN2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define Can_Node_TX_PIN                    GPIO_PIN_13
#define Can_Node_TX_GPIO_PORT              GPIOB
#define Can_Node_TX_AF                     GPIO_AF9_CAN2
#define Can_Node_RX_PIN                    GPIO_PIN_12
#define Can_Node_RX_GPIO_PORT              GPIOB
#define Can_Node_RX_AF                     GPIO_AF9_CAN2

/* Definition for CAN's NVIC */
#define Can_Node_RX_IRQn                   CAN2_RX1_IRQn
#define Can_Node_RX_IRQHandler             CAN2_RX1_IRQHandler

/* Exported macro ------------------------------------------------------------*/
extern CAN_HandleTypeDef     Can_NodeHandle;
/* Exported functions ------------------------------------------------------- */
uint8_t Can_Node_Config( uint32_t baud );
uint8_t Can_Node_send_msg ( uint32_t id, uint8_t *msg, uint8_t len );
uint32_t Can_Node_receive_msg ( uint8_t *buf );

#endif /* _CAN_NODE_H_ */







