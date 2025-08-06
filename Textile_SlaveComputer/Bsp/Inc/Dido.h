/**
* 版权所有(C)
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
* <日期> | <版本> | <作者> | <描述>
*
* xxxx/xx/xx | 1.0.0 | HWW | 创建文件
*
*/
//------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
#ifndef _Dido__H_
#define _Dido__H_
//-------------------- include files ----------------------------------------
#include "Driver.h"
#include "ErrorCode.h"
//PA15 0
//PC10 1
//PC11 2 版本号，下拉
//-------------------- public definitions -----------------------------------
#define DI_VID0_GPIO_PORT                     GPIOA     //textile
#define DI_VID1_GPIO_PORT                     GPIOC     //textile
#define DI_VID2_GPIO_PORT                     GPIOC     //textile
#define DI_PC_5V_GPIO_PORT                    GPIOB     //textile
#define DI_PC_MODE_GPIO_PORT                  GPIOG
#define DI_TouchEdge_GPIO_PORT                GPIOG
//#define DI_HAND_MODE_GPIO_PORT                GPIOD
#define DI_KEY1_GPIO_PORT                     GPIOG
#define DI_KEY2_GPIO_PORT                     GPIOG
#define DI_KEY3_GPIO_PORT                     GPIOD
#define DI_POWERSWSTA_GPIO_PORT                  GPIOD     //textile
#define DI_2_STOP_GPIO_PORT                   GPIOF
#define DI_0_STOP_GPIO_PORT                   GPIOG     //textile
//#define DI_LEFT_GPIO_PORT                     GPIOE
//#define DI_RIGHT_GPIO_PORT                    GPIOE
//#define DI_DOWN_GPIO_PORT                     GPIOE
#define DI_RST_MCU_GPIO_PORT                  GPIOD     //textile
#define DI_SYNC_RTK_GPIO_PORT                 GPIOB


#define DI_VID0_GPIO_PIN                      GPIO_PIN_15//textile
#define DI_VID1_GPIO_PIN                      GPIO_PIN_10//textile
#define DI_VID2_GPIO_PIN                      GPIO_PIN_11//textile
#define DI_PC_5V_GPIO_PIN                     GPIO_PIN_10   //textile
#define DI_PC_MODE_GPIO_PIN                   GPIO_PIN_8
#define DI_TouchEdge_GPIO_PIN                   GPIO_PIN_7
//#define DI_HAND_MODE_GPIO_PIN                 GPIO_PIN_4
#define DI_KEY1_GPIO_PIN                      GPIO_PIN_5
#define DI_KEY2_GPIO_PIN                      GPIO_PIN_3
#define DI_KEY3_GPIO_PIN                      GPIO_PIN_10
#define DI_POWERSWSTA_GPIO_PIN                   GPIO_PIN_7    //textile
#define DI_2_STOP_GPIO_PIN                    GPIO_PIN_6
#define DI_0_STOP_GPIO_PIN                    GPIO_PIN_9    //textile
//#define DI_LEFT_GPIO_PIN                      GPIO_PIN_2
//#define DI_RIGHT_GPIO_PIN                     GPIO_PIN_3
//#define DI_DOWN_GPIO_PIN                      GPIO_PIN_4
#define DI_RST_MCU_GPIO_PIN                   GPIO_PIN_5    //textile
#define DI_SYNC_RTK_GPIO_PIN                  GPIO_PIN_6


#define DI_GPIO_PORT_LIST \
{                         \
    DI_VID0_GPIO_PORT,\
    DI_VID1_GPIO_PORT,\
    DI_VID2_GPIO_PORT,\
    DI_PC_5V_GPIO_PORT,\
    DI_PC_MODE_GPIO_PORT,\
    DI_TouchEdge_GPIO_PORT,\
    DI_KEY1_GPIO_PORT,\
    DI_KEY2_GPIO_PORT,\
    DI_KEY3_GPIO_PORT,\
    DI_POWERSWSTA_GPIO_PORT,\
    DI_2_STOP_GPIO_PORT,\
    DI_0_STOP_GPIO_PORT,\
    DI_RST_MCU_GPIO_PORT,\
    DI_SYNC_RTK_GPIO_PORT,\
}

#define DI_GPIO_PIN_LIST \
{                        \
    DI_VID0_GPIO_PIN,\
    DI_VID1_GPIO_PIN,\
    DI_VID2_GPIO_PIN,\
    DI_PC_5V_GPIO_PIN,\
    DI_PC_MODE_GPIO_PIN,\
    DI_TouchEdge_GPIO_PIN,\
    DI_KEY1_GPIO_PIN,\
    DI_KEY2_GPIO_PIN,\
    DI_KEY3_GPIO_PIN,\
    DI_POWERSWSTA_GPIO_PIN,\
    DI_2_STOP_GPIO_PIN,\
    DI_0_STOP_GPIO_PIN,\
    DI_RST_MCU_GPIO_PIN,\
    DI_SYNC_RTK_GPIO_PIN,\
}

typedef enum
{
    DI_CH_VID0,//下拉分界
    DI_CH_VID1,
    DI_CH_VID2,
    DI_CH_PC_5V,
    DI_CH_PC_MODE,
    DI_CH_TouchEdge,
    DI_CH_KEY1,
    DI_CH_KEY2,
    DI_CH_KEY3,
    DI_CH_POWERSWSTA,
    DI_CH_2_STOP,
    DI_CH_0_STOP,
    DI_CH_RST_MCU,
    DI_CH_SYNC_RTK,
    DI_CH_NUM
} E_DI_CH, *PE_DI_CH;


#define DO_LED_GPIO_PORT               GPIOB        //textile
#define DO_WL_RST_GPIO_PORT            GPIOD        //textile
#define DO_MAGNET_ON_GPIO_PORT         GPIOD        //textile
#define DO_Power1_GPIO_PORT            GPIOD
//#define DO_Power2_GPIO_PORT            GPIOD
#define DO_KEY1_LED_GPIO_PORT          GPIOG
#define DO_KEY2_LED_GPIO_PORT          GPIOG
#define DO_LED_RIGHT_GPIO_PORT         GPIOG        //textile
#define DO_LED_LEFT_GPIO_PORT          GPIOG        //textile
#define DO_STOP0_GPIO_PORT             GPIOB        //textile
#define DO_STOP2_GPIO_PORT             GPIOC
#define DO_LED_R_GPIO_PORT             GPIOB        //textile
#define DO_LED_G_GPIO_PORT             GPIOG        //textile
#define DO_LED_Y_GPIO_PORT             GPIOG        //textile
#define DO_POWER_SW3_GPIO_PORT         GPIOF        //textile
#define DO_POWER_SW4_GPIO_PORT         GPIOF        //textile
#define DO_BEEP_SE_GPIO_PORT           GPIOG        //textile
#define DO_POWER_SW2_GPIO_PORT         GPIOF        //textile
#define DO_POWER_SW1_GPIO_PORT         GPIOF        //textile
#define DO_SYNC_LIVOX_GPIO_PORT        GPIOE        //textile
#define DO_SYNC_IMU_GPIO_PORT          GPIOE        //textile
#define DO_SYNC_RGB_GPIO_PORT          GPIOE        //textile
#define DO_SYNC_RGBD_GPIO_PORT         GPIOE        //textile
//#define DO_PC_24V_GPIO_PORT            GPIOA


#define DO_LED_GPIO_PIN                 GPIO_PIN_14    //textile
#define DO_WL_RST_GPIO_PIN              GPIO_PIN_3     //textile
#define DO_MAGNET_ON_GPIO__PIN          GPIO_PIN_4     //textile
#define DO_Power1_GPIO_PIN              GPIO_PIN_6
//#define DO_Power2_GPIO_PIN              GPIO_PIN_7
#define DO_KEY1_LED_GPIO_PIN            GPIO_PIN_6
#define DO_KEY2_LED_GPIO_PIN            GPIO_PIN_4
#define DO_LED_RIGHT_GPIO_PIN           GPIO_PIN_10    //textile
#define DO_LED_LEFT_GPIO_PIN            GPIO_PIN_11    //textile
#define DO_STOP0_GPIO_PIN               GPIO_PIN_3     //textile
#define DO_STOP2_GPIO_PIN               GPIO_PIN_13
#define DO_LED_R_GPIO_PIN               GPIO_PIN_4     //textile
#define DO_LED_G_GPIO_PIN               GPIO_PIN_14    //textile
#define DO_LED_Y_GPIO_PIN               GPIO_PIN_13    //textile
#define DO_POWER_SW3_GPIO_PIN           GPIO_PIN_2     //Motor2
#define DO_POWER_SW4_GPIO_PIN           GPIO_PIN_1     //PC
#define DO_BEEP_SE_GPIO_PIN             GPIO_PIN_12    //textile
#define DO_POWER_SW2_GPIO_PIN           GPIO_PIN_4     //Motor1
#define DO_POWER_SW1_GPIO_PIN           GPIO_PIN_5     //sensor
#define DO_SYNC_LIVOX_GPIO_PIN          GPIO_PIN_3     //textile
#define DO_SYNC_IMU_GPIO_PIN            GPIO_PIN_4     //textile
#define DO_SYNC_RGB_GPIO_PIN            GPIO_PIN_5     //textile
#define DO_SYNC_RGBD_GPIO_PIN           GPIO_PIN_2     //textile
//#define DO_PC_24V_GPIO_PIN              GPIO_PIN_5
typedef enum
{
    DO_CH_LED,
    DO_CH_WL_RST,
    DO_CH_MAGNET_ON,
    DO_CH_Power1,
    DO_CH_KEY1_LED,
    DO_CH_KEY2_LED,
    DO_CH_LED_RIGHT,
    DO_CH_LED_LEFT,
    DO_CH_STOP0,
    DO_CH_STOP2,
    DO_CH_LED_R,/*10*/
    DO_CH_LED_G,
    DO_CH_LED_Y,
    DO_CH_POWER_SW3,
    DO_CH_POWER_SW4,
    DO_CH_BEEP_SE,
    DO_CH_POWER_SW2,
    DO_CH_POWER_SW1,
    DO_CH_SYNC_LIVOX,
    DO_CH_SYNC_IMU,
    DO_CH_SYNC_RGB,/*20*/
    DO_CH_SYNC_RGBD,
    DO_CH_NUM
} E_DO_CH, *PE_DO_CH;

#define DO_GPIO_PORT_LIST \
{                         \
    DO_LED_GPIO_PORT,\
    DO_WL_RST_GPIO_PORT,\
    DO_MAGNET_ON_GPIO_PORT,\
    DO_Power1_GPIO_PORT,\
    DO_KEY1_LED_GPIO_PORT,\
    DO_KEY2_LED_GPIO_PORT,\
    DO_LED_RIGHT_GPIO_PORT,\
    DO_LED_LEFT_GPIO_PORT,\
    DO_STOP0_GPIO_PORT,\
    DO_STOP2_GPIO_PORT,\
    DO_LED_R_GPIO_PORT,\
    DO_LED_G_GPIO_PORT,\
    DO_LED_Y_GPIO_PORT,\
    DO_POWER_SW3_GPIO_PORT,\
    DO_POWER_SW4_GPIO_PORT,\
    DO_BEEP_SE_GPIO_PORT,\
    DO_POWER_SW2_GPIO_PORT,\
    DO_POWER_SW1_GPIO_PORT,\
    DO_SYNC_LIVOX_GPIO_PORT,\
    DO_SYNC_IMU_GPIO_PORT,\
    DO_SYNC_RGB_GPIO_PORT,\
    DO_SYNC_RGBD_GPIO_PORT,\
}

#define DO_GPIO_PIN_LIST \
{                        \
    DO_LED_GPIO_PIN,\
    DO_WL_RST_GPIO_PIN,\
    DO_MAGNET_ON_GPIO__PIN,\
    DO_Power1_GPIO_PIN,\
    DO_KEY1_LED_GPIO_PIN,\
    DO_KEY2_LED_GPIO_PIN,\
    DO_LED_RIGHT_GPIO_PIN,\
    DO_LED_LEFT_GPIO_PIN,\
    DO_STOP0_GPIO_PIN,\
    DO_STOP2_GPIO_PIN,\
    DO_LED_R_GPIO_PIN,\
    DO_LED_G_GPIO_PIN,\
    DO_LED_Y_GPIO_PIN,\
    DO_POWER_SW3_GPIO_PIN,\
    DO_POWER_SW4_GPIO_PIN,\
    DO_BEEP_SE_GPIO_PIN,\
    DO_POWER_SW2_GPIO_PIN,\
    DO_POWER_SW1_GPIO_PIN,\
    DO_SYNC_LIVOX_GPIO_PIN,\
    DO_SYNC_IMU_GPIO_PIN,\
    DO_SYNC_RGB_GPIO_PIN,\
    DO_SYNC_RGBD_GPIO_PIN,\
}

//-------------------- public data ------------------------------------------
//-------------------- public functions -------------------------------------
void Dido_Init ( void );
void DidoLED_Init ( void );
uint8_t DI_ReadAll ( uint16_t* pu16Value );
uint8_t DI_ReadByIndex ( GPIO_PinState* eStatus, E_DI_CH eIndex );
uint8_t DO_WriteAll ( uint16_t u16Value );
uint8_t DO_ReadAll ( uint16_t* pu16Value );
uint8_t DO_WriteByIndex ( GPIO_PinState eStatus, E_DO_CH eIndex );
uint8_t DO_ToggleByIndex ( E_DO_CH eIndex );

//-------------------- inline functions -------------------------------------

#endif /* DIDO_H_ */

//-----------------------End of file------------------------------------------
/** @}*/
