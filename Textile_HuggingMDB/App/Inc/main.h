/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
#define Standby_Delay 2000  //2√Î

/* Exported constants --------------------------------------------------------*/
typedef enum {
    LED_Ready = 1,
    LED_LonOffSta,
    LED_On_Sta,
    LED_Cnt1Sta,
} LEDFaultHandle;
extern uint8_t u8Standby_Flag;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
