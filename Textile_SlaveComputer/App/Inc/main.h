/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/

#include "Driver.h"
#include "TimerBase.h"
#include "pullrod_Ctrl.h"
#include "DataBaseProcess.h"
/* Exported types ------------------------------------------------------------*/
typedef enum {
    LED_Ready = 1,
    LED_LonOffSta,
    LED_On_Sta,
    LED_Cnt1Sta,
} LEDFaultHandle;

typedef enum {
    FAST   = 1,
    Normal = 2,
} BlinkFreq;


#define Standby_Delay       3000
#define AtuoMode            1
#define ManualMode          0
void ModeChange(uint8_t Mode);
extern uint16_t CANSendFlag ;
extern uint8_t  u8FaultRetFlag, u8StandbyFlag;
extern uint8_t   HostShutDowncmd;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
