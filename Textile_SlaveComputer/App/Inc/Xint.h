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
#ifndef _XINT__H_
#define _XINT__H_
//-------------------- include files ----------------------------------------
#include "Driver.h"
#include "pullrod_Ctrl.h"
#include "Spd_Ctrl.h"
#include "SteeringWheel_Ctrl.h"
#include "IMU_Com.h"
#include "EncoderProcess.h"
#include "Dido.h"
#include "Adc.h"
#include "Debug_Com.h"
#include "StFlash.h"
#include "Wireless_Ctrl.h"
#include "Uart_Sprintf.h"

//-------------------- public definitions -----------------------------------
/*************************************系统配置*************************************/
#define TIM_BASE_FREQ     10000           //基时定时器频率
#define TIM_FAST_FREQ     50000           //基时定时器频率

#define PRINTF_UART_BUAD  230400//256000         //调试打印串口波特率

//系统各中断优先级(数字越小,优先级越高)
#define IRQ_Priority_SPI2        0
#define IRQ_Priority_TIM_FAST    0
#define IRQ_Priority_TIM_BASE    1
#define IRQ_Priority_ADC_Irq     2
#define IRQ_Priority_ADC_DMA     3
#define IRQ_Priority_CAN         4
#define IRQ_Priority_UART        5
typedef struct {
    uint32_t InTick;
    uint32_t OutTick;
    uint16_t Task_Rate[10];
}CPURateHandle;    //放在xint.h中

//-------------------- public data ------------------------------------------
extern uint8_t g_u8Tick1kHz;
extern uint16_t LiftCmdCnt,RotateCmdCnt,BowRiseCmdCnt,ClawCmdCnt,SideShiftCmdCnt,LiftRaderCmdCnt, SlowSpdCmdCnt, BrakeCanelCmdCnt, BrakeCmdCnt;
extern uint16_t PosModeFlag;
extern uint32_t System_Cnt;
extern uint16_t RadarSwitch;
extern uint8_t u8RCStop_2;
extern uint16_t StreeCnt;
extern uint16_t  PosModeFlag,SlowModeFlag;
extern uint16_t   SlowDiatance,SlowDir;
extern uint16_t BreakPos,BreakFlag;
extern uint8_t  BreakCanelSwitch ;
extern uint8_t u8PowerSwFlag;
extern int16_t Diatance;
extern uint16_t PowerOnFlag;
//-------------------- public functions -------------------------------------

void TimerBase_Update_Callback(void);
void DelaySet_mS(uint32_t delay);
uint8_t Delay_CheckTimeOut(void);
#endif // _XINT_H_

//-----------------------End of file------------------------------------------
/** @}*/
