/**
* ��Ȩ����(C) 
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
* <����> | <�汾> | <����> | <����>
*
* xxxx/xx/xx | 1.0.0 | HWW | �����ļ�
*
*/
//------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
#ifndef _SteerWheel_Ctrl__H_
#define _SteerWheel_Ctrl__H_
//-------------------- include files ----------------------------------------
#include "SteeringWheelProcess.h"
//-------------------- public definitions -----------------------------------
#define RS485_Ctrl               0
#define CAN_Ctrl                 1

typedef enum{
    PosCtrlByCmd = 0,
    FreeStop,
    PosMiddle,
    DogFeed,
    GetSta,
    GetAbsPos,
    GetSWVer,
    GetPressure,
    FaultAlarmClear, 
}SteeringWheel_StepS;

typedef enum 
{
    CmdMode_Idle = 0,     // 0:��ʼ״̬
    CmdMode_PosCtrl,      // 1:λ�ÿ���״̬
    CmdMode_SpdCtrl,      // 2:�ٶȿ���״̬
    CmdMode_TorqueCtrl,   // 3:ת�ؿ���״̬
    CmdMode_FreeStop,     // 4:����״̬
    CmdMode_Fault,        // 5:�й���
} E_CMD_MODE;
extern E_CMD_MODE eCmdMode;
//-------------------- public data ------------------------------------------
extern SteeringWheel_StepS g_u16SteeringWheel_Step;
extern _iq _iqSteerPosRef;
//-------------------- public functions -------------------------------------

int8_t SteeringWheel_Init(void);
void SteeringWheel_Ctrl(void);
void SteeringWheel_SetPos ( _iq PosRef );
#endif // _SteerWheel_Ctrl__H_

//-----------------------End of file------------------------------------------
/** @}*/
