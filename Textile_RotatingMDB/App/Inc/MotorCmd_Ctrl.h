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
* xxxx/xx/xx | 1.0.0 | MUNIU | 创建文件
*/
//------------------------------------------------------------------------------
//-----------------------------------------------------------------------------
#ifndef _MotorState_Ctrl__H_
#define _MotorState_Ctrl__H_
//-------------------- include files ----------------------------------------
#include "stm32f4xx_hal.h"
#include "HostProcess.h"

//-------------------- public definitions -----------------------------------
typedef enum 
{
    Idle_MODE = 0,  //初始模式
    RC_MODE,        //遥控模式
    PC_MODE,        //自动模式
    HAND_MODE,      //手动模式
} E_CTRL_MODE;

typedef enum 
{
    CmdMode_Idle = 0,     // 0:初始状态
    CmdMode_PosCtrl,      // 1:位置控制状态
    CmdMode_SpdCtrl,      // 2:速度控制模式
    CmdMode_TorqueCtrl,   // 3:转矩控制模式
    CmdMode_FreeStop,     // 4:位置空闲状态
    CmdMode_Fault,        // 5:有故障
    CmdMode_RC,           // 6:遥控控制
} E_CMD_MODE;

typedef enum
{
    EndOfCtrl = 0,
    CmdRuning,
    CmdTimeOut,
    CmdRes1,
    CmdStop_Invalid,
    CmdRes2,
    CmdStop_Disable,
    CmdRes3,
    CmdRes4,
    CmdRes5,
    CmdRes6,
    CmdRes7,
    CmdRes8,
    CmdRes9,
    CmdFault = 14,
} E_RUN_STATE;

typedef struct
{
    E_CTRL_MODE eCtrlMode;
    E_CMD_MODE eCmdMode;
    E_RUN_STATE eRunState;
    _iq iqTargetPos;
    _iq iqTargetSpd;
    _iq iqTargetTorque;
    _iq iqRuningTime;
} S_MOTOR_CMD;
extern S_MOTOR_CMD g_sMotorCmd;

typedef enum {
    CtrlDir_Foward = 0,
    CtrlDir_Reverse,
    CtrlDir_Stop,
} E_CTRLDIR_MODE;
extern uint16_t u16RunningDir;

extern uint16_t u16StopFlag;
//-------------------- public functions -------------------------------------
void MotorCmd_Ctrl ( RegDef_PosCtrlCmd *PosCtrlCmd, S_MOTOR_CMD *sMotorCmd );
#endif // _MotorState_Ctrl__H_

//-----------------------End of file------------------------------------------
/** @}*/
