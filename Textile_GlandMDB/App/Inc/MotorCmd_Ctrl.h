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
* xxxx/xx/xx | 1.0.0 | MUNIU | �����ļ�
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
    Idle_MODE = 0,  //��ʼģʽ
    RC_MODE,        //ң��ģʽ
    PC_MODE,        //�Զ�ģʽ
    HAND_MODE,      //�ֶ�ģʽ
} E_CTRL_MODE;

typedef enum 
{
    CmdMode_Idle = 0,     // 0:��ʼ״̬
    CmdMode_PosCtrl,      // 1:λ�ÿ���״̬
    CmdMode_SpdCtrl,      // 2:�ٶȿ���ģʽ
    CmdMode_TorqueCtrl,   // 3:ת�ؿ���ģʽ
    CmdMode_FreeStop,     // 4:λ�ÿ���״̬
    CmdMode_Fault,        // 5:�й���
    CmdMode_RC,
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
    CmdStop_PosLimit,
    CmdRes4,
    CmdStop_Torque,
    CmdRes5,
    CmdStop_NoneCali,
    CmdRes6,
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

extern uint16_t u16StopFlag;
//-------------------- public functions -------------------------------------
void MotorCmd_Ctrl ( RegDef_PosCtrlCmd *PosCtrlCmd, S_MOTOR_CMD *sMotorCmd );
#endif // _MotorState_Ctrl__H_

//-----------------------End of file------------------------------------------
/** @}*/
