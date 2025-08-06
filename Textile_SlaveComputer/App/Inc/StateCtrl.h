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
#ifndef _StateCtrl__H_
#define _StateCtrl__H_
//-------------------- include files ----------------------------------------
#include "Driver.h"
#include "DidoProcess.h"
#include "SteeringWheel_Ctrl.h"
#include "Monitor.h"
//-------------------- public definitions -----------------------------------
typedef enum 
 {
    RunSta_Reset2Idle = 0,// 初始状态
    RunSta_Idle,  // 1:空闲状态，初始化完成正常
    RunSta_Run_D, // 2: 车挂D档
    RunSta_Run_R, // 3: 车挂R档
    RunSta_Run_N, // 4:车挂空挡
    RunSta_Fault, // 有故障
    RunSta_Run_P, // 6:P档

} RunStates;
typedef enum {
    Act_Brake = 0, // 1:正在刹车
    Act_Shift_N,   // 1:空档
    Act_Lift,      // 1:升降推杆回中
    Act_Claw,      // 1:推杆回中
    Act_BowRise,   // 1:推杆回中
    Act_Rotate,    // 1:推杆回中
    Act_Shift_N_Bump_ON, // 空挡
    Act_Shift_D,   // 挂D档，油门0
    Act_Steering,  // 转向，轮子回正
    Act_Finish,    // 初始化完成，无故障
} ActStates;//动作状态，记录当前正在执行的动作
extern uint16_t RunState;//当前上位机给定的档位
extern RunStates m_eRunState;
#define Lim_ON    ((LimStaNow == LimH)||(LimStaNow == LimM)||(LimStaNow == LimL))

#define NextStateCheck()  {                                             \
        s8Ret = HostCom_ReadDrvModeCommand(&s16SetRunSta);              \
        if((Err_None == s8Ret)&&(s16SetRunSta < RunSta_Fault))          \
        {                                                               \
            m_eRunState = s16SetRunSta;                                 \
            RunState = s16SetRunSta;                                    \
            if(RunSta_Reset2Idle == m_eRunState){StateResetSystem();}   \
        }                                                               \
        if(0 != FaultCtrl.all)                            \
        {                                                               \
            m_eRunState = RunSta_Fault;                                 \
        }                                                               \
    }
//-------------------- public data ------------------------------------------
//-------------------- public functions -------------------------------------
void StateResetSystem();
void RunStateInit ( void );
void RunStateCtrl ( void );
RunStates GetRUNSta ( void );
ActStates GetPreSta ( void );   
void Delay_mS ( uint32_t Delay );
#endif // _XXX_H_

//-----------------------End of file------------------------------------------
/** @}*/
