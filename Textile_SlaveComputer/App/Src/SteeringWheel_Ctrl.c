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

//-------------------- include files ----------------------------------------
#include "SteeringWheel_Ctrl.h"
#include "StateCtrl.h"
#include "Host_Com.h"
#include "ErrorCode.h"
#include "Uart_Dbug.h"
#include "main.h"
#include "VCUProcess.h"
#include "NodeProcess.h"

//-------------------- local definitions ------------------------------------
extern ActStates m_eActState;

//-------------------- private data -----------------------------------------
static uint32_t Wheel_CtrlCnt = 0;
_iq _iqSteerPosRef = _IQ ( 0.5 );
static SteeringWheel_StepS u16SteeringWheel_Step = FreeStop;
//-------------------- private functions declare ----------------------------

//-------------------- public data ------------------------------------------
SteeringWheel_StepS g_u16SteeringWheel_Step = FreeStop;
E_CMD_MODE eCmdMode = CmdMode_Idle;
//-------------------- public functions -------------------------------------
_iq g_qSteerTargetAngle = 0;            //目标位置                      方向轮位置标幺值
_iq g_qSteerMaxSpeed = _IQ ( 1.2 );     //最大转动加速度                轮子速度标幺值/0.001
_iq g_qSteerGasAccKP = 1;               //方向角kp
_iq g_qSteerThreshold = 10;             //速度到达允许误差              轮子速度标幺值
uint16_t RUNSta, u16PosRef, G_StreePos;

int8_t SteeringWheel_Init ( void )
{
    return Err_None;
}
#if RS485_Ctrl
void SteeringWheel_Ctrl ( void )  //1KHz
{
    Wheel_CtrlCnt++;
    if ( Warning_Report.bit.BLDC_Power24V_Low == 0 )           //有动力电，给转向电机板发运动指令
    {
        u16SteeringWheel_Step = PosCtrlByCmd;
        if ( 0 == HostCom_ReadSteerCommand ( &u16PosRef ) )
        {
            _iqSteerPosRef = _IQsat ( _IQdiv ( u16PosRef, 32767 ), _IQ ( 0.67 ), _IQ ( 0.33 ) ); //转向限制角度-61°-+61°
        }
    } else if ( RunSta_Reset2Idle == RUNSta ) {
        u16SteeringWheel_Step = g_u16SteeringWheel_Step;
    } else {
        u16SteeringWheel_Step = FreeStop;
    }

    if ( ( m_eCtrlMode == HAND_MODE ) || ( VCU_ParamHandle.u8EmergencyState == 0 ) || ( u8StandbyFlag ) )
    {
        u16SteeringWheel_Step = FreeStop;
    }
    if ( ( Wheel_CtrlCnt % 2 ) == 0 )               //2mS1次
    {
        u16SteeringWheel_Step = GetAbsPos;
    }
    if ( ( Wheel_CtrlCnt % 10 ) == 0 )               //10mS1次
    {
        u16SteeringWheel_Step = GetPressure;
    }
    if ( ( Wheel_CtrlCnt % 1000 ) == 4 )            //1秒一次
    {
        u16SteeringWheel_Step = DogFeed;
    } else if ( ( Wheel_CtrlCnt % 1000 ) == 204 )   //1秒一次
    {
        u16SteeringWheel_Step = GetSta;
    } else if ( ( Wheel_CtrlCnt % 1000 ) == 504 )   //1S一次
    {
        u16SteeringWheel_Step = GetSWVer;
    }
    if ( u8FaultRetFlag == 1 ) //消除故障优先级最高，非定时发送
    {
        u16SteeringWheel_Step = FaultAlarmClear;
    }
    switch ( u16SteeringWheel_Step )
    {
    case PosCtrlByCmd:
    {
        SteeringWheel_SendPos ( _iqSteerPosRef );
    }
    break;
    case PosMiddle:
    {
        SteeringWheel_SendPos ( _IQ ( 0.5 ) );
    }
    break;
    case FreeStop:
    {
        SteeringWheel_SetStopFree();
    }
    break;
    case DogFeed:
    {
        SteeringWheel_DogFeed();
    }
    break;
    case GetSta:
    {
        SteeringWheel_GetSta();
    }
    break;
    case GetAbsPos:
    {
        SteeringWheel_GetAbsPos();
    }
    break;
    case GetSWVer:
    {
        SteeringWheel_GetVersion ( );
    }
    break;
    case GetPressure:
    {
        SteeringWheel_GetPressure ( );
    }
    break;
    case FaultAlarmClear:
    {
        SteeringWheel_FaultAlarmClear ( );
    }
    break;
    default:
        break;
    }
    RUNSta = GetRUNSta();    //检测是否挂上档

//    if ( ( _iqSteerPosRef < _IQ ( 0.48 ) ) && ( ( RUNSta == RunSta_Run_D ) || ( RUNSta == RunSta_Run_R ) ) )
//    {
//        DoSetSteeringLampSW ( Sta_L ); //左转向灯亮
//    }
//    else if ( ( _iqSteerPosRef > _IQ ( 0.52 ) ) && ( ( RUNSta == RunSta_Run_D ) || ( RUNSta == RunSta_Run_R ) ) )
//    {
//        DoSetSteeringLampSW ( Sta_R ); //右转向灯亮
//    }
//    else
//    {
//        DoSetSteeringLampSW ( Sta_N ); //转向灯灭
//    }
}
void SteeringWheel_SetPos ( _iq PosRef )
{
    _iqSteerPosRef = PosRef;
}
#elif CAN_Ctrl
/**
  * @brief  转向板转向板寄存器填充函数
  * @param  none
  * @retval None
  */
void SteeringWheel_Ctrl ( void )  //1KHz
{
    static E_CMD_MODE eCmdLastMode = CmdMode_Idle;
    static uint32_t u32PowerOnCnt = 0;

//    if ( Fault_Handle1.Fault_Steer.all != 0 )
//    {
//        eCmdMode = CmdMode_Fault;
//    }
//    else if ( ( Warning_Report.bit.BLDC_Power24V_Low ) || ( FaultCtrl.all ) || ( m_eCtrlMode == HAND_MODE ) )
//    {
//        eCmdMode = CmdMode_FreeStop;
//    }
//    else if ( ( m_eCtrlMode == RC_MODE ) || ( m_eCtrlMode == PC_MODE ) ) //遥控模式||自动模式
//    {
////        if ( u16PosCheckPass_Flag == 1 )  //上电自检完成之后，进行位置控制
//        {
//            eCmdMode = CmdMode_PosCtrl;
//        }
////        else
////        {
////            if ( u32PowerOnCnt++ > 10000 )
////            {
////            eCmdMode = CmdMode_PosMiddle;
////            }
////        }
//    }
//    else
//    {
//        ;
//    }

    switch ( eCmdMode )
    {
    case CmdMode_Idle:
    {
        RegReal_SteerPosCtrlCmd.u16Mode = eCmdMode;
        RegReal_SteerPosCtrlCmd.u16EN = DISABLE;
    }
    break;
    case CmdMode_PosCtrl://位置控制状态
    {
        RegReal_SteerPosCtrlCmd.u16Mode = eCmdMode;
        RegReal_SteerPosCtrlCmd.u16EN = ENABLE;
//        HostCom_ReadSteerCommand ( &RegReal_SteerPosCtrlCmd.u16PosRef );//获取位置参考值
    }
    break;
    case CmdMode_FreeStop://空闲状态
    {
        RegReal_SteerPosCtrlCmd.u16Mode = eCmdMode;
        RegReal_SteerPosCtrlCmd.u16EN = DISABLE;
    }
    break;
    case CmdMode_Fault://有故障
    {
        RegReal_SteerPosCtrlCmd.u16Mode = eCmdMode;
        RegReal_SteerPosCtrlCmd.u16EN = DISABLE;
    }
    break;
    default:
        break;
    }

    eCmdLastMode = eCmdMode;
}
#endif
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


