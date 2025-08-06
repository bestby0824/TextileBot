/**
* 版权所有(C)
*
* ********
*
* @file
* @brief
* @details
* @author MUNIU
* @version 1.0.0
* @date xxxx-xx-xx
* @par Description:
* @par Change History:
*
* <日期> | <版本> | <作者> | <描述>
*
* xxxx/xx/xx | 1.0.0 | MUNIU | 创建文件
*
*/
//------------------------------------------------------------------------------

//-------------------- include files ----------------------------------------
#include "MotorCmd_Ctrl.h"
#include "HostProcess.h"
#include "MotorInfo.h"
#include "Monitor.h"
#include "string.h"
#include "DataBaseProcess.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------

//-------------------- private functions declare ----------------------------

//-------------------- public data ------------------------------------------
S_MOTOR_CMD g_sMotorCmd = { 0, 0, 0, 0, 0, 0, 0 };
uint16_t u16StopFlag = 0;
uint16_t u16RunningDir = 0;

//-------------------- public functions -------------------------------------
/**
* @brief        转向驱动板状态控制
* @param       输入：eCtrlMode:叉车运行模式（来源:下位机）；
                     SteerPosCtrlCmd：转向控制指令（来源:下位机）
               输出：sMotorCmd:过滤之后得到给电机控制的CMD
 * @retval      无
 */
void MotorCmd_Ctrl ( RegDef_PosCtrlCmd *PosCtrlCmd, S_MOTOR_CMD *sMotorCmd )
{
    static E_CMD_MODE eLastCmdMode = CmdMode_Idle;
    static uint16_t u16SteerCmdLastSN = 0;
    static RegDef_PosCtrlCmd Rx_SteerCtrlCmd = { 0, 0, 0, _IQ ( 0.5 ) };
    static uint32_t u32RCTime[2];

    __disable_irq();         //关闭总中断
    memcpy ( ( uint8_t * ) &Rx_SteerCtrlCmd, ( uint8_t * ) PosCtrlCmd, 8 );
    __enable_irq();          // 开启总中断

    if ( g_u32FaultNum.all != NoFault ) sMotorCmd->eCmdMode = CmdMode_Fault;  //有故障，置故障状态
    else
    {
        if ( ( sMotorCmd->eCmdMode != CmdMode_Idle ) && \
            ( ( u16StopFlag ) || ( Rx_SteerCtrlCmd.u16EN == 0 ) ) ) 
        {
            sMotorCmd->eCmdMode = CmdMode_FreeStop;
        }
    }

    switch ( sMotorCmd->eCmdMode )
    {
        case CmdMode_Idle:
        {
            sMotorCmd->eCmdMode = Rx_SteerCtrlCmd.u16Mode;
            eLastCmdMode = CmdMode_Idle;
        }
        break;

        case CmdMode_PosCtrl:
        {
            //刷新目标位置
            if ( ( u16SteerCmdLastSN != Rx_SteerCtrlCmd.u16SN ) || ( eLastCmdMode != CmdMode_PosCtrl ) )
            {
                if ( ( Rx_SteerCtrlCmd.u16Mode == CmdMode_Idle ) || ( Rx_SteerCtrlCmd.u16Mode > CmdMode_RC ) || \
                    ( ( Rx_SteerCtrlCmd.u16Mode >= CmdMode_SpdCtrl ) && ( Rx_SteerCtrlCmd.u16Mode < CmdMode_TorqueCtrl ) ) )
                {
                    sMotorCmd->eCmdMode = CmdMode_FreeStop;
                }
                else
                {
                    sMotorCmd->eCmdMode = Rx_SteerCtrlCmd.u16Mode;
                    
                    if ( sMotorCmd->eCmdMode == CmdMode_PosCtrl )
                    {
                        sMotorCmd->iqTargetPos = _IQsat ( Rx_SteerCtrlCmd.u16Ref, ( ParamHandle_Steer.Reg_PosMax - 364 ), ParamHandle_Steer.Reg_PosMin );
                        if ( abs ( g_sMotor_Info.iqPosAbs - sMotorCmd->iqTargetPos ) > ParamHandle_Steer.Reg_CtrlDeadZone )
                        {
                            if ( g_sMotor_Info.iqPosAbs < sMotorCmd->iqTargetPos ) {
                                u16RunningDir = CtrlDir_Foward;
                            } else {
                                u16RunningDir = CtrlDir_Reverse;
                            }
                            sMotorCmd->eRunState = CmdRuning;
                            sMotorCmd->iqRuningTime = 0;
                        }
                        else
                        {
                            sMotorCmd->eRunState = CmdStop_Invalid; 
                        }
                    }
                }
                u16SteerCmdLastSN = Rx_SteerCtrlCmd.u16SN;
            }
            
            if ( sMotorCmd->eRunState == CmdRuning )
            {
                if ( sMotorCmd->iqRuningTime < ParamHandle_Steer.Reg_RunTimeOut ) sMotorCmd->iqRuningTime++;
                else
                {
                    sMotorCmd->eRunState = CmdTimeOut;
                }
            }
            eLastCmdMode = CmdMode_PosCtrl;
        }
        break;

        case CmdMode_FreeStop:
        {
            //电机主动刹车
            sMotorCmd->eRunState = EndOfCtrl;
            if ( Rx_SteerCtrlCmd.u16EN == 0 )
            {
                sMotorCmd->eRunState = CmdStop_Disable;
                u16SteerCmdLastSN = 0;
            }
            else if ( ( Rx_SteerCtrlCmd.u16Mode == CmdMode_Idle ) || ( Rx_SteerCtrlCmd.u16Mode > CmdMode_RC ) || 
                ( ( Rx_SteerCtrlCmd.u16Mode >= CmdMode_SpdCtrl ) && ( Rx_SteerCtrlCmd.u16Mode < CmdMode_TorqueCtrl ) ) )
            {
                sMotorCmd->eRunState = CmdStop_Invalid;
            }
            else
            {
                if ( ( u16StopFlag == 0 ) && ( Rx_SteerCtrlCmd.u16EN ) )
                {
                    if ( ( ( Rx_SteerCtrlCmd.u16Mode == CmdMode_RC ) && ( Rx_SteerCtrlCmd.u16Ref != 0 ) ) || \
                        ( ( Rx_SteerCtrlCmd.u16Mode > CmdMode_Idle ) && ( Rx_SteerCtrlCmd.u16Mode < CmdMode_RC ) ) )
                    {
                        sMotorCmd->eCmdMode = Rx_SteerCtrlCmd.u16Mode;
                    }
                }
            }
            eLastCmdMode = CmdMode_FreeStop;
        }
        break;

        case CmdMode_Fault:
        {
            sMotorCmd->eRunState = CmdFault;
            //电机被动刹车
            if ( g_u32FaultNum.all == NoFault ) //无故障，恢复
            {
                sMotorCmd->eCmdMode = CmdMode_Idle;
            }
            eLastCmdMode = CmdMode_Fault;
        }
        break;

        case CmdMode_RC:
        {
            if ( Rx_SteerCtrlCmd.u16Ref == 1 )
            {
                sMotorCmd->eRunState = CmdRuning;
                u32RCTime[0] += 2;
                u32RCTime[0] = _IQsat ( u32RCTime[0], _IQ(0.25),0);
                u32RCTime[1] = 0;
                sMotorCmd->iqTargetPos = _IQsat ( g_sMotor_Info.iqPosAbs+u32RCTime[0], ParamHandle_Steer.Reg_PosMax, ParamHandle_Steer.Reg_PosMin ); 
            }
            else if ( Rx_SteerCtrlCmd.u16Ref == 0xFFFF )
            {
                sMotorCmd->eRunState = CmdRuning;
                u32RCTime[0] = 0;
                u32RCTime[1] += 2;
                u32RCTime[1] = _IQsat ( u32RCTime[1], _IQ(0.25),0);
                sMotorCmd->iqTargetPos = _IQsat ( g_sMotor_Info.iqPosAbs-u32RCTime[1], ParamHandle_Steer.Reg_PosMax, ParamHandle_Steer.Reg_PosMin ); 
            } 
            else
            {
                u32RCTime[0] = 0;
                u32RCTime[1] = 0;
                sMotorCmd->eCmdMode = CmdMode_FreeStop;
            }
            eLastCmdMode = CmdMode_RC;            
        }
        break;
        
        default:
            break;
    }
}

//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


