/**
* ��Ȩ����(C)
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
* <����> | <�汾> | <����> | <����>
*
* xxxx/xx/xx | 1.0.0 | MUNIU | �����ļ�
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
#include "Foc.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------

//-------------------- private functions declare ----------------------------

//-------------------- public data ------------------------------------------
S_MOTOR_CMD g_sMotorCmd = { 0, 0, 0, 0, 0, 0, 0 };
uint16_t u16StopFlag = 0;

//-------------------- public functions -------------------------------------
/**
* @brief        ת��������״̬����
* @param       ���룺eCtrlMode:�泵����ģʽ����Դ:��λ������
                     SteerPosCtrlCmd��ת�����ָ���Դ:��λ����
               �����sMotorCmd:����֮��õ���������Ƶ�CMD
 * @retval      ��
 */
void MotorCmd_Ctrl ( RegDef_PosCtrlCmd *PosCtrlCmd, S_MOTOR_CMD *sMotorCmd )
{
    static E_CMD_MODE eLastCmdMode = CmdMode_Idle;
    static uint16_t u16SteerCmdLastSN = 0, u16Cnt_Init = 0;
    static RegDef_PosCtrlCmd Rx_SteerCtrlCmd = { 0, 0, 0, _IQ ( 0.5 ) };

    __disable_irq();         //�ر����ж�
    memcpy ( ( uint8_t * ) &Rx_SteerCtrlCmd, ( uint8_t * ) PosCtrlCmd, 8 );
    __enable_irq();          // �������ж�

    if ( g_u32FaultNum.all != NoFault ) sMotorCmd->eCmdMode = CmdMode_Fault;  //�й��ϣ��ù���״̬
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
            if ( sTeles_Ctrl.PosCali_OK )
            {
                sMotorCmd->eCmdMode = Rx_SteerCtrlCmd.u16Mode;
            }
            else
            {
                if ( eLastCmdMode != CmdMode_Idle ) u16Cnt_Init = 0;
                if ( u16Cnt_Init < ParamHandle_Steer.Reg_CaliTimeOut )
                {
                    u16Cnt_Init++;
                    sMotorCmd->eRunState = CmdRuning;
                }
                else sMotorCmd->eCmdMode = CmdMode_FreeStop;
            }
            eLastCmdMode = CmdMode_Idle;
        }
        break;

        case CmdMode_PosCtrl:
        {
            //ˢ��Ŀ���ٶ�
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
                        sMotorCmd->iqTargetPos = Rx_SteerCtrlCmd.u16Ref;
                        if ( HugTotalMin == sMotorCmd->iqTargetPos )  //����
                        {
                            sTeles_Ctrl.PosCmd_Dir = CtrlDir_Foward;
                            sMotorCmd->eRunState = CmdRuning;
                            sMotorCmd->iqRuningTime = 0;                            
                        }
                        else if ( HugTotalMax == sMotorCmd->iqTargetPos )
                        {
                            sTeles_Ctrl.PosCmd_Dir = CtrlDir_Reverse;
                            sMotorCmd->eRunState = CmdRuning;
                            sMotorCmd->iqRuningTime = 0;
                        }
                        else
                        {
                            sTeles_Ctrl.PosCmd_Dir = CtrlDir_Stop;
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
            //�������ɲ��
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
            //�������ɲ��
            if ( g_u32FaultNum.all == NoFault )  //�޹��ϣ��ָ�
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
                sMotorCmd->iqTargetSpd = ParamHandle_Steer.Reg_RC_Spdmax; 
            }
            else if ( Rx_SteerCtrlCmd.u16Ref == 0xFFFF )
            {
                sMotorCmd->eRunState = CmdRuning;
                sMotorCmd->iqTargetSpd = -ParamHandle_Steer.Reg_RC_Spdmax; 
            } 
            else
            {
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


