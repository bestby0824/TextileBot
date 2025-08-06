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
_iq g_qSteerTargetAngle = 0;            //Ŀ��λ��                      ������λ�ñ���ֵ
_iq g_qSteerMaxSpeed = _IQ ( 1.2 );     //���ת�����ٶ�                �����ٶȱ���ֵ/0.001
_iq g_qSteerGasAccKP = 1;               //�����kp
_iq g_qSteerThreshold = 10;             //�ٶȵ����������              �����ٶȱ���ֵ
uint16_t RUNSta, u16PosRef, G_StreePos;

int8_t SteeringWheel_Init ( void )
{
    return Err_None;
}
#if RS485_Ctrl
void SteeringWheel_Ctrl ( void )  //1KHz
{
    Wheel_CtrlCnt++;
    if ( Warning_Report.bit.BLDC_Power24V_Low == 0 )           //�ж����磬��ת�����巢�˶�ָ��
    {
        u16SteeringWheel_Step = PosCtrlByCmd;
        if ( 0 == HostCom_ReadSteerCommand ( &u16PosRef ) )
        {
            _iqSteerPosRef = _IQsat ( _IQdiv ( u16PosRef, 32767 ), _IQ ( 0.67 ), _IQ ( 0.33 ) ); //ת�����ƽǶ�-61��-+61��
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
    if ( ( Wheel_CtrlCnt % 2 ) == 0 )               //2mS1��
    {
        u16SteeringWheel_Step = GetAbsPos;
    }
    if ( ( Wheel_CtrlCnt % 10 ) == 0 )               //10mS1��
    {
        u16SteeringWheel_Step = GetPressure;
    }
    if ( ( Wheel_CtrlCnt % 1000 ) == 4 )            //1��һ��
    {
        u16SteeringWheel_Step = DogFeed;
    } else if ( ( Wheel_CtrlCnt % 1000 ) == 204 )   //1��һ��
    {
        u16SteeringWheel_Step = GetSta;
    } else if ( ( Wheel_CtrlCnt % 1000 ) == 504 )   //1Sһ��
    {
        u16SteeringWheel_Step = GetSWVer;
    }
    if ( u8FaultRetFlag == 1 ) //�����������ȼ���ߣ��Ƕ�ʱ����
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
    RUNSta = GetRUNSta();    //����Ƿ���ϵ�

//    if ( ( _iqSteerPosRef < _IQ ( 0.48 ) ) && ( ( RUNSta == RunSta_Run_D ) || ( RUNSta == RunSta_Run_R ) ) )
//    {
//        DoSetSteeringLampSW ( Sta_L ); //��ת�����
//    }
//    else if ( ( _iqSteerPosRef > _IQ ( 0.52 ) ) && ( ( RUNSta == RunSta_Run_D ) || ( RUNSta == RunSta_Run_R ) ) )
//    {
//        DoSetSteeringLampSW ( Sta_R ); //��ת�����
//    }
//    else
//    {
//        DoSetSteeringLampSW ( Sta_N ); //ת�����
//    }
}
void SteeringWheel_SetPos ( _iq PosRef )
{
    _iqSteerPosRef = PosRef;
}
#elif CAN_Ctrl
/**
  * @brief  ת���ת���Ĵ�����亯��
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
//    else if ( ( m_eCtrlMode == RC_MODE ) || ( m_eCtrlMode == PC_MODE ) ) //ң��ģʽ||�Զ�ģʽ
//    {
////        if ( u16PosCheckPass_Flag == 1 )  //�ϵ��Լ����֮�󣬽���λ�ÿ���
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
    case CmdMode_PosCtrl://λ�ÿ���״̬
    {
        RegReal_SteerPosCtrlCmd.u16Mode = eCmdMode;
        RegReal_SteerPosCtrlCmd.u16EN = ENABLE;
//        HostCom_ReadSteerCommand ( &RegReal_SteerPosCtrlCmd.u16PosRef );//��ȡλ�òο�ֵ
    }
    break;
    case CmdMode_FreeStop://����״̬
    {
        RegReal_SteerPosCtrlCmd.u16Mode = eCmdMode;
        RegReal_SteerPosCtrlCmd.u16EN = DISABLE;
    }
    break;
    case CmdMode_Fault://�й���
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


