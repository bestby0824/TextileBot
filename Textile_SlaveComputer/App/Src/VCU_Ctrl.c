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
#include "VCU_Ctrl.h"
#include "VCUProcess.h"
#include "Spd_Ctrl.h"
#include "pullrod_Ctrl.h"
#include "Xint.h"
#include "StateCtrl.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------

uint16_t LiftCmdCnt, RotateCmdCnt, BowRiseCmdCnt, ClawCmdCnt, LiftRaderCmdCnt, SideShiftCmdCnt, SlowSpdCmdCnt, BrakeCanelCmdCnt, BrakeCmdCnt;          //100ms
uint16_t LiftPumpMotorSpd = 3000, ClawPumpMotorSpd = 1300, OtherPumpMotorSpd = 1000, RotatePumpMotorSpd = 1000;
uint16_t RunState_RC, LiftDir;
uint8_t u8ChassisMotorAcc_Test = 250;
uint16_t Claw_Pump_Time = 0;
uint16_t Claw_PumpClos_Time = 0;
_iq LiftUpErr = 0;
float LiftPumpKp = 0.278;
_iq iqPosition_m;
_iq iq_Position, iq_PositionDelta, iq_PositionLast;
_iq iq_PosAddCircles;
void PosAddCalc ()
{

    iq_Position = WalkPosClc ( Timer3Counter );

    iq_PositionDelta = iq_Position - iq_PositionLast;//��ת�Ǹ�ֵ

    if ( iq_PositionDelta < -_IQ ( 0.5 ) ) //��ת����
    {
        iq_PositionDelta += _IQ ( 1 );
    }

    if ( iq_PositionDelta > _IQ ( 0.5 ) ) //��ת����
    {
        iq_PositionDelta -=  _IQ ( 1 );
    }

    iq_PositionLast = iq_Position;
    iq_PosAddCircles += iq_PositionDelta;//���װ����iqDirectionFlg��ֵ����ֵiqDirectionFlg *
    iqPosition_m = _IQrmpy ( LengthOf_1_Circle, iq_PosAddCircles ); //λ�÷�����ʵ�������ջ�,unit m
    //���ۼӺ��㣬�����ܼ�������������ۼ����
}
_iq iqOldAddPos = 0;
_iq iqNewSpdRPM;
_iq OuTPos;
//-------------------- private functions declare ----------------------------
_iq LiftSpdCalc ( uint16_t CalcFreq )
{
    static _iq iqSpd_PU;

    _iq iqNowAddPos = u32LiftPos;

//    iqNowAddPos = *psInfo;
    //iqNowAddPos = GetReangleValue ( );
    OuTPos = ( iqNowAddPos - iqOldAddPos );//���װ����iqDirectionFlg
//    if ( OuTPos < -_IQ ( 0.5 ) )
//    {
//        OuTPos += _IQ ( 1 );
//    }
//    if ( OuTPos > _IQ ( 0.5 ) )
//    {
//        OuTPos -= _IQ ( 1 );
//    }
    iqNewSpdRPM = OuTPos * CalcFreq;  //IQ(1):1mps

    // iqSpd_PU = _IQrmpy ( iqSpd_PU, _IQ ( 0.9 ) ) + _IQrmpy ( _IQdiv ( iqNewSpdRPM, SPD_MAX_RPM ), _IQ ( 0.1 ) );//�ٶȷ���?��ͨ�˲���ϵ����ôѡ�ģ���ֹƵ��63HZ?


    iqOldAddPos = iqNowAddPos;
    return iqSpd_PU;
}
//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
void VCU_RegState_Ctrl ( void );
//uint8_t AGVCtrlMode;
//int16_t s16AGVWalkSpd;
//uint8_t AGVLiftCtrlMode;
uint8_t AGVTurnCtrlMode;
int32_t TurnSpd = 3000;
int32_t TurnDeg;//, TurnDegFb;
//uint8_t u8WorkMode;
//uint16_t u16CtrlWord;
uint32_t u32LiftPos;
//int16_t s16WalkSpd;
//_iq iqPosRef_m = 0;
_iq PosLpKp = 10000;
//_iq LiftSpd;
uint16_t LiftSpdKp = 0x0002;
//_iq PosLpup = 0;

int16_t s16AGVWalkBrakePosFB = 0;
uint16_t AGVLiftEndOfCtrlCnt = 0;
//int32_t LiftPosRef = 0;
uint16_t  AGVWalkBrakeCnt = 0, AGVSlowSpdPosFB = 0, AGVSlowSpdPosCnt = 0, BreakCanelCnt = 0;
uint8_t u8PumpSpd;
int32_t LiftPosup = 0;
int32_t s32PumpSpd = 0;
//int32_t TurnPosTest = 0, TurnPCPos;
/**
 * @brief       
 * @param       
 * @retval      ��
 */
void AGVLiftPosLp ( uint32_t ref )
{
    LiftPosup = ref - u32LiftPos;//�����߷ֱ����й�
    if ( LiftPosup > 0 )
    {
        s32PumpSpd = ( LiftSpdKp * LiftPosup ) >> 3;
//        u8PumpSpd = _IQdiv ( LiftSpdKp * LiftPosup, _IQ ( 10 ) );//��Ӧ�����ת��
        u8PumpSpd = _IQsat ( s32PumpSpd, LiftPumpMaxSpd, 0 );
        Reg_AGVCtrlCmd2.u8PumpSpd = u8PumpSpd;
        Reg_AGVCtrlCmd1.u8WalkDnPV = 0;
        Reg_AGVCtrlCmd1.EMVCtrl = 1;
    }
    else if ( LiftPosup < 0 )
    {
        s32PumpSpd = abs(( LiftSpdKp * LiftPosup ) >> 3);
//        u8PumpSpd = _IQdiv ( LiftSpdKp * LiftPosup, _IQ ( 10 ) );
        u8PumpSpd = _IQsat ( s32PumpSpd, LiftDnMaxSpd, 0 );
        Reg_AGVCtrlCmd2.u8PumpSpd = 0;
        Reg_AGVCtrlCmd1.u8WalkDnPV = u8PumpSpd;
        Reg_AGVCtrlCmd1.EMVCtrl = 0;
    }
    else
    {
        Reg_AGVCtrlCmd1.u8WalkDnPV = 0;
        Reg_AGVCtrlCmd2.u8PumpSpd = 0;
        Reg_AGVCtrlCmd1.EMVCtrl = 0;
    }
    if ( _IQrmpy(u32LiftPos, CONSTANT_1500_DIV_1024) >= LiftPosMax )
    {
        Reg_AGVCtrlCmd2.u8PumpSpd = 0;
    }
    if ( abs ( LiftPosup ) <= _IQdiv(100, CONSTANT_1500_DIV_1024))
    {
        AGVLiftEndOfCtrlCnt++;
    }
    else
    {
        AGVLiftEndOfCtrlCnt = 0;
    }
}
_iq iqAGVWalkPosup;
/**
 * @brief       
 * @param       
 * @retval      ��
 */

int16_t AGVWalkPosLp ( _iq iqPosRef_m )
{
    static uint16_t u16Acc = 3;
    static int32_t s32AGVWalkSpd_div10 = 0;
    iqAGVWalkPosup = iqPosRef_m - iqPosition_m;
    s32AGVWalkSpd_div10 = ramper ( _IQrmpy ( PosLpKp, iqAGVWalkPosup ), s32AGVWalkSpd_div10, u16Acc );
    return _IQsat ( _IQdiv ( s32AGVWalkSpd_div10, _IQ ( 10 ) ), WalkspdMax, -WalkspdMax );
}
/**
 * @brief       
 * @param       
 * @retval      ��
 */
void AGVTurnCtrl(void)
{
    switch ( AGVTurnCtrlMode )
    {
//        case 1://�ϵ�
//        {
//            Reg_AGVTurnCtrlCmd1.u16CtrlWord.all = 0x06;
//            AGVTurnCtrlMode = 2;
//        }
//        break;
//        case 2://�ϵ�
//        {
//            Reg_AGVTurnCtrlCmd1.u16CtrlWord.all = 0x0F;
//            AGVTurnCtrlMode = 3;
//        }
//        break;
        case 1://����
        {
            Reg_AGVTurnCtrlCmd1.s32MotorPosCmd.all = TurnDeg;
            Reg_AGVTurnCtrlCmd1.u8WorkMode = 1;
            Reg_AGVTurnCtrlCmd1.u16CtrlWord.all = 0x103F;
            Reg_AGVTurnCtrlCmd2.s32PosLpSpdCmd = TurnSpdCLc(TurnSpd);
            Reg_AGVTurnCtrlCmd2.s32SpdLpSpdCmd = 0;
//            Reg_AGVCtrlCmd2.u8WalkAccTime = 50;
//            Reg_AGVCtrlCmd2.u8WalkDecTime = 10;
        }
        break;
        default:
        break;
    }
}
/**
 * @brief       VCU��д�Ĵ���״̬���£�1000HZ
 * @param       �Ĵ���ID
 * @retval      ��
 */
_iq iqAgvWalkPosRef_m;
_iq iqAGVWalkPosFb = 65535;
void VCU_RegState_Ctrl ( void )
{
    iqAGVWalkPosFb = _IQdiv ( Timer3Counter, ENCLines );
    if ( iqAGVWalkPosFb >= 65536 )
    {
        iqAGVWalkPosFb = iqAGVWalkPosFb - 65536;
    }
	
    if ( m_eCtrlMode == PC_MODE )             //�Զ�ģʽ��ȡ��λ��ָ��
    {
       
         AGVTurnCtrl(); 
		u32LiftPos = * ( uint32_t * ) Reg_DrawWireSensorFb.u8Databuf;
        /*��������*/
       if(SolenoidCtrlCmd.Lift.EN == 1)	
       { 
            if ( SolenoidCtrlCmd.Lift.CtrlMode == 0 ) //����
            {
                AGVLiftPosLp ( SolenoidCtrlCmd.Lift.unPos );
                if ( AGVLiftEndOfCtrlCnt > 500 ) //0.5s
                {
                    SolenoidCmddown.liftEndOfCtrl = 0;
                    //AGVLiftEndOfCtrlCnt = 0;
                }
            }
            else if ( SolenoidCtrlCmd.Lift.CtrlMode == 1 ) //
            {
                Reg_AGVCtrlCmd1.u8WalkDnPV = 200;
                Reg_AGVCtrlCmd1.EMVCtrl = 0;
                Reg_AGVCtrlCmd2.u8PumpSpd = 0;
            } else if ( SolenoidCtrlCmd.Lift.CtrlMode == 2 )
            {
                Reg_AGVCtrlCmd2.u8PumpSpd = 255;
                Reg_AGVCtrlCmd1.u8WalkDnPV = 0;
                Reg_AGVCtrlCmd1.EMVCtrl = 1;
            }
        }
        /*�н�����*/
        if ( PosModeFlag ) //����ɲ��
        {
            Reg_AGVCtrlCmd1.s16WalkSpd =  AGVWalkPosLp ( - ( Diatance << 1 ) );
            s16AGVWalkBrakePosFB = ( int16_t ) ( ( -iqPosition_m ) >> 1 );//����
            if ( abs(iqAGVWalkPosup) <= _IQ(0.01) )
            {
                AGVWalkBrakeCnt++;
                //SolenoidCmddown.Brake_EndOfCtrl = 0;
            }
            else
            {
                AGVWalkBrakeCnt = 0;
            }
            if ( AGVWalkBrakeCnt > 500 ) //0.5s
            {
                SolenoidCmddown.Brake_EndOfCtrl = 0;
            }
            if(FaultCtrl.all || RegReal_RotatingMBDFaultStaFB.u32Fault || RegReal_TelescopicMBDFaultStaFB.u32Fault \
                || (RegReal_HuggingMBDFaultStaFB.u32Fault) || (RegReal_GlandMBDFaultStaFB.u32Fault) || (Reg_AGVDataFb1.u8WalkFaultCode) \
                || (Reg_AGVPumpCtrFb.u8WalkFaultCode) || (Reg_AGVBMSStaOrigDataFb.u8FaultCode))
            {
                AGVWalkBrakeCnt = 0;
                SolenoidCmddown.Brake_EndOfCtrl = 2;
            }
        }
        else if ( SlowDir == 1 ) //�䶯
        {
            iqAgvWalkPosRef_m = -_IQrmpy ( SlowDiatance, LengthOf_1_Circle );
            Reg_AGVCtrlCmd1.s16WalkSpd =  AGVWalkPosLp ( iqAgvWalkPosRef_m );
            AGVSlowSpdPosFB = _IQdiv ( -iqPosition_m, LengthOf_1_Circle );
            if ( abs(iqAGVWalkPosup) <= _IQ(0.01) )
            {
                AGVSlowSpdPosCnt++;
            }
            else
            {
                AGVSlowSpdPosCnt = 0;
            }
            if ( AGVSlowSpdPosCnt > 500 )
            {
                SolenoidCmddown.SlowSpd_EndOfCtrl = 0;
            }
            if(FaultCtrl.all || RegReal_RotatingMBDFaultStaFB.u32Fault || RegReal_TelescopicMBDFaultStaFB.u32Fault \
                || (RegReal_HuggingMBDFaultStaFB.u32Fault) || (RegReal_GlandMBDFaultStaFB.u32Fault) || (Reg_AGVDataFb1.u8WalkFaultCode) \
                || (Reg_AGVPumpCtrFb.u8WalkFaultCode) || (Reg_AGVBMSStaOrigDataFb.u8FaultCode))
            {
                AGVSlowSpdPosCnt = 0;
                SolenoidCmddown.SlowSpd_EndOfCtrl = 2;
            }
        }
        else if ( SlowDir == 2 ) //�䶯
        {
            iqAgvWalkPosRef_m = -_IQrmpy ( SlowDiatance, LengthOf_1_Circle );
            Reg_AGVCtrlCmd1.s16WalkSpd =  AGVWalkPosLp ( -iqAgvWalkPosRef_m );
            AGVSlowSpdPosFB = _IQdiv ( iqPosition_m, LengthOf_1_Circle );
            if ( abs(iqAGVWalkPosup) <= _IQ(0.01) )
            {
                AGVSlowSpdPosCnt++; 
            }
            else
            {
                AGVSlowSpdPosCnt = 0;
            }
            if ( AGVSlowSpdPosCnt > 500 )
            {
                SolenoidCmddown.SlowSpd_EndOfCtrl = 0;
            }
            if(FaultCtrl.all || RegReal_RotatingMBDFaultStaFB.u32Fault || RegReal_TelescopicMBDFaultStaFB.u32Fault \
                || (RegReal_HuggingMBDFaultStaFB.u32Fault) || (RegReal_GlandMBDFaultStaFB.u32Fault) || (Reg_AGVDataFb1.u8WalkFaultCode) \
                || (Reg_AGVPumpCtrFb.u8WalkFaultCode) || (Reg_AGVBMSStaOrigDataFb.u8FaultCode))
            {
                AGVSlowSpdPosCnt = 0;
                SolenoidCmddown.SlowSpd_EndOfCtrl = 2;
            }
        }
        else if ( BreakCanelSwitch == 1 ) //����ɲ��
        {
            Reg_AGVCtrlCmd1.s16WalkSpd = 0;
//            if ( Reg_AGVDataFb1.s16WalkSpd == 0 )
//            {
//                BreakCanelCnt++;
//                if(FaultCtrl.all || RegReal_RotatingMBDFaultStaFB.u32Fault || RegReal_TelescopicMBDFaultStaFB.u32Fault \
//                || (RegReal_HuggingMBDFaultStaFB.u32Fault) || (RegReal_GlandMBDFaultStaFB.u32Fault) || (Reg_AGVDataFb1.u8WalkFaultCode) \
//                || (Reg_AGVPumpCtrFb.u8WalkFaultCode) || (Reg_AGVBMSStaOrigDataFb.u8FaultCode))
//                {
//                    BreakCanelCnt = 0;
//                    SolenoidCmddown.BreakCanel_EndOfCtrl = 2;
//                }
//            }
//            else
//            {
//                BreakCanelCnt = 0;
//            }
//            if ( BreakCanelCnt > 1000 ) //
//            {
//                SolenoidCmddown.BreakCanel_EndOfCtrl = 0;
//            }
        }
        else
        {
            Reg_AGVCtrlCmd1.s16WalkSpd = -_IQsat ( s16TargetSpeed, WalkspdMax, -WalkspdMax );
        }
       
    }
    /*���ϼ���ͣ*/
   // if ( InputIOInfor.TouchEdgeSta || InputIOInfor.STOP_0Sta || RegReal_RCMODECmd.u16Cmd == RC_ESTOPCmd )
    if ( InputIOInfor.TouchEdgeSta || InputIOInfor.STOP_0Sta || Reg_RCCtrlCmdFb.u8EmergencyStopCmd == 1 )
    {
        FaultCtrl.bit.EmergencyStop = 1; 
    }
    else
    {
        FaultCtrl.bit.EmergencyStop = 0;
    } 
    if( StopFlag == 2 )
    {
        if(Reg_AGVDataFb1.s16WalkSpd < 0)//ǰ��
        {
            Reg_AGVCtrlCmd1.s16WalkSpd = -462;
        }
        else if(Reg_AGVDataFb1.s16WalkSpd > 0)
        {
            Reg_AGVCtrlCmd1.s16WalkSpd = 462;
        }
    }
    else if(StopFlag == 3)
    {
        STOP_0_Ctrl(ON);
        FaultCtrl.bit.EmergencyStop = 1; 
    }
    FaultCtrl.bit.TurnError = Reg_AGVTurnFb.u16StaWord.bit.MotorErr; 
    Warning_Report.bit.TurnWarning = Reg_AGVTurnFb.u16StaWord.bit.MotorWarning;
    if(FaultCtrl.all || RegReal_RotatingMBDFaultStaFB.u32Fault || RegReal_TelescopicMBDFaultStaFB.u32Fault \
            || (RegReal_HuggingMBDFaultStaFB.u32Fault) || (RegReal_GlandMBDFaultStaFB.u32Fault) || (Reg_AGVDataFb1.u8WalkFaultCode) \
        || (Reg_AGVPumpCtrFb.u8WalkFaultCode) || (Reg_AGVBMSStaOrigDataFb.u8FaultCode))
    {
        Reg_AGVCtrlCmd1.s16WalkSpd = 0;
        Reg_AGVTurnCtrlCmd2.s32PosLpSpdCmd = 0;
        Reg_AGVCtrlCmd2.u8PumpSpd = 0;
        Reg_AGVCtrlCmd1.u8WalkDnPV = 0;
//        Reg_AGVTurnCtrlCmd1.u16CtrlWord.all = 0x0B;
    }
    Reg_AGVBMSCtrlCmd.u8InquiryBMSSta = 0xF4;
    Reg_AGVCtrlCmd2.u8WalkAccTime = ParamHandle_Control.Reg_PCWalkAccTime;
    Reg_AGVCtrlCmd2.u8WalkDecTime = ParamHandle_Control.Reg_PCWalkDecTime;
//    TurnDegFb = INVReduction_Ratio ( Reg_AGVTurnFb.s32MotorPosFb );
    if((Reg_AGVTurnFb.u16StaWord.all & 0x07) == 0){
        Reg_AGVTurnCtrlCmd1.u16CtrlWord.all = 0x06;
    }
    

}

//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


