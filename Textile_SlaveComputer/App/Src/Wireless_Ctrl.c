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

//-------------------- include files -------------------------------------------
#include "Wireless_Ctrl.h"
#include "VCU_Ctrl.h"
#include "VCUProcess.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
uint8_t u8FaultClear_Flag = 0;
//#define POSCtrl
//-------------------- private functions declare ----------------------------
static void WirelessSend()
{
    if ( RCMODEComFlag == 1 )
    {
        RCMODEComFlag = 0;
        RCMODE_Send();
    }
}
//-------------------- public data ------------------------------------------
int16_t s16WalkSpdTest = 0;
uint16_t u16TurnCtrlWord = 0x103F;
uint8_t u8TurnWorkMode = 1;
//-------------------- public functions -------------------------------------
void WirelessCtrl()
{
#if 0
    uint16_t PairingFlag = ParamHandle_Control.Reg_PairingFlag;

    if ( PairingFlag == 0 ) {
        switch ( RegReal_RCMODECmd.u16Cmd )
        {
        case PairingReqCmd:
        {
            ParamHandle_Control.Reg_WirelessBDSN = WirelessBDSN;
            if ( Param2Eeprom () )
            {
                FaultCtrl.bit.EepromWriteErr = 1;
                TxRegReal_RCMODE.u16Data = 1;
            }
            else
            {
                TxRegReal_RCMODE.u16Data = 0;
                PairingFlag = 1;
                ParamHandle_Control.Reg_PairingFlag = PairingFlag;
                if ( Param2Eeprom () )
                {
                    FaultCtrl.bit.EepromWriteErr = 1;
                    TxRegReal_RCMODE.u16Data = 1;
                    PairingFlag = 0;
                }
            }
        }
        break;

        case PairingClcCmd:
        {
            TxRegReal_RCMODE.u16Data = 0;
            PairingFlag = 0;
            ParamHandle_Control.Reg_PairingFlag = PairingFlag;
            ParamHandle_Control.Reg_WirelessBDSN = 0xFFFFFFFF;
            if ( Param2Eeprom () )
            {
                FaultCtrl.bit.EepromWriteErr = 1;
                TxRegReal_RCMODE.u16Data = 1;
                PairingFlag = 1;
            }
            WirelessSend();
        }
        break;
        default:
            break;
        }
        WirelessSend();
    } else {
        if ( PairingClcCmd == RegReal_RCMODECmd.u16Cmd )
        {
            TxRegReal_RCMODE.u16Data = 0;
            PairingFlag = 0;
            ParamHandle_Control.Reg_PairingFlag = PairingFlag;
            ParamHandle_Control.Reg_WirelessBDSN = 0xFFFFFFFF;
            if ( Param2Eeprom () )
            {
                FaultCtrl.bit.EepromWriteErr = 1;
                TxRegReal_RCMODE.u16Data = 1;
                PairingFlag = 1;
            }
            WirelessSend();
        }
        if ( ParamHandle_Control.Reg_WirelessBDSN == WirelessBDSN ) {
            TxRegReal_RCMODE.u16Data = 0;

            switch ( RegReal_RCMODECmd.u16Cmd )
            {
            case PairingReqCmd:
            {
                TxRegReal_RCMODE.u16Data = 0;
            }
            break;
            case PairingClcCmd:
            {
                TxRegReal_RCMODE.u16Data = 0;
                PairingFlag = 0;
                ParamHandle_Control.Reg_PairingFlag = PairingFlag;
                ParamHandle_Control.Reg_WirelessBDSN = 0xFFFFFFFF;
                if ( Param2Eeprom () )
                {
                    FaultCtrl.bit.EepromWriteErr = 1;
                    TxRegReal_RCMODE.u16Data = 1;
                    PairingFlag = 1;
                }
            }
            break;
            case RC_ESTOPCmd:
            {
                if ( RegReal_RCMODECmd.u16Data == 1 )
                {
                    FaultCtrl.bit.EmergencyStop = 1;
                    u8FaultClear_Flag = 1;
                }
//                    else if ( RegReal_RCMODECmd.u16Data == 0 )
//                    {
//                        FaultCtrl.all = 0;
//                    }
            }
            break;
            case RC_RunningCmd:
            {
                if ( u8FaultClear_Flag == 1 ) {
                    u8FaultClear_Flag = 0;
                    FaultCtrl.all = 0;
                }
                if ( FaultCtrl.bit.EmergencyStop == 0 ) {
                    if ( RegReal_RCMODECmd.u8Data[0] == 0x01 ) {//后退
//                            AGVCtrlMode = RC_Debug;

#ifdef POSCtrl
                        iqPosRef_m = _IQ ( 2 );
                        iq_PosAddCircles = 0;
#else
//                            iqPosRef_m = _IQ(2)+iqPosition_m;
                        Reg_AGVCtrlCmd1.s16WalkSpd = 1000;
#endif
                        Reg_AGVCtrlCmd2.u8WalkAccTime = 50;
                        Reg_AGVCtrlCmd2.u8WalkDecTime = 10;
                    } else if ( RegReal_RCMODECmd.u8Data[0] == 0xFF ) {//前进
//                            AGVCtrlMode = RC_Debug;

#ifdef POSCtrl
                        iqPosRef_m = -_IQ ( 2 );
                        iq_PosAddCircles = 0;
#else
//                            iqPosRef_m = -_IQ(2)+iqPosition_m;
                        Reg_AGVCtrlCmd1.s16WalkSpd = -1000;
#endif
                        Reg_AGVCtrlCmd2.u8WalkAccTime = 50;
                        Reg_AGVCtrlCmd2.u8WalkDecTime = 10;
                    } else if ( RegReal_RCMODECmd.u8Data[0] == 0 ) {
//                            AGVCtrlMode = RC_Debug;
#ifndef POSCtrl
                        Reg_AGVCtrlCmd1.s16WalkSpd = 0;
                        //iq_PosAddCircles = 0;
//                            Reg_AGVCtrlCmd1.s16WalkSpd =  AGVWalkPosLp(iqPosRef_m);
#endif
                    }
#ifdef POSCtrl
                    Reg_AGVCtrlCmd1.s16WalkSpd =  AGVWalkPosLp ( iqPosRef_m );
#endif
                    AGVTurnCtrlMode = Forward;
                    TurnDeg = _IQsat ( _IQ8toIQ ( RegReal_RCMODECmd.u8Data[1] ), _IQ ( 0.75 ), _IQ ( 0.25 ) ) - _IQ ( 0.5 );
                    TurnSpd = 3000;
                    if ( TurnDeg >= 1800 )
                    {
                        TurnSignalCtrl ( Sta_L );
                    }
                    else if ( TurnDeg <= -1800 )
                    {
                        TurnSignalCtrl ( Sta_R );
                    }
                    else
                    {
                        TurnSignalCtrl ( Sta_N );
                    }
                    if ( RegReal_RCMODECmd.u8Data[2] == 0x01 ) //升降
                    {

//                            AGVCtrlMode = RC_Debug;
                        Reg_AGVCtrlCmd1.EMVCtrl = 1;
                        Reg_AGVCtrlCmd1.u8WalkDnPV = 0;
                        Reg_AGVCtrlCmd2.u8PumpSpd = 100;
                    } else if ( RegReal_RCMODECmd.u8Data[2] == 0xFF ) {
//                            AGVCtrlMode = RC_Debug;
                        Reg_AGVCtrlCmd1.EMVCtrl = 0;
                        Reg_AGVCtrlCmd1.u8WalkDnPV = 100;
                        Reg_AGVCtrlCmd2.u8PumpSpd = 0;
                    } else if ( RegReal_RCMODECmd.u8Data[2] == 0 ) {
//                            AGVCtrlMode = RC_Debug;
                        Reg_AGVCtrlCmd1.EMVCtrl = 0;
                        Reg_AGVCtrlCmd1.u8WalkDnPV = 0;
                        Reg_AGVCtrlCmd2.u8PumpSpd = 0;
                    }
                    RegReal_RotatingMBDCmd.u16EN = 1;
                    RegReal_RotatingMBDCmd.u16SN = 1;
                    RegReal_RotatingMBDCmd.u16Cmd = 6;
                    if ( RegReal_RCMODECmd.u8Data[3] == 1 ) //旋转
                    {
                        RegReal_RotatingMBDCmd.u16Ref = 1;
                    }
                    else if ( RegReal_RCMODECmd.u8Data[3] == 0xFF )
                    {
                        RegReal_RotatingMBDCmd.u16Ref = 0xFFFF;
                    }
                    else
                    {
                        RegReal_RotatingMBDCmd.u16Ref = 0;
                    }

                    RegReal_TelescopicMBDCmd.u16EN = 1;
                    RegReal_TelescopicMBDCmd.u16SN = 1;
                    RegReal_TelescopicMBDCmd.u16Cmd = 6;
                    if ( RegReal_RCMODECmd.u8Data[4] == 1 )
                    {
                        RegReal_TelescopicMBDCmd.u16Ref = 1;
                    }
                    else if ( RegReal_RCMODECmd.u8Data[4] == 0xFF ) //伸出
                    {
                        RegReal_TelescopicMBDCmd.u16Ref = 0xFFFF;
                    }
                    else
                    {
                        RegReal_TelescopicMBDCmd.u16Ref = 0;
                    }

                    RegReal_HuggingMBDCmd.u16EN = 1;
                    RegReal_HuggingMBDCmd.u16SN = 1;
                    RegReal_HuggingMBDCmd.u16Cmd = 6;
                    if ( RegReal_RCMODECmd.u8Data[5] == 1 ) //环抱
                    {
                        RegReal_HuggingMBDCmd.u16Ref = 1;
                    }
                    else if ( RegReal_RCMODECmd.u8Data[5] == 0xFF )
                    {
                        RegReal_HuggingMBDCmd.u16Ref = 0xFFFF;
                    }
                    else
                    {
                        RegReal_HuggingMBDCmd.u16Ref = 0;
                    }

                    RegReal_GlandMBDCmd.u16EN = 1;
                    RegReal_GlandMBDCmd.u16SN = 1;
                    RegReal_GlandMBDCmd.u16Cmd = 6;
                    if ( RegReal_RCMODECmd.u8Data[6] == 1 ) //抬压
                    {
                        RegReal_GlandMBDCmd.u16Ref = 1;
                    }
                    else if ( RegReal_RCMODECmd.u8Data[6] == 0xFF )
                    {
                        RegReal_GlandMBDCmd.u16Ref = 0xFFFF;
                    }
                    else
                    {
                        RegReal_GlandMBDCmd.u16Ref = 0;
                    }
                    if ( RegReal_RCMODECmd.u8Data[7] == 1 ) //电池电量低
                    {
                        Warning_Report.bit.HandleBatteryLow = 1;
                        memset ( &Reg_AGVCtrlCmd1, 0, sizeof ( Reg_AGVCtrlCmd1 ) );
                        memset ( &Reg_AGVCtrlCmd2, 0, sizeof ( Reg_AGVCtrlCmd2 ) );
                        RegReal_GlandMBDCmd.u16EN = 0;
                        RegReal_HuggingMBDCmd.u16EN = 0;
                        RegReal_TelescopicMBDCmd.u16EN = 0;
                        RegReal_RotatingMBDCmd.u16EN = 0;
                    }
                    else
                    {
                        Warning_Report.bit.HandleBatteryLow = 0;

                    }
                } else {
                    memset ( &Reg_AGVCtrlCmd1, 0, sizeof ( Reg_AGVCtrlCmd1 ) );
                    memset ( &Reg_AGVCtrlCmd2, 0, sizeof ( Reg_AGVCtrlCmd2 ) );
                    RegReal_GlandMBDCmd.u16EN = 0;
                    RegReal_HuggingMBDCmd.u16EN = 0;
                    RegReal_TelescopicMBDCmd.u16EN = 0;
                    RegReal_RotatingMBDCmd.u16EN = 0;
                }
            }
            break;

            default:
                break;
            }
            RegReal_RCMODECmd.u16Cmd = 0;

            WirelessSend();

        }
    }
#endif
    if ( FaultCtrl.bit.EmergencyStop == 0 ) {//
        if ( Reg_RCCtrlCmdFb.u8WalkCmd == 1 ) {//后退
            Reg_AGVCtrlCmd1.s16WalkSpd = ParamHandle_Control.Reg_WLSpdLimit;
            Reg_AGVCtrlCmd2.u8WalkAccTime = ParamHandle_Control.Reg_WLWalkAccTime;
            Reg_AGVCtrlCmd2.u8WalkDecTime = ParamHandle_Control.Reg_WLWalkDecTime;
        } else if ( Reg_RCCtrlCmdFb.u8WalkCmd == 0xFF ) {//前进
            Reg_AGVCtrlCmd1.s16WalkSpd = -ParamHandle_Control.Reg_WLSpdLimit;
            Reg_AGVCtrlCmd2.u8WalkAccTime = ParamHandle_Control.Reg_WLWalkAccTime;
            Reg_AGVCtrlCmd2.u8WalkDecTime = ParamHandle_Control.Reg_WLWalkDecTime;
        } else if ( Reg_RCCtrlCmdFb.u8WalkCmd == 0 ) {
            Reg_AGVCtrlCmd1.s16WalkSpd = s16WalkSpdTest;// 0
        }
     
        TurnDeg = _IQsat ( _IQ8toIQ ( Reg_RCCtrlCmdFb.u8TurnCmd ), _IQ ( 0.75 ), _IQ ( 0.25 ) ) - _IQ ( 0.5 );//0.4985
        TurnSpd = 3000;
        if ( TurnDeg <= -1800 )
        {
            TurnSignalCtrl ( Sta_L );
        }
        else if ( TurnDeg >= 1800 )
        {
            TurnSignalCtrl ( Sta_R );
        }
        else
        {
            TurnSignalCtrl ( Sta_N );
        }
        
        Reg_AGVTurnCtrlCmd1.s32MotorPosCmd.all = Reduction_Ratio ( TurnDeg );
        Reg_AGVTurnCtrlCmd1.u8WorkMode = u8TurnWorkMode;
        Reg_AGVTurnCtrlCmd1.u16CtrlWord.all = u16TurnCtrlWord;
        Reg_AGVTurnCtrlCmd2.s32PosLpSpdCmd = TurnSpdCLc ( TurnSpd );
        Reg_AGVTurnCtrlCmd2.s32SpdLpSpdCmd = 0;
        
        if ( Reg_RCCtrlCmdFb.u8LiftCmd == 0x01 ) //升降
        {
            Reg_AGVCtrlCmd1.EMVCtrl = 1;
            Reg_AGVCtrlCmd1.u8WalkDnPV = 0;
            Reg_AGVCtrlCmd2.u8PumpSpd = 100;
        } else if ( Reg_RCCtrlCmdFb.u8LiftCmd == 0xFF ) {
            Reg_AGVCtrlCmd1.EMVCtrl = 0;
            Reg_AGVCtrlCmd1.u8WalkDnPV = 100;
            Reg_AGVCtrlCmd2.u8PumpSpd = 0;
        } else if ( Reg_RCCtrlCmdFb.u8LiftCmd == 0 ) {
            Reg_AGVCtrlCmd1.EMVCtrl = 0;
            Reg_AGVCtrlCmd1.u8WalkDnPV = 0;
            Reg_AGVCtrlCmd2.u8PumpSpd = 0;
        }
        RegReal_RotatingMBDCmd.u16EN = 1;
        RegReal_RotatingMBDCmd.u16SN = 1;
        RegReal_RotatingMBDCmd.u16Cmd = 6;
        if ( Reg_RCCtrlCmdFb.u8RotatingCmd == 1 ) //旋转
        {
            RegReal_RotatingMBDCmd.u16Ref = 1;
        }
        else if ( Reg_RCCtrlCmdFb.u8RotatingCmd == 0xFF )
        {
            RegReal_RotatingMBDCmd.u16Ref = 0xFFFF;
        }
        else
        {
            RegReal_RotatingMBDCmd.u16Ref = 0;
        }

        RegReal_TelescopicMBDCmd.u16EN = 1;
        RegReal_TelescopicMBDCmd.u16SN = 1;
        RegReal_TelescopicMBDCmd.u16Cmd = 6;
        if ( Reg_RCCtrlCmdFb.u8TelescopicCmd == 1 )//伸出
        {
            RegReal_TelescopicMBDCmd.u16Ref = 1;
        }
        else if (Reg_RCCtrlCmdFb.u8TelescopicCmd == 0xFF ) 
        {
            RegReal_TelescopicMBDCmd.u16Ref = 0xFFFF;
        }
        else
        {
            RegReal_TelescopicMBDCmd.u16Ref = 0;
        }

        RegReal_HuggingMBDCmd.u16EN = 1;
        RegReal_HuggingMBDCmd.u16SN = 1;
        RegReal_HuggingMBDCmd.u16Cmd = 6;
        if ( Reg_RCCtrlCmdFb.u8HuggingCmd == 1 ) //环抱
        {
            RegReal_HuggingMBDCmd.u16Ref = 1;
        }
        else if ( Reg_RCCtrlCmdFb.u8HuggingCmd == 0xFF )
        {
            RegReal_HuggingMBDCmd.u16Ref = 0xFFFF;
        }
        else
        {
            RegReal_HuggingMBDCmd.u16Ref = 0;
        }

        RegReal_GlandMBDCmd.u16EN = 1;
        RegReal_GlandMBDCmd.u16SN = 1;
        RegReal_GlandMBDCmd.u16Cmd = 6;
        if ( Reg_RCCtrlCmdFb.u8GlandCmd == 1 ) //抬压
        {
            RegReal_GlandMBDCmd.u16Ref = 1;
        }
        else if ( Reg_RCCtrlCmdFb.u8GlandCmd == 0xFF )
        {
            RegReal_GlandMBDCmd.u16Ref = 0xFFFF;
        }
        else
        {
            RegReal_GlandMBDCmd.u16Ref = 0;
        }
    } else {
        memset ( &Reg_AGVCtrlCmd1, 0, sizeof ( Reg_AGVCtrlCmd1 ) );
        memset ( &Reg_AGVCtrlCmd2, 0, sizeof ( Reg_AGVCtrlCmd2 ) );
        RegReal_GlandMBDCmd.u16EN = 0;
        RegReal_HuggingMBDCmd.u16EN = 0;
        RegReal_TelescopicMBDCmd.u16EN = 0;
        RegReal_RotatingMBDCmd.u16EN = 0;
    }
}