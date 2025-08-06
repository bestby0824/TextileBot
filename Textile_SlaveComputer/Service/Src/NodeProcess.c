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
#include "NodeProcess.h"
#include "CAN_Node.h"
#include "string.h"
#include "IMU_Com.h"
#include "main.h"
#include "VCU_Ctrl.h"
#include "StateCtrl.h"
#include "Host_Com.h"
#include "Monitor.h"
#include "IMU_Com.h"


//-------------------- local definitions ------------------------------------
#define IMUCom_FullLife  500                              //IMU通信最大心跳
#define SteeringCom_FullLife  500                              //转向电机板通信最大心跳
#define SolenoidCom_FullLife  1000

//-------------------- private data -----------------------------------------

//-------------------- private functions declare ----------------------------

//-------------------- public data ------------------------------------------
UserFrame2_t Tx2_frame;
UserFrame2_t Rx2_frame;
uint32_t u32CanTask2Cnt = 0;
int16_t  RadarLife = -1, DrawWireLife = -1;
BootRxStruct BootRxHandle;  //BOOT反馈句柄
RegDef_ValveLiftCmd RegReal_ValveLiftCmd;  //寄存器0x100
RegDef_ValveClawCmd RegReal_ValveClawCmd;  //寄存器0x101
RegDef_ValveBowRiseCmd RegReal_ValveBowRiseCmd;  //寄存器0x102
RegDef_ValveRotateCmd RegReal_ValveRotateCmd;  //寄存器0x103
RegDef_ValveSideShiftCmd RegReal_ValveSideShiftCmd;   //寄存器0x104
RegDef_ValveLiftRaderCmd RegReal_ValveLiftRaderCmd;  //寄存器0x105
RegDef_ValveSensorCaliCmd RegReal_ValveSensorCaliCmd;  //寄存器0x106
RegDef_ValveFaultClearCmd RegReal_ValveFaultClearCmd;  //寄存器0x107
RegDef_ValveFaultEnableCmd RegReal_ValveFaultEnableCmd;  //寄存器0x108
RegDef_ValveCtrlState RegReal_ValveCtrlState;  //寄存器0x109
RegDef_SteerPosCtrlCmd RegReal_SteerPosCtrlCmd = { 0, 0, 0, 32767 };  //寄存器0x180
RegDef_SteerSensorCaliCmd RegReal_SteerSensorCaliCmd;  //寄存器0x181
RegDef_SteerFaultClearCmd RegReal_SteerFaultClearCmd;  //寄存器0x182
RegDef_SteerFaultEnableCmd RegReal_SteerFaultEnableCmd;  //寄存器0x183
RegDef_SteerCtrlState RegReal_SteerCtrlState;  //寄存器0x184

RegDef_ValveSensorPos1FB RegReal_ValveSensorPos1FB;     //寄存器0x200
RegDef_ValveSensorPos2FB RegReal_ValveSensorPos2FB;     //寄存器0x201
RegDef_ValveCmd1SNFB RegReal_ValveCmd1SNFB;             //寄存器0x202
RegDef_ValveCmd2SNFB RegReal_ValveCmd2SNFB;             //寄存器0x203
RegDef_ValveCtrlMode1FB RegReal_ValveCtrlMode1FB;       //寄存器0x204
RegDef_ValveCtrlMode2FB RegReal_ValveCtrlMode2FB;       //寄存器0x205
RegDef_ValveState1FB RegReal_ValveState1FB;             //寄存器0x206
RegDef_ValvePressureFB RegReal_ValvePressureFB;         //寄存器0x207
RegDef_ValveFaultCodeFB RegReal_ValveFaultCodeFB;       //寄存器0x208
RegDef_ValveCaliStateFB RegReal_ValveCaliStateFB;       //寄存器0x209
RegDef_ValveState2FB RegReal_ValveState2FB;             //寄存器0x20A
RegDef_SteerPosCtrlFB RegReal_SteerPosCtrlFB;       //寄存器0x300
RegDef_SteerSensorDataFB RegReal_SteerSensorDataFB; //寄存器0x301
RegDef_SteerFaultCodeFB RegReal_SteerFaultCodeFB;  //寄存器0x302
RegDef_SteerSensorCaliFB RegReal_SteerSensorCaliFB;  //寄存器0x303
RegDef_SteerFaultCtrlFB RegReal_SteerFaultCtrlFB;  //寄存器0x304

//-------------------- public data ------------------------------------------
uint8_t u8BootFB_Flag = 0;
/*Textiles*/
/*控制*/
RegDef_MotorMBDCmd RegReal_RotatingMBDCmd;   //0x110
RegDef_MotorMBDCmd RegReal_TelescopicMBDCmd;//0x120
RegDef_MotorMBDCmd RegReal_HuggingMBDCmd;//0x130
RegDef_MotorMBDCmd RegReal_GlandMBDCmd;//0x140
/*标定*/
RegDef_MotorMBDCalaCmd RegReal_RotatingMBDCalaCmd;//0x111
RegDef_MotorMBDCalaCmd RegReal_TelescopicMBDCalaCmd;//0x121
RegDef_MotorMBDCalaCmd RegReal_HuggingMBDCalaCmd;//0x131
RegDef_MotorMBDCalaCmd RegReal_GlandMBDCalaCmd;//0x141
/*故障清除*/
RegDef_MotorMBDFaultClearCmd RegReal_RotatingMBDFaultClearCmd;//0x112
RegDef_MotorMBDFaultClearCmd RegReal_TelescopicMBDFaultClearCmd;//0x122
RegDef_MotorMBDFaultClearCmd RegReal_HuggingMBDFaultClearCmd;//0x132
RegDef_MotorMBDFaultClearCmd RegReal_GlandMBDFaultClearCmd;//0x142
/*参数管理*/
RegDef_MotorMBDParamCmd RegReal_RotatingMBDParamCmd;//0x113
RegDef_MotorMBDParamCmd RegReal_TelescopicMBDParamCmd;//0x123
RegDef_MotorMBDParamCmd RegReal_HuggingMBDParamCmd;//0x133
RegDef_MotorMBDParamCmd RegReal_GlandMBDParamCmd;//0x143
/*心跳*/
RegDef_MotorMBDWatchDogCmd RegReal_RotatingMBDWatchDogCmd;//0x114
RegDef_MotorMBDWatchDogCmd RegReal_TelescopicMBDWatchDogCmd;//0x124
RegDef_MotorMBDWatchDogCmd RegReal_HuggingMBDWatchDogCmd;//0x134
RegDef_MotorMBDWatchDogCmd RegReal_GlandMBDWatchDogCmd;//0x144
/*复位*/
RegDef_MotorMBDResetCmd RegReal_RotatingMBDResetCmd;//0x115
RegDef_MotorMBDResetCmd RegReal_TelescopicMBDResetCmd;//0x125
RegDef_MotorMBDResetCmd RegReal_HuggingMBDResetCmd;//0x135
RegDef_MotorMBDResetCmd RegReal_GlandMBDResetCmd;//0x145
/*控制ack*/
RegDef_RotatingMBDCtrlFB RegReal_RotatingMBDCtrlFB;//0x200
RegDef_TelescopicMBDCtrlFB RegReal_TelescopicMBDCtrlFB;//0x300
RegDef_HuggingMBDCtrlFB RegReal_HuggingMBDCtrlFB;//0x400
RegDef_GlandMBDCtrlFB RegReal_GlandMBDCtrlFB;//0x500
/*标定ack*/
RegDef_RotatingMBDCalaFB RegReal_RotatingMBDCalaFB;//0x201
RegDef_TelescopicMBDCalaFB RegReal_TelescopicMBDCalaFB;//0x301
RegDef_HuggingMBDCalaFB RegReal_HuggingMBDCalaFB;//0x401
RegDef_GlandMBDCalaFB RegReal_GlandMBDCalaFB;//0x501
/*故障清除ack*/
RegDef_RotatingMBDFaultClearFB RegReal_RotatingMBDFaultClearFB;//0x202
RegDef_TelescopicMBDFaultClearFB RegReal_TelescopicMBDFaultClearFB;//0x302
RegDef_HuggingMBDFaultClearFB RegReal_HuggingMBDFaultClearFB;//0x402
RegDef_GlandMBDFaultClearFB RegReal_GlandMBDFaultClearFB;//0x502
/*参数管理ack*/
RegDef_RotatingMBDParamFB RegReal_RotatingMBDParamFB;//0x203
RegDef_TelescopicMBDParamFB RegReal_TelescopicMBDParamFB;//0x303
RegDef_HuggingMBDParamFB RegReal_HuggingMBDParamFB;//0x403
RegDef_GlandMBDParamFB RegReal_GlandMBDParamFB;//0x503
/*心跳ack*/
RegDef_RotatingMBDWatchDogFB RegReal_RotatingMBDWatchDogFB;//0x204
RegDef_TelescopicMBDWatchDogFB RegReal_TelescopicMBDWatchDogFB;//0x304
RegDef_HuggingMBDWatchDogFB RegReal_HuggingMBDWatchDogFB;//0x404
RegDef_GlandMBDWatchDogFB RegReal_GlandMBDWatchDogFB;//0x504
/*复位ack*/
RegDef_RotatingMBDResetFB RegReal_RotatingMBDResetFB;//0x205
RegDef_TelescopicMBDResetFB RegReal_TelescopicMBDResetFB;//0x305
RegDef_HuggingMBDResetFB RegReal_HuggingMBDResetFB;//0x405
RegDef_GlandMBDResetFB RegReal_GlandMBDResetFB;//0x505
/*故障警告ack*/
RegDef_RotatingMBDFaultStaFB RegReal_RotatingMBDFaultStaFB;//0x210
RegDef_TelescopicMBDFaultStaFB RegReal_TelescopicMBDFaultStaFB;//0x310
RegDef_HuggingMBDFaultStaFB RegReal_HuggingMBDFaultStaFB;//0x410
RegDef_GlandMBDFaultStaFB RegReal_GlandMBDFaultStaFB;//0x510
/*传感器状态ack*/
RegDef_RotatingMBDSensorStaFB RegReal_RotatingMBDSensorStaFB;//0x211
RegDef_TelescopicMBDSensorStaFB RegReal_TelescopicMBDSensorStaFB;//0x311
RegDef_HuggingMBDSensorStaFB RegReal_HuggingMBDSensorStaFB;//0x411
RegDef_GlandMBDSensorStaFB RegReal_GlandMBDSensorStaFB;//0x511
/*运行状态1ack*/
RegDef_RotatingMBDRunningSta1FB RegReal_RotatingMBDRunningSta1FB;//0x212
RegDef_TelescopicMBDRunningSta1FB RegReal_TelescopicMBDRunningSta1FB;//0x312
RegDef_HuggingMBDRunningSta1FB RegReal_HuggingMBDRunningSta1FB;//0x412
RegDef_GlandMBDRunningSta1FB RegReal_GlandMBDRunningSta1FB;//0x512
int16_t m_nRotatingMBLife = -1, m_nTelescopicMBLife = -1, m_nHuggingMBLife = -1, m_nGlandMBLife = -1;
RegDef_DrawWireSensor Reg_DrawWireSensor;
RegDef_DrawWireSensor Reg_DrawWireSensorFb;
RegDef_RotatingMBDRunningSta1FB RegReal_RotatingMBDRunningSta2FB;//0x213

RegDef_TelescopicMBDRelayCmd RegReal_TelescopicMBDRelayCmd;
RegDef_TelescopicMBDRelayCmdFB RegReal_TelescopicMBDRelayCmdFB;
uint8_t RotatingFaultClearFlag = 0, TelescopicFaultClearFlag = 0, HuggingFaultClearFlag = 0, GlandFaultClearFlag = 0;
uint8_t RotatingMotorCalaFlag = 0, TelescopicMotorCalaFlag = 0, HuggingMotorCalaFlag = 0, GlandMotorCalaFlag = 0;
uint8_t RotatingMotorResetFlag = 0, TelescopicMotorResetFlag = 0, HuggingMotorResetFlag = 0, GlandMotorResetFlag = 0;
uint8_t RotatingWRParamCmdFlag = 0, TelescopicWRParamCmdFlag = 0, HuggingWRParamCmdFlag = 0, GlandWRParamCmdFlag = 0;
RegDef_RadarSend  RegReal_RadarSend;
RegDef_RadarRev RegReal_RadarRev;
//-------------------- public functions -------------------------------------
void CanTask2Process ( void );
/*
* @brief       故障清除函数
* @param       无
* @retval      无
*
*/
int8_t  FualtClear ( void )      //故障清除
{
    static uint16_t BoardIDOpt, RST_MCUStaFlag;

    if ( InputIOInfor.RST_MCUSta )
    {
        BoardIDOpt = 0;
        RST_MCUStaFlag = 1;
    }
    if ( HostFaultClear == 1 )
    {
        BoardIDOpt = m_sFaultClear.u16BoardID;
    }
    else if ( RST_MCUStaFlag == 1 )
    {
        BoardIDOpt++;
    }
    if ( RST_MCUStaFlag == 1 || HostFaultClear == 1 )
    {
        switch ( BoardIDOpt )
        {
        case SlaveBoard:
        {
            FaultCtrl.all = 0;
            STOP_0_Ctrl(OFF);
            Reg_AGVTurnCtrlCmd1.u16CtrlWord.all = 0x86;
        }
        break;
        case RotatingBoard:
        {
            RegReal_RotatingMBDFaultClearCmd.u16EN = 1;
//        RegReal_RotatingMBDFaultClearCmd.u16SN= m_sFaultClear.u16SN;
            RegReal_RotatingMBDFaultClearCmd.u32FaultBit = 0xFFFFFFFF;
//        Can2Task_Send(Reg_RotatingMBDFaultClearCmd);
            RotatingFaultClearFlag = 1;
        }
        break;
        case TelescopicBoard:
        {
            RegReal_TelescopicMBDFaultClearCmd.u16EN = 1;
//        RegReal_TelescopicMBDFaultClearCmd.u16SN= m_sFaultClear.u16SN;
            RegReal_TelescopicMBDFaultClearCmd.u32FaultBit = 0xFFFFFFFF;
//        Can2Task_Send(Reg_TelescopicMBDFaultClearCmd);
            TelescopicFaultClearFlag = 1;
        }
        break;
        case HuggingBoard:
        {
            RegReal_HuggingMBDFaultClearCmd.u16EN = 1;
//        RegReal_HuggingMBDFaultClearCmd.u16SN= m_sFaultClear.u16SN;
            RegReal_HuggingMBDFaultClearCmd.u32FaultBit = 0xFFFFFFFF;
//        Can2Task_Send(Reg_HuggingMBDFaultClearCmd);
            HuggingFaultClearFlag = 1;
        }
        break;
        case GlandBoard:
        {
            RegReal_GlandMBDFaultClearCmd.u16EN = 1;
//        RegReal_GlandMBDFaultClearCmd.u16SN= m_sFaultClear.u16SN;
            RegReal_GlandMBDFaultClearCmd.u32FaultBit = 0xFFFFFFFF;
//        Can2Task_Send(Reg_GlandMBDFaultClearCmd);
            GlandFaultClearFlag = 1;
        }
        break;
        default:
            RST_MCUStaFlag = 0;
            break;
            HostFaultClear = 0;
        }
    }
    return Err_None;
}
#if 0
/*
* @brief       电机零位标定监控函数
* @param       无
* @retval      无
*
*/
int8_t MotorCalaCheck ( void )
{
    if ( 0 == m_MotorCala.u16SN )
        return Err_NotData;
    m_sCommandAck.unSteerCalaAck = m_MotorCala.u16SN;
    if ( m_MotorCala.u16Cmd == 1 )
    {
        switch ( m_MotorCala.u16BoardID )
        {
        case RotatingBoard:
        {
            RegReal_RotatingMBDCalaCmd.u16Cmd = 1;//电机零点标定
            RegReal_RotatingMBDCalaCmd.u16SN = m_MotorCala.u16SN;
            RegReal_RotatingMBDCalaCmd.u16Cmd = 1;
            RotatingMotorCalaFlag = 1;
//            Can2Task_Send ( Reg_RotatingMBDCalaCmd );
        }
        break;
        case TelescopicBoard:
        {
            RegReal_TelescopicMBDCalaCmd.u16Cmd = 1;//
            RegReal_TelescopicMBDCalaCmd.u16SN = m_MotorCala.u16SN;
            RegReal_TelescopicMBDCalaCmd.u16Cmd = 1;
            TelescopicMotorCalaFlag = 1;
//            Can2Task_Send ( Reg_TelescopicMBDCalaCmd );
        }
        break;
        case HuggingBoard:
        {
            RegReal_HuggingMBDCalaCmd.u16Cmd = 1;//电机零点标定
            RegReal_HuggingMBDCalaCmd.u16SN = m_MotorCala.u16SN;
            RegReal_HuggingMBDCalaCmd.u16Cmd = 1;
            HuggingMotorCalaFlag = 1;
//            Can2Task_Send ( Reg_HuggingMBDCalaCmd );
        }
        break;
        case GlandBoard:
        {
            RegReal_GlandMBDCalaCmd.u16Cmd = 1;//电机零点标定
            RegReal_GlandMBDCalaCmd.u16SN = m_MotorCala.u16SN;
            RegReal_GlandMBDCalaCmd.u16Cmd = 1;
            GlandMotorCalaFlag = 1;
//            Can2Task_Send ( Reg_GlandMBDCalaCmd );
        }
        break;
        }
    }
    return Err_None;
}
#endif
static void SoftReset ( void )
{
    __disable_irq();
    __NVIC_SystemReset();
}
/*
* @brief       复位监测函数
* @param       无
* @retval      无
*
*/
int8_t ResetCheck ( void )
{
    if ( 0 == m_Reset.u16SN )
        return Err_NotData;
    m_sCommandAck.unResetAck = m_Reset.u16SN;
    if ( m_Reset.u16Delay < 10 )
    {
        m_Reset.u16Delay = 10;
    }
    m_Reset.u16SN = 0;
    switch ( m_Reset.u16BoardID )
    {
    case SlaveBoard:
    {
        if ( m_Reset.u16Delay > 1 )
        {
            m_Reset.u16Delay--;
        }
        if ( m_Reset.u16Delay == 1 )
        {
            SoftReset();
        }
    }
    break;

    case RotatingBoard:
    {
        RegReal_RotatingMBDResetCmd.u16SN = m_Reset.u16SN;
        RegReal_RotatingMBDResetCmd.u16Delay = m_Reset.u16Delay;
        RotatingMotorResetFlag = 1;
//        Can2Task_Send ( Reg_RotatingMBDResetCmd );
    }
    break;
    case TelescopicBoard:
    {
        RegReal_TelescopicMBDResetCmd.u16SN = m_Reset.u16SN;
        RegReal_TelescopicMBDResetCmd.u16Delay = m_Reset.u16Delay;
        TelescopicMotorResetFlag = 1;
//        Can2Task_Send ( Reg_TelescopicMBDResetCmd );
    }
    break;
    case HuggingBoard:
    {
        RegReal_HuggingMBDResetCmd.u16SN = m_Reset.u16SN;
        RegReal_HuggingMBDResetCmd.u16Delay = m_Reset.u16Delay;
        HuggingMotorResetFlag = 1;
//        Can2Task_Send ( Reg_HuggingMBDResetCmd );
    }
    break;
    case GlandBoard:
    {
        RegReal_GlandMBDResetCmd.u16SN = m_Reset.u16SN;
        RegReal_GlandMBDResetCmd.u16Delay = m_Reset.u16Delay;
        GlandMotorResetFlag = 1;
//        Can2Task_Send ( Reg_GlandMBDResetCmd );
    }
    break;
    }
    return Err_None;
}
/**
 * @brief       读写参数到E2Prom
 * @param       无
 * @retval      周期1K
 */
void ParamRead_Write ( void )
{
    if ( g_u16ParamStep == 1 )
    {
        m_ParamMngCmdAck.u16AddrAck = m_ParamMngCmd.u16Addr;
        m_ParamMngCmdAck.u16SN = m_ParamMngCmd.u16SN;
        m_ParamMngCmdAck.u16CmdAck = m_ParamMngCmd.u16Cmd;
        m_ParamMngCmdAck.u16BoardIDAck = m_ParamMngCmd.u16BoardID;
        switch ( m_ParamMngCmd.u16BoardID )
        {
        case SlaveBoard:
        {

            if ( m_ParamMngCmd.u16Cmd == Para_READ )
            {
                if ( m_ParamMngCmd.u16Addr < ( uint16_t* ) & ( ParamHandle_Control.Reg_CRC ) - ( uint16_t* ) & ( ParamHandle_Control.Reg_Tab_Ver ) )
                {
                    m_ParamMngCmdAck.u16ValueAck = * ( ( uint16_t* ) & ( ParamHandle_Control.Reg_Tab_Ver ) + m_ParamMngCmd.u16Addr );
                }
                else
                {
                    m_ParamMngCmdAck.u16ValueAck = 0xFFFF;
                }
            }
            else if ( m_ParamMngCmd.u16Cmd == Para_WRITE )
            {
                if ( m_ParamMngCmd.u16Addr > 15 )
                {
                    * ( ( uint16_t* ) & ( ParamHandle_Control.Reg_Tab_Ver ) + m_ParamMngCmd.u16Addr ) = m_ParamMngCmd.u16Value;
                    m_ParamMngCmdAck.u16ValueAck = * ( ( uint16_t* ) & ( ParamHandle_Control.Reg_Tab_Ver ) + m_ParamMngCmd.u16Addr );
                }
            }
            if ( Param2Eeprom () )
            {
                FaultCtrl.bit.EepromWriteErr = 1;
            }
            g_u16ParamStep = 2;
        }
        break;
        case RotatingBoard:
        {
            RegReal_RotatingMBDParamCmd.u16Addr = m_ParamMngCmd.u16Addr;
            RegReal_RotatingMBDParamCmd.u16SN = m_ParamMngCmd.u16SN;
            RegReal_RotatingMBDParamCmd.mod = m_ParamMngCmd.u16Cmd;
            RegReal_RotatingMBDParamCmd.u16Value = m_ParamMngCmd.u16Value;
            RotatingWRParamCmdFlag = 1;
//            Can2Task_Send(Reg_RotatingMBDParamCmd);
            m_ParamMngCmdAck.u16ValueAck = RegReal_RotatingMBDParamFB.u16Value;
        }
        break;
        case TelescopicBoard:
        {
            RegReal_TelescopicMBDParamCmd.u16Addr = m_ParamMngCmd.u16Addr;
            RegReal_TelescopicMBDParamCmd.u16SN = m_ParamMngCmd.u16SN;
            RegReal_TelescopicMBDParamCmd.mod = m_ParamMngCmd.u16Cmd;
            RegReal_TelescopicMBDParamCmd.u16Value = m_ParamMngCmd.u16Value;
            TelescopicWRParamCmdFlag = 1;
//            Can2Task_Send(Reg_TelescopicMBDParamCmd);
            m_ParamMngCmdAck.u16ValueAck = RegReal_TelescopicMBDParamFB.u16Value;
        }
        break;
        case HuggingBoard:
        {
            RegReal_HuggingMBDParamCmd.u16Addr = m_ParamMngCmd.u16Addr;
            RegReal_HuggingMBDParamCmd.u16SN = m_ParamMngCmd.u16SN;
            RegReal_HuggingMBDParamCmd.mod = m_ParamMngCmd.u16Cmd;
            RegReal_HuggingMBDParamCmd.u16Value = m_ParamMngCmd.u16Value;
            HuggingWRParamCmdFlag = 1;
//            Can2Task_Send(Reg_HuggingMBDParamCmd);
            m_ParamMngCmdAck.u16ValueAck = RegReal_HuggingMBDParamFB.u16Value;
        }
        break;
        case GlandBoard:
        {
            RegReal_GlandMBDParamCmd.u16Addr = m_ParamMngCmd.u16Addr;
            RegReal_GlandMBDParamCmd.u16SN = m_ParamMngCmd.u16SN;
            RegReal_GlandMBDParamCmd.mod = m_ParamMngCmd.u16Cmd;
            RegReal_GlandMBDParamCmd.u16Value = m_ParamMngCmd.u16Value;
            GlandWRParamCmdFlag = 1;
//            Can2Task_Send(Reg_GlandMBDParamCmd);
            m_ParamMngCmdAck.u16ValueAck = RegReal_GlandMBDParamFB.u16Value;
        }
        break;

        }
        if ( g_u16ParamStep == 2 )
        {
            g_u16ParamStep = 0;
            HostCom_ParamSendReport ( &m_ParamMngCmdAck );
        }
    }
}
/**
 * @brief       Node对应寄存器状态CAN2发送给液压板、转向板等附属外设
 * @param       寄存器ID
 * @retval      无
 */
int Can2Task_Send ( uint32_t Cmd_ID )
{
    static uint8_t res = 0;
    Tx2_frame.ID = Cmd_ID;
    Tx2_frame.Len = 8;

    if ( ( u8BootFlg_Solenoid == 1 ) || ( u8BootFlg_SteeringWheel == 1 ) ) return res;

    switch ( Cmd_ID )
    {
    /*控制*/
    case Reg_RotatingMBDCmd://0x110
    {
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_RotatingMBDCmd, Tx2_frame.Len );
    }
    break;
    case Reg_TelescopicMBDCmd://0x120
    {
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_TelescopicMBDCmd, Tx2_frame.Len );
    }
    break;
    case Reg_HuggingMBDCmd://0x130
    {
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_HuggingMBDCmd, Tx2_frame.Len );
    }
    break;
    case Reg_GlandMBDCmd://0x140
    {
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_GlandMBDCmd, Tx2_frame.Len );
    }
    break;
    /*心跳*/
    case Reg_RotatingMBDWatchDogCmd://
    {
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_RotatingMBDWatchDogCmd, Tx2_frame.Len );
    }
    break;
    case Reg_TelescopicMBDWatchDogCmd://
    {
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_TelescopicMBDWatchDogCmd, Tx2_frame.Len );
    }
    break;
    case Reg_HuggingMBDWatchDogCmd://
    {
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_HuggingMBDWatchDogCmd, Tx2_frame.Len );
    }
    break;
    case Reg_GlandMBDWatchDogCmd://
    {
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_GlandMBDWatchDogCmd, Tx2_frame.Len );
    }
    break;
    /*零点标定*/
    case Reg_RotatingMBDCalaCmd://
    {
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_RotatingMBDCalaCmd, Tx2_frame.Len );
    }
    break;
    case Reg_TelescopicMBDCalaCmd://
    {
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_TelescopicMBDCalaCmd, Tx2_frame.Len );
    }
    break;

    case Reg_HuggingMBDCalaCmd://
    {
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_HuggingMBDCalaCmd, Tx2_frame.Len );
    }
    break;
    case Reg_GlandMBDCalaCmd://
    {
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_GlandMBDCalaCmd, Tx2_frame.Len );
    }
    break;
    /*故障清除*/
    case Reg_RotatingMBDFaultClearCmd://
    {
        RegReal_RotatingMBDFaultClearCmd.u16SN = u32CanTask2Cnt;
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_RotatingMBDFaultClearCmd, Tx2_frame.Len );
    }
    break;
    case Reg_TelescopicMBDFaultClearCmd://
    {
        //RegReal_SteerCtrlState.u16CtrlMode = m_eCtrlMode;
        RegReal_TelescopicMBDFaultClearCmd.u16SN = u32CanTask2Cnt;
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_TelescopicMBDFaultClearCmd, Tx2_frame.Len );
    }
    break;
    case Reg_HuggingMBDFaultClearCmd://
    {
        RegReal_HuggingMBDFaultClearCmd.u16SN = u32CanTask2Cnt;
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_HuggingMBDFaultClearCmd, Tx2_frame.Len );
    }
    break;
    case Reg_GlandMBDFaultClearCmd: //
    {
        RegReal_GlandMBDFaultClearCmd.u16SN = u32CanTask2Cnt;
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_GlandMBDFaultClearCmd, Tx2_frame.Len );
    }
    break;
    /*参数管理*/
    case Reg_RotatingMBDParamCmd: //
    {
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_RotatingMBDParamCmd, Tx2_frame.Len );

    }
    break;
    case Reg_TelescopicMBDParamCmd: //
    {
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_TelescopicMBDParamCmd, Tx2_frame.Len );
    }
    break;
    case Reg_HuggingMBDParamCmd: //
    {
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_HuggingMBDParamCmd, Tx2_frame.Len );
    }
    break;
    case Reg_GlandMBDParamCmd: //
    {
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_GlandMBDParamCmd, Tx2_frame.Len );
    }
    break;
    /*复位*/
    case Reg_RotatingMBDResetCmd: //
    {
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_RotatingMBDResetCmd, Tx2_frame.Len );
    }
    break;
    case Reg_TelescopicMBDResetCmd: //
    {
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_TelescopicMBDResetCmd, Tx2_frame.Len );
    }
    break;
    case Reg_HuggingMBDResetCmd: //
    {
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_HuggingMBDResetCmd, Tx2_frame.Len );
    }
    break;
    case Reg_GlandMBDResetCmd: //
    {
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_GlandMBDResetCmd, Tx2_frame.Len );
    }
    break;
    case Step_TelescopicMBDRelayCmd: //
    {
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_TelescopicMBDRelayCmd, Tx2_frame.Len );
    }
    break;
    case Reg_RadarSend: //
    {
        memcpy ( ( uint8_t * ) &Tx2_frame.Buff, ( uint8_t * ) &RegReal_RadarSend, Tx2_frame.Len );
    }
    break;
    default:
        break;
    }

    res = Can_Node_send_msg ( Tx2_frame.ID, ( uint8_t * ) &Tx2_frame.Buff, Tx2_frame.Len );

    return res;
}
/**
 * @brief       接收CAN1上数据并分类填充到VCU结构体
 * @param       无
 * @retval      无
 */
void Can_Node_Receive ( void )
{
    Rx2_frame.ID = Can_Node_receive_msg ( Rx2_frame.Buff );
    Rx2_frame.Len = 8;
    if ( Rx2_frame.ID == 0 ) return;

    switch ( Rx2_frame.ID )
    {
    case Reg_BootModeFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &BootRxHandle, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        u8BootFB_Flag = 1;
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_IMUState:
    {
        __disable_irq();         //关闭总中断
        memcpy ( &g_bIMURxBuf, ( uint8_t * ) &Rx2_frame.Buff, 8 );
        __enable_irq();          // 开启总中断
        IMULife = IMUCom_FullLife;                  //收到IMU数据后将心跳等待时间放到最大
        u8IMUDataRxFlag = 1;
    }
    break;
    /*控制ack*/
    case Reg_RotatingMBDCtrlFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_RotatingMBDCtrlFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_TelescopicMBDCtrlFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_TelescopicMBDCtrlFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_HuggingMBDCtrlFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_HuggingMBDCtrlFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_GlandMBDCtrlFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_GlandMBDCtrlFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    /*零点标定ack*/
    case Reg_RotatingMBDCalaFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_RotatingMBDCalaFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_TelescopicMBDCalaFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_TelescopicMBDCalaFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_HuggingMBDCalaFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_HuggingMBDCalaFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_GlandMBDCalaFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_GlandMBDCalaFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    /*故障清除ack*/
    case Reg_RotatingMBDFaultClearFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_RotatingMBDFaultClearFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_TelescopicMBDFaultClearFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_TelescopicMBDFaultClearFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_HuggingMBDFaultClearFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_HuggingMBDFaultClearFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_GlandMBDFaultClearFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_GlandMBDFaultClearFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    /*参数管理ack*/
    case Reg_RotatingMBDParamFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_RotatingMBDParamFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
        g_u16ParamStep = 2;
    }
    break;
    case Reg_TelescopicMBDParamFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_TelescopicMBDParamFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
        g_u16ParamStep = 2;
    }
    break;
    case Reg_HuggingMBDParamFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_HuggingMBDParamFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
        g_u16ParamStep = 2;
    }
    break;
    case Reg_GlandMBDParamFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_GlandMBDParamFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
        g_u16ParamStep = 2;
    }
    break;
    /*心跳*/
    case Reg_RotatingMBDWatchDogFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_RotatingMBDWatchDogFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        m_nRotatingMBLife = ParamHandle_Control.Reg_BD1DogTimeOut;
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_TelescopicMBDWatchDogFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_TelescopicMBDWatchDogFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        m_nTelescopicMBLife = ParamHandle_Control.Reg_BD2DogTimeOut;
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_HuggingMBDWatchDogFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_HuggingMBDWatchDogFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        m_nHuggingMBLife = ParamHandle_Control.Reg_BD3DogTimeOut;
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_GlandMBDWatchDogFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_GlandMBDWatchDogFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        m_nGlandMBLife = ParamHandle_Control.Reg_BD4DogTimeOut;
        __enable_irq();          // 开启总中断
    }
    break;
    /*复位*/
    case Reg_RotatingMBDResetFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_RotatingMBDResetFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_TelescopicMBDResetFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_TelescopicMBDResetFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_HuggingMBDResetFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_HuggingMBDResetFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_GlandMBDResetFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_GlandMBDResetFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    /*故障状态*/
    case Reg_RotatingMBDFaultStaFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_RotatingMBDFaultStaFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_TelescopicMBDFaultStaFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_TelescopicMBDFaultStaFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_HuggingMBDFaultStaFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_HuggingMBDFaultStaFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_GlandMBDFaultStaFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_GlandMBDFaultStaFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    /*传感器状态*/
    case Reg_RotatingMBDSensorStaFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_RotatingMBDSensorStaFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_TelescopicMBDSensorStaFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_TelescopicMBDSensorStaFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_HuggingMBDSensorStaFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_HuggingMBDFaultStaFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_GlandMBDSensorStaFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_GlandMBDFaultStaFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    /*运行状态1*/
    case Reg_RotatingMBDRunningSta1FB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_RotatingMBDRunningSta1FB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_TelescopicMBDRunningSta1FB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_TelescopicMBDRunningSta1FB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_HuggingMBDRunningSta1FB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_HuggingMBDRunningSta1FB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_GlandMBDRunningSta1FB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_GlandMBDRunningSta1FB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_RotatingMBDRunningSta2FB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_RotatingMBDRunningSta2FB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    case Step_DrawWireSensor:
    {
        __disable_irq();         //关闭总中断
        Reg_DrawWireSensorFb.u8DataLen = 7;
        DrawWireLife = ParamHandle_Control.Reg_DrawWireDogTimeOut;
        memcpy ( ( uint8_t * ) &Reg_DrawWireSensorFb, ( uint8_t * ) &Rx2_frame.Buff, Reg_DrawWireSensorFb.u8DataLen );
        __enable_irq();          // 开启总中断
    }
    break;
    case Reg_RadarRev:
    {
        __disable_irq();         //关闭总中断
        RadarLife = ParamHandle_Control.Reg_RadarDogTimeOut;
        memcpy ( ( uint8_t * ) &RegReal_RadarRev, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    case Step_TelescopicMBDRelayCmdFB:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &RegReal_TelescopicMBDRelayCmdFB, ( uint8_t * ) &Rx2_frame.Buff, Rx2_frame.Len );
        __enable_irq();          // 开启总中断
    }
    break;
    default:
        break;
    }
}
/**
 * @brief       VCU对应寄存器状态发送，1KHZ处理(IMU、转向驱动板、液压驱动板)
 * @param       无
 * @retval      无
 * @notes       500+6*50+2*50=900
 */
void CanTask2Process ( void )
{

    static uint16_t u16CanTask2Step = 0;
    static uint16_t u16ValveTask2Step = ValveSensorCaliCmd, u16SteerTask2Step = SteerSensorCaliCmd;

    u32CanTask2Cnt++;
    if ( ( u32CanTask2Cnt % 20 ) == 0 )  //2ms一次，转向位置控制指令
    {
        //u16CanTask2Step = SteerPosCtrlCmd;
        if ( RotatingMotorCalaFlag == 1 )
        {
            u16CanTask2Step = Reg_RotatingMBDCalaCmd;
            RotatingMotorCalaFlag = 0;
        }
        else if ( TelescopicMotorCalaFlag == 1 )
        {
            u16CanTask2Step = Reg_TelescopicMBDCalaCmd;
            TelescopicMotorCalaFlag = 0;
        }
        else  if ( HuggingMotorCalaFlag == 1 )
        {
            u16CanTask2Step = Reg_HuggingMBDCalaCmd;
            HuggingMotorCalaFlag = 0;
        }
        else if ( GlandMotorCalaFlag == 1 )
        {
            u16CanTask2Step = Reg_GlandMBDCalaCmd;
            GlandMotorCalaFlag = 0;
        }
        else
        {
            u16CanTask2Step = Step_TelescopicMBDRelayCmd;
        }
    }
    else if ( ( u32CanTask2Cnt % 20 ) == 1 )  //20ms一次，旋转控制指令
    {
        u16CanTask2Step = Reg_RotatingMBDCmd;
    }
    else if ( ( u32CanTask2Cnt % 20 ) == 3 )  //20ms一次，拉伸控制指令
    {
        u16CanTask2Step = Reg_TelescopicMBDCmd;
    }
    else if ( ( u32CanTask2Cnt % 20 ) == 5 )  //20ms一次，环抱控制指令
    {
        u16CanTask2Step = Reg_HuggingMBDCmd;
    }
    else if ( ( u32CanTask2Cnt % 20 ) == 7 )  //20ms一次，压盖控制指令
    {
        u16CanTask2Step = Reg_GlandMBDCmd;
    }
    else if ( ( u32CanTask2Cnt % 20 ) == 9 )  //20ms一次，旋转心跳指令
    {
        u16CanTask2Step = Reg_RotatingMBDWatchDogCmd;
        RegReal_RotatingMBDWatchDogCmd.u16SN = u32CanTask2Cnt;
    }
    else if ( ( u32CanTask2Cnt % 20 ) == 11 )  //20ms一次，拉伸心跳指令
    {
        u16CanTask2Step = Reg_TelescopicMBDWatchDogCmd;
        RegReal_TelescopicMBDWatchDogCmd.u16SN = u32CanTask2Cnt;
    }
    else if ( ( u32CanTask2Cnt % 20 ) == 13 )  //20ms一次，环抱心跳指令
    {
        u16CanTask2Step = Reg_HuggingMBDWatchDogCmd;
        RegReal_HuggingMBDWatchDogCmd.u16SN = u32CanTask2Cnt;
    }
    else if ( ( u32CanTask2Cnt % 20 ) == 15 )  //20ms一次，压盖心跳指令
    {
        u16CanTask2Step = Reg_GlandMBDWatchDogCmd;
        RegReal_GlandMBDWatchDogCmd.u16SN = u32CanTask2Cnt;
    }
    else if ( ( u32CanTask2Cnt % 20 ) == 17 )  //20ms一次
    {
        if ( RotatingFaultClearFlag == 1 )
        {
            u16CanTask2Step = Reg_RotatingMBDFaultClearCmd;
            RotatingFaultClearFlag = 0;
        }
        else if ( TelescopicFaultClearFlag == 1 )
        {
            u16CanTask2Step = Reg_TelescopicMBDFaultClearCmd;
            TelescopicFaultClearFlag = 0;
        }
        else if ( HuggingFaultClearFlag == 1 )
        {
            u16CanTask2Step = Reg_HuggingMBDFaultClearCmd;
            HuggingFaultClearFlag = 0;
        } else if ( GlandFaultClearFlag == 1 )
        {
            u16CanTask2Step = Reg_GlandMBDFaultClearCmd;
            GlandFaultClearFlag = 0;
        }
        else if ( RotatingMotorResetFlag == 1 )
        {
            u16CanTask2Step = Reg_RotatingMBDResetCmd;
            RotatingMotorResetFlag = 0;
        }
        else if ( TelescopicMotorResetFlag == 1 )
        {
            u16CanTask2Step = Reg_TelescopicMBDResetCmd;
            TelescopicMotorResetFlag = 0;
        }
        else if ( HuggingMotorResetFlag == 1 )
        {
            u16CanTask2Step = Reg_HuggingMBDResetCmd;
            HuggingMotorResetFlag = 0;
        }
        else if ( GlandMotorResetFlag == 1 )
        {
            u16CanTask2Step = Reg_GlandMBDResetCmd;
            GlandMotorResetFlag = 0;
        }
        else if ( RotatingWRParamCmdFlag == 1 )
        {
            u16CanTask2Step = Reg_RotatingMBDParamCmd;
            RotatingWRParamCmdFlag = 0;
        }
        else if ( TelescopicWRParamCmdFlag == 1 )
        {
            u16CanTask2Step = Reg_TelescopicMBDParamCmd;
            TelescopicWRParamCmdFlag = 0;
        }
        else if ( HuggingWRParamCmdFlag == 1 )
        {
            u16CanTask2Step = Reg_HuggingMBDParamCmd;
            HuggingWRParamCmdFlag = 0;
        }
        else if ( GlandWRParamCmdFlag == 1 )
        {
            u16CanTask2Step = Reg_GlandMBDParamCmd;
            GlandWRParamCmdFlag = 0;
        }
    }
    else if ( ( u32CanTask2Cnt % 20 ) == 19 )  //20ms一次
    {
        u16CanTask2Step = Reg_RadarSend;
    }
    else
    {
    }

    switch ( u16CanTask2Step )
    {
    /*控制*/
    case Reg_RotatingMBDCmd:
    {
        Can2Task_Send ( Reg_RotatingMBDCmd );
    }
    break;
    case Reg_TelescopicMBDCmd:
    {
        Can2Task_Send ( Reg_TelescopicMBDCmd );
    }
    break;
    case Reg_HuggingMBDCmd:
    {
        Can2Task_Send ( Reg_HuggingMBDCmd );
    }
    break;
    case Reg_GlandMBDCmd:
    {
        Can2Task_Send ( Reg_GlandMBDCmd );
    }
    break;
    /*心跳*/
    case Reg_RotatingMBDWatchDogCmd:
    {
        Can2Task_Send ( Reg_RotatingMBDWatchDogCmd );
    }
    break;
    case Reg_TelescopicMBDWatchDogCmd:
    {
        Can2Task_Send ( Reg_TelescopicMBDWatchDogCmd );
    }
    break;
    case Reg_HuggingMBDWatchDogCmd:
    {
        Can2Task_Send ( Reg_HuggingMBDWatchDogCmd );
    }
    break;
    case Reg_GlandMBDWatchDogCmd:
    {
        Can2Task_Send ( Reg_GlandMBDWatchDogCmd );
    }
    break;
    /*复位*/
    case Reg_RotatingMBDResetCmd:
    {
        Can2Task_Send ( Reg_RotatingMBDResetCmd );
    }
    break;
    case Reg_TelescopicMBDResetCmd:
    {
        Can2Task_Send ( Reg_TelescopicMBDResetCmd );
    }
    break;
    case Reg_HuggingMBDResetCmd:
    {
        Can2Task_Send ( Reg_HuggingMBDResetCmd );
    }
    break;
    case Reg_GlandMBDResetCmd:
    {
        Can2Task_Send ( Reg_GlandMBDResetCmd );
    }
    break;
    /*参数管理*/
    case Reg_RotatingMBDParamCmd:
    {
        Can2Task_Send ( Reg_RotatingMBDParamCmd );
    }
    break;
    case Reg_TelescopicMBDParamCmd:
    {
        Can2Task_Send ( Reg_TelescopicMBDParamCmd );
    }
    break;
    case Reg_HuggingMBDParamCmd:
    {
        Can2Task_Send ( Reg_HuggingMBDParamCmd );
    }
    break;
    case Reg_GlandMBDParamCmd:
    {
        Can2Task_Send ( Reg_GlandMBDParamCmd );
    }
    break;
    /*故障清除*/
    case Reg_RotatingMBDFaultClearCmd:
    {
        Can2Task_Send ( Reg_RotatingMBDFaultClearCmd );
    }
    break;
    case Reg_TelescopicMBDFaultClearCmd:
    {
        Can2Task_Send ( Reg_TelescopicMBDFaultClearCmd );
    }
    break;
    case Reg_HuggingMBDFaultClearCmd:
    {
        Can2Task_Send ( Reg_HuggingMBDFaultClearCmd );
    }
    break;
    case Reg_GlandMBDFaultClearCmd:
    {
        Can2Task_Send ( Reg_GlandMBDFaultClearCmd );
    }
    break;
    /*标定*/
    case Reg_RotatingMBDCalaCmd:
    {
        Can2Task_Send ( Reg_RotatingMBDCalaCmd );
    }
    break;
    case Reg_TelescopicMBDCalaCmd:
    {
        Can2Task_Send ( Reg_TelescopicMBDCalaCmd );
    }
    break;
    case Reg_HuggingMBDCalaCmd:
    {
        Can2Task_Send ( Reg_HuggingMBDCalaCmd );
    }
    break;
    case Reg_GlandMBDCalaCmd://
    {
        Can2Task_Send ( Reg_GlandMBDCalaCmd );
    }
    break;
     case Reg_RadarSend://
    {
        Can2Task_Send ( Reg_RadarSend );
    }
    break;
    case Step_TelescopicMBDRelayCmd://
    {
        Can2Task_Send ( Step_TelescopicMBDRelayCmd );
    }
    break;
    default:
        break;
    }
}

