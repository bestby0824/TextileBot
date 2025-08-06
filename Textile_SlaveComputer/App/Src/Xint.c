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

//-------------------- pragmas ----------------------------------------------

//-------------------- include files ----------------------------------------
#include "Xint.h"
#include "TimerBase.h"
#include "Pid.h"
#include "AdcProcess.h"
#include "DidoProcess.h"
#include "Monitor.h"
#include "DataBaseProcess.h"
#include "Host_Com.h"
#include "Spd_Ctrl.h"
#include "VCUProcess.h"
#include "VCU_Ctrl.h"
#include "main.h"
#include "CAN_VCU.h"
#include "SteeringWheelProcess.h"
#include "NodeProcess.h"
#include "Monitor.h"
//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
uint16_t PowerOnCnt = 0, PowerOnFlag = 0, PowerOffCnt = 0,PowerFlag = 0, PowerTryCnt = 0;
//-------------------- private functions declare ----------------------------
static void PCPowerCtrl ( void )
{
   // static 
    if ( InputIOInfor.PowerSWSta && (PowerFlag == 0))
    {
        PowerOnCnt++;
        if ( (PowerOnCnt > 1000) && (PowerOnCnt < 5000)) //1s
        {
            if ( PowerOnFlag == 0 ) //开机状态
            {
                PowerOnFlag = 1;//进入关机状态标志
    //            m_PowerSwCmd.PowerSwOpt.bit.PcPowerSw = 0;//关机标志
                PowerOffCnt = 0;
            }
            else if ( PowerOnFlag == 2 ) //关机状态
            {
                PowerOnFlag = 3;
                PowerOffCnt = 0;
            }
        }
        else if(PowerOnCnt > 5000)//强制关机
        {
            
            PowerOffCnt++;
            if ( PowerOffCnt % 100 == 0 )
            {
                m_PowerSwCmd.PowerSwOpt.bit.PcPowerSw = 0;
            }
            /*6s长按强制关机*/
            if(PowerOnCnt > 6000)//强制关机动作
            {
                PowerFlag = 1;
                PowerOnFlag = 2;//关机完成标识
                PowerCtrl ( PCPower, OFF );
    //            PowerCtrl ( SensorPower, OFF );
                PowerOnCnt = 0;
                PowerOffCnt = 0;
            }
        }
    }
    else
    {
        PowerOnCnt = 0;
    }
    /*开关机逻辑*/
    if ( PowerOnFlag == 1 ) //进入关机逻辑
    {
        PowerOffCnt++;    
        if ( PowerOffCnt % 100 == 0 )
        {
            m_PowerSwCmd.PowerSwOpt.bit.PcPowerSw = 0;
        }
        if ( PowerOffCnt > 15000 )//15s
        {
            m_PowerSwCmd.PowerSwOpt.bit.PcPowerSw = 1;
            PowerOffCnt = 0;
            PowerOnFlag = 0;//开机完成
//                PowerOnFlag = 0;
        }
         /*检测Usb_5V电压,掉电关机完成，上电开机完成*/
        if ( InputIOInfor.PC_5VSta == 0 )//USB5v掉电，表示上位机已关机
        {
            PowerFlag = 1;
            PowerOffCnt = 0;
            PowerOnFlag = 2;//关机完成
            PowerCtrl ( PCPower, OFF );
            PowerCtrl ( SensorPower, OFF );
        }
    }
    else if ( PowerOnFlag == 3 ) //开机逻辑
    {
        PowerOffCnt++;
        PowerCtrl ( PCPower, ON );
        PowerCtrl ( SensorPower, ON );
        if ( PowerOffCnt > 15000 )//关机失败
        { 
            m_PowerSwCmd.PowerSwOpt.bit.PcPowerSw = 0;
            PowerOffCnt = 0;
            PowerOnFlag = 2;//关机完成
        }
        if ( InputIOInfor.PC_5VSta == 1 )
        {
            PowerFlag = 1;
            PowerOnFlag = 0;//开机完成
            PowerOffCnt = 0;
            
            m_PowerSwCmd.PowerSwOpt.bit.PcPowerSw = 1;
        }
    }
   
    /*开关机要间隔5s才能2次触发*/
    if(PowerFlag)
    {
        PowerTryCnt++;
    }
    if(PowerTryCnt > 5000)//5s
    {
        PowerFlag = 0;
        PowerTryCnt = 0;
    }
}
#if 0
uint16_t RegReal_TelescopicMBDRelayCmdCnt, RegReal_TelescopicMBDRelayCmdCnt1;
static void TelescopicMBDRelayCtrl ( void )
{
    static uint16_t LastSN;
    if(RegReal_TelescopicMBDRelayCmdFB.u16SN == 0)
    {
        return;
    }
    if(RegReal_TelescopicMBDRelayCmdFB.u16SwSta == 1)//碰撞触发，打开
    {
        RegReal_TelescopicMBDRelayCmdCnt++;
        RegReal_TelescopicMBDRelayCmd.u16EN = 1;
        RegReal_TelescopicMBDRelayCmd.u16Cmd = 1;
        RegReal_TelescopicMBDRelayCmd.u16RunTime = ParamHandle_Control.Reg_TelescopicMBDRelayRunTime;
        SolenoidCmddown.TelescopicMBDRelay_EndOfCtrl = RegReal_TelescopicMBDRelayCmdFB.u16Sta;
        if(RegReal_TelescopicMBDRelayCmdFB.u16Sta == 0)
        {
            RegReal_TelescopicMBDRelayCmd.u16Cmd = 0;
            RegReal_TelescopicMBDRelayCmdCnt = 0;
        }
        if(RegReal_TelescopicMBDRelayCmdCnt > 10000)
        {
            SolenoidCmddown.TelescopicMBDRelay_EndOfCtrl = 4;
        }
    }
    else if (m_sElectronMagCmd.u16SW == 0)
    {
        RegReal_TelescopicMBDRelayCmdCnt1++;
        RegReal_TelescopicMBDRelayCmd.u16EN = 1;
        RegReal_TelescopicMBDRelayCmd.u16Cmd = 2;
        RegReal_TelescopicMBDRelayCmd.u16RunTime = ParamHandle_Control.Reg_TelescopicMBDRelayRunTime;
        SolenoidCmddown.TelescopicMBDRelay_EndOfCtrl = RegReal_TelescopicMBDRelayCmdFB.u16Sta;
        if(RegReal_TelescopicMBDRelayCmdFB.u16Sta == 0)
        {
            RegReal_TelescopicMBDRelayCmd.u16Cmd = 0;
            RegReal_TelescopicMBDRelayCmdCnt1 = 0;
        }
        if(RegReal_TelescopicMBDRelayCmdCnt1 > 10000)
        {
            SolenoidCmddown.TelescopicMBDRelay_EndOfCtrl = 4;
        }
    }

    
}
#endif
//-------------------- public data ------------------------------------------
uint8_t g_u8Tick1kHz = 0;
uint8_t Stop_2_LEFT, Stop_2_RIGHT, Stop_2_DOWN, Stop_2_UP = 0;
uint16_t DbugCom_SendCnt = 0;
uint16_t BreakTimecnt = 0;
uint16_t PosModeFlag = 0, SlowModeFlag;
uint16_t SlowDiatance = 0, SlowDir = 0;
uint8_t Status_1, Status_2, DRStatus;
uint8_t DbugCom_Tx[8], DbugComLen = 8, ReportSendFlag = 0;
uint16_t FirstBreak = 0;
uint16_t RadarSwitch = 0; //避障开关默认打开
uint32_t System_Cnt = 0;
uint16_t HornSwitch = 0, HornTime = 0; //车辆鸣笛计数和鸣笛开关,鸣笛时间
uint32_t HornCnt = 0;
uint8_t  BreakCanelSwitch = 0;
uint16_t Break_Time = 0, u16Wait1000mSCnt = 0, Break_Time_Stop = 500;
uint16_t u16RCStop_2Mode = 0, u16RCStop_2Cnt = 0;
uint16_t StreeCnt = 0;
uint16_t BreakPos = 0;
uint16_t BreakFlag = 0;
CPURateHandle CPURateHandle1;              //采集各线程占用时间
int32_t AGVTurnDegFb = 0;
uint8_t u8PowerSwFlag = 0;
int16_t Diatance = 0;
//-------------------- public functions ----------------------------
/**
  * @brief  void TimerBase_Update_Callback(void)
  * @param  none
  * @retval None
  */
void TimerBase_Update_Callback ( void ) //10KHZ    每个不得超过100uS 优先级最高
{
    uint32_t ticks;
    static uint32_t Cnt_1mS = 0, Cnt_2mS = 5;

    CPURateHandle1.InTick = TimerBase_GetTicks_50KHz();    //放在TimerBase_Update_Callback函数开头
    ticks = TimerBase_GetTicks_10KHz();

    IMUCom_ProcRecvData();//1kHz调用，最大延时1mS，100Hz刷新

    switch ( ticks % 10 )
    {
    case 0://1kHz         2us
    {
        Cnt_1mS++;
        Cnt_2mS++;
        g_u8Tick1kHz = 1;
        System_Cnt++;
        SpdClc ( 1000 );//左右轮
        PosAddCalc();

    }
    break;

    case 1:                       //4us---VCU控制
    {

        //HostCom_SlowSpdCommand ( &SlowDir, &SlowDiatance );//上位机接收的车辆小位置缓行（蠕动模式）的控制距离

        if ( m_eCtrlMode == PC_MODE )             //自动模式读取上位机指令
        {
            if ( 0 == HostCom_ReadBrakeCommand ( &Diatance, &PosModeFlag ) ) //上位机接收的刹车控制距离
            {
                SolenoidCmddown.Brake_EndOfCtrl = 1;
                AGVWalkBrakeCnt = 0;
                BrakeCmdCnt = 0;
                s16AGVWalkBrakePosFB = 0;
            }
            //定点刹车EndOfCtrl更新
            if ( SolenoidCmddown.Brake_EndOfCtrl == 1 )
            {
                BrakeCmdCnt++;
            }
            else if ( SolenoidCmddown.Brake_EndOfCtrl == 0 )
            {
                BrakeCmdCnt = 0;
            }
            if ( BrakeCmdCnt > 25000 ) //25s
            {
                SolenoidCmddown.Brake_EndOfCtrl = 2;
            }

            if ( 0 == HostCom_SlowSpdCommand ( &SlowDir, &SlowDiatance ) ) //上位机接收的车辆小位置缓行（蠕动模式）的控制距离
            {
                SolenoidCmddown.SlowSpd_EndOfCtrl = 1;
                AGVSlowSpdPosCnt = 0;
                SlowSpdCmdCnt = 0;
                AGVSlowSpdPosFB = 0;
            }
            //蠕动EndOfCtrl更新
            if ( SolenoidCmddown.SlowSpd_EndOfCtrl == 1 )
            {
                SlowSpdCmdCnt++;
            }
            else if ( SolenoidCmddown.SlowSpd_EndOfCtrl == 0 )
            {
                SlowSpdCmdCnt = 0;
            }
            if ( SlowSpdCmdCnt > 25000 ) //25s
            {
                SolenoidCmddown.SlowSpd_EndOfCtrl = 2;
            }

            if ( 0 == BreakCanelFun ( &BreakCanelSwitch, &Break_Time ) )  //读取导航刹车指令
            {
                SolenoidCmddown.BreakCanel_EndOfCtrl = 1;
                BrakeCanelCmdCnt = 0;
                BreakCanelCnt = 0;
            }
            //EndOfCtrl更新
//            if ( SolenoidCmddown.BreakCanel_EndOfCtrl == 1 )
//            {
//                BrakeCanelCmdCnt++;
//            }
//            else if ( SolenoidCmddown.BreakCanel_EndOfCtrl == 0 )
//            {
//                BrakeCanelCmdCnt = 0;
//            }
//            if ( BrakeCanelCmdCnt > 20000 ) //20s
//            {
//                SolenoidCmddown.BreakCanel_EndOfCtrl = 2;
//            }

            HostCom_ReadGasCommand ( &s16TargetSpeed ); //
            if ( AGVReadErrCnt >= AGVReadErrMaxCnt )
            {
                m_sCurGasCmd.u16SN = 0;
                s16TargetSpeed = 0;
            }
            if ( 0 == HostCom_ReadSteerCommand ( &TurnDeg ) )
            {
                AGVTurnCtrlMode = 1;
            }
            if ( 0 == HostCom_ReadLiftCommand () )
            {
                SolenoidCmddown.liftEndOfCtrl = 1;      //收到升降指令后默认将EndOfCtrl置1，且需要50ms不变，待液压板上报EndOfCtrl为0时判定为运动结束
                LiftCmdCnt = 0;
                AGVLiftEndOfCtrlCnt = 0;
            }
            //EndOfCtrl更新
            if ( SolenoidCmddown.liftEndOfCtrl == 1 )
            {
                LiftCmdCnt++;
            }
            else if ( SolenoidCmddown.liftEndOfCtrl == 0 )
            {
                LiftCmdCnt = 0;
                SolenoidCtrlCmd.Lift.EN = 0;
            }
            if ( LiftCmdCnt > 10000 ) //10s
            {
                SolenoidCmddown.liftEndOfCtrl = 2;
                SolenoidCtrlCmd.Lift.EN = 0;
            }


            if ( m_sCurSteerCmd.s16Pos >= TurnDegThreshold )
            {
                TurnSignalCtrl ( Sta_R );
            }
            else if ( m_sCurSteerCmd.s16Pos <= -TurnDegThreshold )
            {
                TurnSignalCtrl ( Sta_L );
            }
            else
            {
                TurnSignalCtrl ( Sta_N );
            }

        }
        else if ( m_eCtrlMode == RC_MODE ) //遥控器控制模式，读取遥控器控制指令
        {
//            Hart_DriveCmd();//控制VCU行进
            WirelessCtrl();
            if ( FaultCtrl.bit.RotatingLinkLose ||  RegReal_RotatingMBDFaultStaFB.u32Fault )
            {
                RegReal_RotatingMBDCmd.u16SN = 0;
                RegReal_RotatingMBDCmd.u16EN = 0;
                HostCom_ClearCommand();//故障清除SN
            }
            if ( FaultCtrl.bit.TelescopicLinkLose || RegReal_TelescopicMBDFaultStaFB.u32Fault || FaultCtrl.bit.TelescopicMBDRelayErr )
            {
                RegReal_TelescopicMBDCmd.u16SN = 0;
                RegReal_TelescopicMBDCmd.u16EN = 0;
                HostCom_ClearCommand();//故障清除SN
            }
            if ( FaultCtrl.bit.HuggingLinkLose || RegReal_HuggingMBDFaultStaFB.u32Fault )
            {
                RegReal_HuggingMBDCmd.u16SN = 0;
                RegReal_HuggingMBDCmd.u16EN = 0;
                HostCom_ClearCommand();//故障清除SN
            }
            if ( FaultCtrl.bit.GlandLinkLose ||  RegReal_GlandMBDFaultStaFB.u32Fault )
            {
                RegReal_GlandMBDCmd.u16SN = 0;
                RegReal_GlandMBDCmd.u16EN = 0;
                HostCom_ClearCommand();//故障清除SN
            }
             if ( FaultCtrl.bit.VCULinkLose ||  FaultCtrl.bit.TurnError || FaultCtrl.bit.ExcessTemperature \
                    || Reg_AGVDataFb1.u8WalkFaultCode || Reg_AGVPumpCtrFb.u8WalkFaultCode || Reg_AGVBMSStaOrigDataFb.u8FaultCode )
             {
                Reg_AGVCtrlCmd1.s16WalkSpd = 0;
                Reg_AGVTurnCtrlCmd2.s32PosLpSpdCmd = 0;
                Reg_AGVCtrlCmd2.u8PumpSpd = 0;
                Reg_AGVCtrlCmd1.u8WalkDnPV = 0;
             }
        }
        else
        {
            s16TargetSpeed = 0;
        }

    }
    break;

    case 2:             //2us
    {
         if ( m_eCtrlMode == PC_MODE )        //自动模式下才接收上位机指令
        {
            HostCom_ReadHuggingCommand();
            HostCom_ReadGlandCommand();
            HostCom_ReadTelescopicCommand();
            HostCom_ReadRotatingCommand();
            HostCom_ReadElectronmagCommand();//电磁铁开关控制
            HostCom_ReadTelescopicMBDRelayCommand();
//            HostCom_ResetCommand();//复位
            if(  0 == HostCom_PowerSwCommand() )
            {
                u8PowerSwFlag = 1;
            }
//            MotorCalaCheck();//电机0标定
            if( u8BootLinkFlag == 1 )
            {
                u8BootLinkFlag = 0;
                if(Reg_AGVDataFb1.s16WalkSpd == 0 && Reg_AGVPumpCtrFb.s16PumpSpd == 0 && \
                    RegReal_RotatingMBDCtrlFB.u16Sta == 0 && RegReal_TelescopicMBDCtrlFB.u16Sta == 0 &&\
                    RegReal_HuggingMBDCtrlFB.u16Sta == 0 && RegReal_GlandMBDCtrlFB.u16Sta == 0 )
                {
                    BootProcess();
                }
            }
//            TelescopicMBDRelayCtrl();
             if(FaultCtrl.all)
            {
                RegReal_RotatingMBDCmd.u16SN = 0;
                RegReal_RotatingMBDCmd.u16EN = 0;
                RegReal_TelescopicMBDCmd.u16SN = 0;
                RegReal_TelescopicMBDCmd.u16EN = 0;
                RegReal_GlandMBDCmd.u16SN = 0;
                RegReal_GlandMBDCmd.u16EN = 0;
                RegReal_HuggingMBDCmd.u16SN = 0;
                RegReal_HuggingMBDCmd.u16EN = 0;
                RegReal_TelescopicMBDRelayCmd.u16SN = 0;
                RegReal_TelescopicMBDRelayCmd.u16EN = 0;
                HostCom_ClearCommand();//故障清除SN
            }
        }
        else if ( m_eCtrlMode == RC_MODE ) //遥控模式
        {

            // Hart_SteerCmd();//转向控制
        }
    }
    break;

    case 3:          //5us转向板寄存器填充
    {
        MonitorProcess();     //故障检测函数
       
       
    }
    break;

    case 4://
    {
        UpdateIOSta();
        PCPowerCtrl();//开关机控制
        
    }
    break;

    case 5:                      //3us----调试
    {
#if (DebugMode == Debug_Handle)////用于手持终端调试


#if 0
        if ( ( ( Cnt_1mS % 50 ) == 0 ) && ( u8BootFlg_Solenoid == 0 ) && ( u8BootFlg_SteeringWheel == 0 ) ) {
            DbugCom_SendCnt = DbugCom_SendCnt + 1;
            if ( DbugCom_SendCnt >= 6 ) DbugCom_SendCnt = 0;
            DbugCom_SendPkg ( DbugCom_SendCnt ) ;
        }
#endif
#elif( DebugMode == Debug_Data)
//        VCU_ParamHandle.s16ChassisMotorSpd = VCU_ParamHandle.s16ChassisMotorSpd*1.8269;
        extern int32_t TurnDeg, TurnDegFb;
//        static uint16_t u16TurnDeg,u16TurnDegFb;


//        u16TurnDeg = _IQdiv4(TurnDeg + _IQ(0.5));
//        u16TurnDegFb = _IQdiv4(TurnDegFb + _IQ(0.5));
        DbugCom_Tx[0] = ( iqPosition_m >> 2 ) >> 8;
        DbugCom_Tx[1] = ( iqPosition_m >> 2 );
        DbugCom_Tx[2] = INVWalkspdClc ( Reg_AGVDataFb1.s16WalkSpd ) >> 8;
        DbugCom_Tx[3] = INVWalkspdClc ( Reg_AGVDataFb1.s16WalkSpd );
//        DbugCom_Tx[0] = u16TurnDeg>>8;
//        DbugCom_Tx[1] = u16TurnDeg;
//        DbugCom_Tx[2] = u16TurnDegFb>>8;
//        DbugCom_Tx[3] = u16TurnDegFb;
//        DbugCom_Tx[0] = 0x11;
//        DbugCom_Tx[1] = 0x22;
//        DbugCom_Tx[2] = 0x33;
//        DbugCom_Tx[3] = 0x44;
//        DbugCom_Tx[4] = RegReal_ChassisParam.u8ChassisEnabled.bit.ParkingBrakeEnabled;
//        DbugCom_Tx[5] = 0;
//        DbugCom_Tx[6] = VCU_ParamHandle.u8ParkingBrakeMode;
//        DbugCom_Tx[7] = 0;

        Uart_Sprintf_Send ( DbugCom_Tx, 4 );
#endif
//        DbugCom_Tx[0] = SolenoidReport.unSideShift_Pressure1;
//        DbugCom_Tx[1] = SolenoidReport.unSideShift_Pressure1 >> 8;
//        DbugCom_Tx[2] = SolenoidReport.unSideShift_Pressure2;
//        DbugCom_Tx[3] = SolenoidReport.unSideShift_Pressure2 >> 8;

//        DbugCom_Tx[0] = ParamHandle_Control.Reg_Tab_Ver;
//        DbugCom_Tx[1] = ParamHandle_Control.Reg_Tab_Ver >> 8;
//        DbugCom_Tx[2] = ParamHandle_Control.Reg_SN;
//        DbugCom_Tx[3] = ParamHandle_Control.Reg_SN >> 8;
//        DbugCom_Tx[0] = 0x01;
//        DbugCom_Tx[1] = 0x10;
//        DbugCom_Tx[2] = 0x55;
//        DbugCom_Tx[3] = 0xAA;
//        Uart_PCDbug_Send(DbugCom_Tx, 4);
        if ( 0 == Cnt_1mS % 50 ) //20hz上报状态
        {
            Uart_PCDbug_Send ( DbugCom_Tx, 4 ); //上位机调试


        }

        DbugCom_ReceiveData();
//        RS485_SteeringWheel_Send( DbugCom_Tx, 4 );
    }
    break;

    case 6:     //1us
    {
        ADC_SWStart();

    }
    break;

    case 7:     //14us
    {
        HostCom_RadarCommand ( &RadarSwitch );
    }
    break;

    case 8:            //4us控制模式选择
    {
        HostCom_ProcRecvData();//上位机数据接收
       
        if ( CANSendFlag == 1 )
        {
            CanTaskProcess ( );

            CanTask2Process ( );

            VCU_RegState_Ctrl ( );

            CanBusOff_Reset ( );

        }

    }
    break;

    case 9:             //27us---信息上报上位机
    {

        if ( 0 == Cnt_1mS % 20 ) //50hz上报状态
        {
            HostCom_SendReport();//上报ACK 到上位机
        }
        if ( 0 == Cnt_1mS % 50 ) //20hz查询状态--bin add
        {
            ADC_Value();
            Energy_Consumption();
        }
    }
    break;

    default:
    {

    }
    break;
    }

    CPURateHandle1.OutTick = TimerBase_GetTicks_50KHz();
    if ( CPURateHandle1.OutTick > CPURateHandle1.InTick )
    {
        CPURateHandle1.Task_Rate[ticks % 10] = 20 * ( CPURateHandle1.OutTick - CPURateHandle1.InTick );
    }

}

//-------------------- private functions ----------------------------





/*****************************END OF FILE****/
