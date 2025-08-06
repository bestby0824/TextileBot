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
//---------------------------------------------------------------------------

//-------------------- include files ----------------------------------------
#include "Monitor.h"
#include "DidoProcess.h"
#include "AdcProcess.h"
#include "Uart_Dbug.h"
#include "StateCtrl.h"
#include "stdio.h"
#include "DataBaseProcess.h"
#include "pullrodProcess.h"
#include "SteeringWheel_Ctrl.h"
#include "Spd_Ctrl.h"
#include "EncoderProcess.h"
#include "VCUProcess.h"
#include "Xint.h"
#include "RGBLED_Process.h"
#include "string.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------

//-------------------- private functions declare ----------------------------
static RadarOut GetRadarOut ( void );
static uint8_t KeySta ( void );
//-------------------- public data ------------------------------------------
//Fault_Handle Fault_Handle1;
Fault_REG FaultCtrl;
Warning_REG Warning_Report;
uint8_t Stop_1A_State_Flag;
uint8_t V_Flag, Buzzer_Flag;
uint16_t V_24 = 0, V_80 = 0, Motor_24;
uint32_t ExTemperature_Cnt = 0;
uint16_t D_State = 0, R_State = 0, N_State = 0;
uint16_t RunState;          //当前上位机给定的档位
int16_t Temperature;        //CPU温度
GPIO_PinState Charge_State ;
int16_t  Stop_1A_Data;
GPIO_PinState BumperFlag_1, BumperFlag_2;
extern uint8_t DRStatus;
uint16_t BumperState = 0;
uint16_t LightState = 0;                        //灯光状态 0是绿灯常亮，1是绿灯闪烁，2是黄灯常亮，3是黄红闪烁，4是灯常亮
uint32_t  LightCnt = 0, LightPWMCnt;            //闪灯计数器
RGBMode RGBMode1 = RGBMode_TEST;
//Solenoid_Warning_REG Solenoid_Warning;
uint16_t Radar_Zone_Left = 0, Radar_Zone_Right = 0;
uint8_t ElecMagSta = 0;
//-------------------- public functions -------------------------------------
//int16_t  HARTCMDLife = 3000 ;//遥控器心跳
RadarOut RadarOut_R, RadarOut_L, RadarResult;
uint16_t Radar_Zone_Now = 1;        //避障雷达选择区域
Energy_Struct Energy_PC, Energy_SENSOR, Energy_MOTOR1, Energy_MOTOR2, Energy_ElecMag;
Light_REG Light_State;
uint16_t  HostFaultClear = 0;//
//获取故障位号
/*
* @brief       函数
* @param       无
* @retval      无
*
*/
uint16_t FaultNum_Get ( void )
{
    uint8_t BitCnt;
    uint16_t LEDFaultNum = 0;
    uint32_t u32Fault;
    static uint8_t Lockbit = 0;

    u32Fault = FaultCtrl.all;

    if ( u32Fault )
    {
        for ( BitCnt = Lockbit; BitCnt < 32; BitCnt++ )
        {
            if ( u32Fault & BIT ( BitCnt ) )
            {
                LEDFaultNum = BitCnt + 1;
                Lockbit = LEDFaultNum;
                break;
            }
        }
        if ( BitCnt == 32 )
        {   //最后1位仍没有跳出
            for ( BitCnt = 0; BitCnt < 32; BitCnt++ )
            {
                if ( u32Fault & BIT ( BitCnt ) )
                {
                    LEDFaultNum = BitCnt + 1;
                    Lockbit = LEDFaultNum;
                    break;
                }
            }
        }
    }
    else
    {
        LEDFaultNum = 0;
    }
    return LEDFaultNum;
}

/*
* @brief       函数
* @param       无
* @retval      无
*
*/
void Stop_1A_State ( void )                         //急停状态检测
{
    Stop_1A_Data = _IQmpy ( Get_E_Stop_1A(), _IQ ( 0.1 ) ) + _IQmpy ( Stop_1A_Data, _IQ ( 0.9 ) );

    if ( abs ( Stop_1A_Data ) > 400 )
    {
        Stop_1A_State_Flag = 1;  //急停未开启状态
    }
    else
    {
        Stop_1A_State_Flag = 0;  //急停状态
    }
    if ( Stop_1A_State_Flag == 1 )
    {
        if ( FaultCtrl.bit.EmergencyStop )
        {
            FaultCtrl.bit.EmergencyStop = 0;//急停故障状态标志
//            FaultCtrl.bit.SteerLinkLose = 0;//自动清除
            m_nSteeringLife = 5000;
            StateResetSystem();
        }
    }
}

/*
* @brief       函数
* @param       无
* @retval      无
*
*/
//void Motor_24Check ( void )
//{
//    uint32_t Motor_24_Cnt = 0;
//    if ( ( Motor_24 < 2200 ) && ( FaultCtrl.bit.EmergencyStop == 0 ) )
//    {
//        Motor_24_Cnt++;
//        if ( Motor_24_Cnt > 500 )
//            FaultCtrl.bit.Power24VErr = 1;
//    }
//}
/*
* @brief       心跳监控函数
* @param       无
* @retval      无
*
*/
void HeartBeat_Check ( void )         //心跳状态检测
{
//#if 0
    //上位机心跳检测
    if ( ( m_eCtrlMode == PC_MODE ) && ( FaultCtrl.bit.EmergencyStop == 0 ) && ( InputIOInfor.PC_5VSta == 1 ) )
    {
        if ( m_nHostLife > 300 )
        {
            FaultCtrl.bit.HostLinkLose = 0;//自动清除
        }
        if ( m_nHostLife > 0 ) m_nHostLife--;
        if ( m_nHostLife <= 0 )
        {
            FaultCtrl.bit.HostLinkLose = 1;
            memset ( &m_sCommandAck, 0, sizeof ( m_sCommandAck ) );
        }
    }
    else
    {
        FaultCtrl.bit.HostLinkLose = 0;//自动清除
    }

    //VCU心跳检测

    if ( VCULife > 300 )
    {
        FaultCtrl.bit.VCULinkLose = 0;//自动清除
    }
    if ( VCULife > 0 ) VCULife--;
    if ( VCULife <= 0 )
    {
        FaultCtrl.bit.VCULinkLose = 1;
    }

    //旋转驱动板心跳检测
    if ( m_nRotatingMBLife > 300 )
    {
        FaultCtrl.bit.RotatingLinkLose = 0;//自动清除
    }
    if ( m_nRotatingMBLife > 0 ) m_nRotatingMBLife--;
    if ( m_nRotatingMBLife <= 0 )
    {
//      HostCom_ClearCommand();//故障后不接收控制指令
        FaultCtrl.bit.RotatingLinkLose = 1;
    }
    //伸缩驱动板心跳检测
    if ( m_nTelescopicMBLife > 300 )
    {
        FaultCtrl.bit.TelescopicLinkLose = 0;//自动清除
    }
    if ( m_nTelescopicMBLife > 0 ) m_nTelescopicMBLife--;
    if ( m_nTelescopicMBLife <= 0 )
    {
//      HostCom_ClearCommand();//故障后不接收控制指令
        FaultCtrl.bit.TelescopicLinkLose = 1;
    }
    //环抱驱动板心跳检测
    if ( m_nHuggingMBLife > 300 )
    {
        FaultCtrl.bit.HuggingLinkLose = 0;//自动清除
    }
    if ( m_nHuggingMBLife > 0 ) m_nHuggingMBLife--;
    if ( m_nHuggingMBLife <= 0 )
    {
//      HostCom_ClearCommand();//故障后不接收控制指令
        FaultCtrl.bit.HuggingLinkLose = 1;
    }
    //压盖驱动板心跳检测
    if ( m_nGlandMBLife > 300 )
    {
        FaultCtrl.bit.GlandLinkLose = 0;//自动清除
    }
    if ( m_nGlandMBLife > 0 ) m_nGlandMBLife--;
    if ( m_nGlandMBLife <= 0 )
    {
//      HostCom_ClearCommand();//故障后不接收控制指令
        FaultCtrl.bit.GlandLinkLose = 1;
    }
//#endif
    //IMU心跳检测
    if ( IMULife > 300 )
    {
        Warning_Report.bit.IMULinkLose = 0;//自动清除
    }
    if ( IMULife > 0 ) IMULife--;
    if ( IMULife <= 0 )
    {
        Warning_Report.bit.IMULinkLose = 1;
    }
    if ( RadarLife > 300 )
    {
        Warning_Report.bit.RadarLinkLose = 0;
    }
    if ( RadarLife > 0 ) RadarLife--;
    if ( RadarLife <= 0 )
    {
        Warning_Report.bit.RadarLinkLose = 1;
    }
    if ( DrawWireLife > 300 )
    {
        Warning_Report.bit.DrawWireLinkLose = 0;
    }
    if ( DrawWireLife > 0 ) DrawWireLife--;
    if ( DrawWireLife <= 0 )
    {
        Warning_Report.bit.DrawWireLinkLose = 1;
    }
//手操器心跳检测
//    if ( HARTCMDLife > 300 )
//    {
//        RegReal_HARTCMD.u8HARTCMD_EmergencyStop = 0;//模式 bit1：急停关
//    }
//    if ( HARTCMDLife > 0 ) HARTCMDLife--;
//    if ( HARTCMDLife <= 0 )
//    {
//        RegReal_HARTCMD.u8HARTCMD_EmergencyStop = 1;//模式 bit1：急停开
//    }
    if ( HostSpdLife > 0 )
    {
        HostSpdLife--;
    }
    if ( HostSpdLife == 0 )       //超过2秒没有收到上位机的新的速度指令，速度清零
    {
        s16TargetSpeed = 0;
    }

}

/**
 * @brief       温度检测函数
 * @param       无
 * @retval      返回值为是否过温
 */
static int TemperatureCheck ( void )
{
    static uint8_t res = NoFault;
    int16_t s16Temperature = 0;
    static uint16_t u16HighTempCnt = 0;

    res = NoFault;

    s16Temperature = Get_Temperature();
    // g_sMotor_Info.s16Temp = s16Temperature * 0.01;
    if ( s16Temperature > HighTempValue )
    {
        u16HighTempCnt++;
        if ( u16HighTempCnt > 2000 ) //2s
        {
            res = Fault;
        }
    }
    else
    {
        u16HighTempCnt = 0;
//        res = NoFault;
    }

    return res;
}
uint32_t u32MOTOR1CurrOverCnt = 0;
static int MOTOR1CurrOverCheck ( void )
{
    static uint8_t res = NoFault;
    res = NoFault;
    if ( Energy_MOTOR1.I > _IQ ( 15 ) )
    {
        u32MOTOR1CurrOverCnt++;
        if ( u32MOTOR1CurrOverCnt > 1000 ) //
        {
            res = Fault;
            PowerCtrl ( Motor1Power, OFF );
        }
    }
    else
    {
        if ( u32MOTOR1CurrOverCnt > 0 ) u32MOTOR1CurrOverCnt--;
    }

    return res;
}

uint32_t u32MOTOR2CurrOverCnt = 0;
static int MOTOR2CurrOverCheck ( void )
{
    static uint8_t res = NoFault;
    res = NoFault;
    if ( Energy_MOTOR2.I > _IQ ( 15 ) )
    {
        u32MOTOR2CurrOverCnt++;
        if ( u32MOTOR2CurrOverCnt > 1000 ) //
        {
            res = Fault;
            PowerCtrl ( Motor2Power, OFF );
        }
    }
    else
    {
        if ( u32MOTOR2CurrOverCnt > 0 ) u32MOTOR2CurrOverCnt--;
    }

    return res;
}
uint32_t u32PCCurrOverCnt = 0;
static int PCCurrOverCheck ( void )
{
    static uint8_t res = NoFault;
    res = NoFault;
    if ( Energy_PC.I > _IQ ( 15 ) )
    {
        u32PCCurrOverCnt++;
        if ( u32PCCurrOverCnt > 1000 ) //
        {
            res = Fault;
            PowerCtrl ( PCPower, OFF );
        }
    }
    else
    {
        if ( u32PCCurrOverCnt > 0 ) u32PCCurrOverCnt--;
    }

    return res;
}

uint32_t u32SensorCurrOverCnt = 0;
static int SensorCurrOverCheck ( void )
{
    static uint8_t res = NoFault;
    res = NoFault;
    if ( Energy_SENSOR.I > _IQ ( 10 ) )
    {
        u32SensorCurrOverCnt++;
        if ( u32SensorCurrOverCnt > 1000 ) //
        {
            res = Fault;
            PowerCtrl ( SensorPower, OFF );
        }
    }
    else
    {
        if ( u32SensorCurrOverCnt > 0 ) u32SensorCurrOverCnt--;
    }

    return res;
}
/*
* @brief       监控函数---1k
* @param       无
* @retval      无
*
*/
uint32_t FaultCheckDisable = 0xFFFF & ( ~ ( BIT ( 0 ) | BIT ( 1 ) | BIT ( 2 ) | BIT ( 3 ) | BIT ( 4 ) ) );
int16_t BLDC_24V, VALVE_24V;
uint8_t PC_5V ;
void MonitorProcess ( void )
{
    if ( ( FaultCheckDisable & BIT ( 1 ) ) == 0 )
    {
        FaultCtrl.bit.ExcessTemperature = TemperatureCheck();
    }
    else
    {
        FaultCtrl.bit.ExcessTemperature = 0;
    }
    if ( ( FaultCheckDisable & BIT ( 2 ) ) == 0 )
    {
        HeartBeat_Check();          //心跳状态检测
    }
    else
    {
        FaultCtrl.bit.HostLinkLose = 0;
        Warning_Report.bit.IMULinkLose = 0;
//        FaultCtrl.bit.SolenoidLinkLose = 0;
//        FaultCtrl.bit.SteerLinkLose = 0;
    }

    if ( ( FaultCheckDisable & BIT ( 3 ) ) == 0 )
    {
        // VALVE_24V = _IQrmpy ( ( ( int16_t ) ADC_GetValue ( ADC_CH_VALVE_24V ) ), _IQ ( 1 ) );
        // BLDC_24V = _IQrmpy ( ( ( int16_t ) ADC_GetValue ( ADC_CH_BLDC_24V ) ), _IQ ( 1 ) );
        if ( Energy_PC.U < _IQ ( 20 ) )
        {
            Warning_Report.bit.PC_Power24V_Low = 1;
            m_PowerSwCmd.PowerSwOpt.bit.PcPowerSw = 0;
        }
        else
        {
            Warning_Report.bit.PC_Power24V_Low = 0;
//            m_PowerSwCmd.PowerSwOpt.bit.PcPowerSw = 1;
        }

        if ( Energy_MOTOR1.U < _IQ ( 20 ) )
        {
            Warning_Report.bit.BLDC1_Power24V_Low = 1;
            m_PowerSwCmd.PowerSwOpt.bit.Motor1PowerSw = 0;
        }
        else
        {
            Warning_Report.bit.BLDC1_Power24V_Low = 0;
            m_PowerSwCmd.PowerSwOpt.bit.Motor1PowerSw = 1;
        }
        if ( Energy_MOTOR2.U < _IQ ( 20 ) )
        {
            Warning_Report.bit.BLDC2_Power24V_Low = 1;
            m_PowerSwCmd.PowerSwOpt.bit.Motor2PowerSw = 0;
        }
        else
        {
            Warning_Report.bit.BLDC2_Power24V_Low = 0;
            m_PowerSwCmd.PowerSwOpt.bit.Motor2PowerSw = 1;
        }
        if ( Energy_SENSOR.U < _IQ ( 20 ) )
        {
            Warning_Report.bit.Sensor_Power24V_Low = 1;
            m_PowerSwCmd.PowerSwOpt.bit.SensorPowerSw = 0;
        }
        else
        {
            Warning_Report.bit.Sensor_Power24V_Low = 0;
            m_PowerSwCmd.PowerSwOpt.bit.SensorPowerSw = 1;
        }

//        DI_ReadByIndex ( &PC_5V, DI_CH_PC_5V );
    }
    else
    {
//        FaultCtrl.bit.Power24VErr = 0;
    }
//    if ( ( ( SolenoidReport.unRotateDegree < _IQ ( 0.208 ) ) || ( SolenoidReport.unRotateDegree > _IQ ( 0.792 ) ) ) && ( System_Cnt > 45000 ) ) //旋转角度小于75°或者大于285°报错急停
//    {

//    }
//    else
//    {
//        FaultCtrl.bit.RotatePosOutErr = 0;
////        DoSet0_STOP_CTRL_ON ( OFF );        //急停
//    }
    /************************过温检测****************************/
//    if ( ( WarnCheckEnable & BIT ( 5 ) ) && (  ) )
//    {
//        if ( 0 == g_u32WarningNum.bit.uTempWarnning )
//        {
//            g_u32WarningNum.bit.uTempWarnning = TemperatureCheck ( HighTempWarnValue );
//        }
//    }
//    if ( ( FaultCheckEnable & BIT ( 9 ) ) && ( u8Standby_Flag ) )
//    {
//        if ( ( 0 == g_u32FaultNum.bit.uTempOver ) && ( g_u32WarningNum.bit.uTempWarnning ) )
//        {
//            g_u32FaultNum.bit.uTempOver = TemperatureCheck ( HighTempErrValue );
//        }
//        else
//        {
//            ;
//        }
//    }
//    else
//    {
//        g_u32FaultNum.bit.uTempOver = 0;
//    }
    if ( ( FaultCheckDisable & BIT ( 4 ) ) == 0 )
    {
        FaultCtrl.bit.MOTOR1CurrOver = MOTOR1CurrOverCheck();
        FaultCtrl.bit.MOTOR2CurrOver = MOTOR2CurrOverCheck();
        FaultCtrl.bit.PCCurrOver = PCCurrOverCheck();
        FaultCtrl.bit.SensorCurrOver = SensorCurrOverCheck();
    }
    if ( RadarSwitch )
    {
        Radar_Process();         //避障雷达检测
        RaderReport = RadarResult;
    }
    else
    {
        RaderReport = 4;
    }
    RegReal_RadarSend.u8RadarSw = RadarSwitch;
    if ( Reg_AGVDataFb1.s16WalkSpd < 0 ) //
    {
        RegReal_RadarSend.u8WorkDir = 2;
    }
    if ( Reg_AGVDataFb1.s16WalkSpd > 0 ) //
    {
        RegReal_RadarSend.u8WorkDir = 1;
    }
    RegReal_RadarSend.s16TurnDeg = ( int16_t ) iqVCUTurnDegFB;
//    Fault_Handle1.PumpMotorFault =  Reg_AGVPumpCtrFb.u8WalkFaultCode;
    LightStateCheck();

    //管理各部件停止标记
//    if ( InputIOInfor.RST_MCUSta )
//    {
//        HostFaultClear = SlaveBoard;
//    }
//     switch(HostFaultClear)
//     {
//        case SlaveBoard:
//            HostFaultClear = RotatingBoard;
//            FualtClear(&HostFaultClear);
//            break;
//       case RotatingBoard:
//        HostFaultClear = RotatingBoard;
//        break;
//    }
}
/*textile*/
/*
* @brief       采样电压转换函数
* @param       无
* @retval      无
*
*/
static _iq SampleVolt ( int16_t adc_value )
{
    _iq res;
    res = _IQmpy ( _IQ ( adc_value ), VREF_PER_ADC );

    return ( _IQmpy ( res, RESISTOR_RATIO ) );
}
/*
* @brief       采样电流转换函数
* @param       无
* @retval      无
*
*/
static _iq SampleCurr ( int16_t adc_value, _iq CurrRatio )
{
    _iq res;
    res = _IQmpy ( _IQ ( adc_value ), VREF_PER_ADC );

    return ( _IQmpy ( res, CurrRatio ) );
}
/*textile*/
/*
* @brief       功率计算函数
* @param       无
* @retval      无
*
*/
/*2hz获取电压和电流计算功耗
1.计算下位机24V功耗；
2.计算PC24V功耗；
3.计算液压板功耗；
4.计算转向板功耗；
5.计算24V总功耗；
*/  //采样55ms的频率时时间常数是1，采样500ms的频率时间常数是9
_iq W_SumText = 0;
void Energy_Consumption ( void )
{
//    Energy_Ctrl.U = _IQ ( 24 );
//    Energy_Ctrl.I = _IQdiv ( ADC_Value_Num.ADC_Ctrl_24V_I, ADC_Sens_30A );
//    Energy_Ctrl.T = 9;              //9.1 = 0.5s*(1/3600*65535)
//    Energy_Ctrl.P = _IQmpy ( Energy_Ctrl.U, Energy_Ctrl.I );
//    Energy_Ctrl.W_Step = _IQmpy ( Energy_Ctrl.P, Energy_Ctrl.T );
//    Energy_Ctrl.W_Sum += Energy_Ctrl.W_Step;
    /*textile*/
    Energy_PC.U = SampleVolt ( ADC_Value_Num.ADC_PC_24V_V );
    Energy_PC.I = SampleCurr ( ADC_Value_Num.ADC_PC_24V_I - ADC_Value_Num.ADC_2_5V_REF, POWERCURR_RATIO );

    Energy_SENSOR.U = _IQrmpy ( SampleVolt ( ADC_Value_Num.ADC_SENSOR_24V_V ), _IQ ( 0.9 ) ) + _IQrmpy ( Energy_SENSOR.U, _IQ ( 0.1 ) );
    Energy_SENSOR.I = SampleCurr ( ADC_Value_Num.ADC_SENSOR_24V_I - ADC_Value_Num.ADC_2_5V_REF, POWERCURR_RATIO );

    Energy_MOTOR1.U = SampleVolt ( ADC_Value_Num.ADC_MOTOR1_24V_V );
    Energy_MOTOR1.I = SampleCurr ( ADC_Value_Num.ADC_MOTOR1_24V_I - ADC_Value_Num.ADC_2_5V_REF, POWERCURR_RATIO );

    Energy_MOTOR2.U = SampleVolt ( ADC_Value_Num.ADC_MOTOR2_24V_V );
    Energy_MOTOR2.I = SampleCurr ( ADC_Value_Num.ADC_MOTOR2_24V_I - ADC_Value_Num.ADC_2_5V_REF, POWERCURR_RATIO );

    Energy_ElecMag.I = SampleVolt ( ADC_Value_Num.ADC_M_I_MCU );
    if ( ADC_Value_Num.ADC_M_I_MCU < 200 )
    {
        ElecMagSta = 0;
    }
    else if ( ADC_Value_Num.ADC_M_I_MCU > 1500 )
    {
        ElecMagSta = 1;
    }
    /*textile*/
    Energy_PC.T = 1;              //0.91 = 0.05s*(1/3600*65535)
    Energy_PC.P = _IQmpy ( Energy_PC.U, Energy_PC.I );
    Energy_PC.W_Step = _IQmpy ( Energy_PC.P, Energy_PC.T );
    Energy_PC.W_Sum += Energy_PC.W_Step;

    Energy_SENSOR.T = 1;              //0.91 = 0.05s*(1/3600*65535)
    Energy_SENSOR.P = _IQmpy ( Energy_SENSOR.U, Energy_SENSOR.I );
    Energy_SENSOR.W_Step = _IQmpy ( Energy_SENSOR.P, Energy_SENSOR.T );
    Energy_SENSOR.W_Sum += Energy_SENSOR.W_Step;

    Energy_MOTOR1.T = 1;              //0.91 = 0.05s*(1/3600*65535)
    Energy_MOTOR1.P = _IQmpy ( Energy_MOTOR1.U, Energy_MOTOR1.I );
    Energy_MOTOR1.W_Step = _IQmpy ( Energy_MOTOR1.P, Energy_MOTOR1.T );
    Energy_MOTOR1.W_Sum += Energy_MOTOR1.W_Step;

    Energy_MOTOR2.T = 1;              //0.91 = 0.05s*(1/3600*65535)
    Energy_MOTOR2.P = _IQmpy ( Energy_MOTOR2.U, Energy_MOTOR2.I );
    Energy_MOTOR2.W_Step = _IQmpy ( Energy_MOTOR2.P, Energy_MOTOR2.T );
    Energy_MOTOR2.W_Sum += Energy_MOTOR2.W_Step;



//    Energy_VALVE.U = _IQmpy ( ADC_Value_Num.ADC_VALVE_V, 772 * 65535 );
//    Energy_VALVE.I = _IQdiv ( ADC_Value_Num.ADC_VALVE_I, ADC_Sens_10A );
//    Energy_VALVE.T = 9;              //9.1 = 0.5s*(1/3600*65535)
//    Energy_VALVE.P = _IQmpy ( Energy_VALVE.U, Energy_VALVE.I );
//    Energy_VALVE.W_Step = _IQmpy ( Energy_VALVE.P, Energy_VALVE.T );
//    Energy_VALVE.W_Sum += Energy_VALVE.W_Step;

//    Energy_BLDC.U = _IQmpy ( ADC_Value_Num.ADC_BLDC_V, 772 * 65535 );
//    Energy_BLDC.I = _IQdiv ( ADC_Value_Num.ADC_BLDC_I, ADC_Sens_30A );
//    Energy_BLDC.T = 9;              //9.1 = 0.5s*(1/3600*65535)
//    Energy_BLDC.P = _IQmpy ( Energy_BLDC.U, Energy_BLDC.I );
//    Energy_BLDC.W_Step = _IQmpy ( Energy_BLDC.P, Energy_BLDC.T );
//    Energy_BLDC.W_Sum += Energy_BLDC.W_Step;

//    Energy_BAT.U = _IQmpy ( ADC_Value_Num.ADC_BAT_24V, 772 * 65535 );
//    Energy_BAT.I = _IQdiv ( ADC_Value_Num.ADC_BAT_I, ADC_Sens_30A );
//    Energy_BAT.T = 9;              //9.1 = 0.5s*(1/3600*65535)
//    Energy_BAT.P = _IQmpy ( Energy_BAT.U, Energy_BAT.I );
//    Energy_BAT.W_Step = _IQmpy ( Energy_BAT.P, Energy_BAT.T );
//    Energy_BAT.W_Sum += Energy_BAT.W_Step;

//    Energy_FORK_24V.U = _IQmpy ( ADC_Value_Num.ADC_FORK_24V, 772 * 65535 );
//    Energy_FORK_24V.I = _IQdiv ( ADC_Value_Num.ADC_FORK_I, ADC_Sens_30A );
//    Energy_FORK_24V.T = 9;              //9.1 = 0.5s*(1/3600*65535)
//    Energy_FORK_24V.P = _IQmpy ( Energy_FORK_24V.U, Energy_FORK_24V.I );
//    Energy_FORK_24V.W_Step = _IQmpy ( Energy_FORK_24V.P, Energy_FORK_24V.T );
//    Energy_FORK_24V.W_Sum += Energy_FORK_24V.W_Step;
//    W_SumText = Energy_Ctrl.W_Sum + Energy_PC.W_Sum + Energy_VALVE.W_Sum + Energy_BAT.W_Sum;
}


//车速异常检测
void SpeedErrCheck ( void )
{
//    if((m_eRunState == RunSta_Run_R)||(m_eRunState == RunSta_Run_D))
//    {
//        if((abs(g_qCurWSpeed) - m_qTargetSpeed*1.75 ) > 0.5*m_qTargetSpeed)  //超速判定
//        {
//            OverSpeedCnt++;
//        }
//        else
//        {
//           OverSpeedCnt = 0;
//        }
//        if(((m_qTargetSpeed*1.75 - abs(g_qCurWSpeed) ) > 0.5*m_qTargetSpeed)&&(VCU_Speed>200))
//        {
//           LowSpeedCnt++;
//        }
//        else
//        {
//          LowSpeedCnt = 0;
//        }
//    }
//    else
//    {
//        OverSpeedCnt = 0;
//        LowSpeedCnt = 0;
//    }
}



//上位机开关机//
/*
* @brief       上位机开关机函数(没用)
* @param       无
* @retval      无
*
*/
uint8_t Switch_Flag = 0, PC_Key_State, PC_Key_Switch, PC_State;
uint16_t Switch_Cnt = 0, PC_Key_Cnt = 0;
#define PowerOn_ed      0
#define ShutDown_ing    1
#define ShutDown_ed     2
#define PowerOn_ing     3
#if 0
void PC_ON_OFF ( void )
{
    DI_ReadByIndex ( &PC_Key_Switch, DI_CH_KEY1 );   //读取PC开关机按钮状态
    DI_ReadByIndex ( &PC_State, DI_CH_PC_5V );      //读取PC开关机状态 （电压）
    if ( ( PC_Key_Switch ) && ( PC_Key_State != 1 ) ) //按键1s后认定为真实触发
    {
        PC_Key_Cnt++;
        if ( PC_Key_Cnt > 100 )
        {
            PC_Key_State = 1;
        }
    }
    else
    {
        PC_Key_Cnt = 0;
    }

    if ( ( PC_Key_State ) && ( Switch_Flag == PowerOn_ed ) ) //按下按钮，处于开机状态，则关机
    {
        Switch_Flag = ShutDown_ing;
    }
    else if ( ( PC_Key_State ) && ( Switch_Flag == ShutDown_ed ) ) //按下按钮，处于关机状态，则开机
    {
        Switch_Flag = PowerOn_ing;
        DoSetPower_SW3 ( ON );                  //PC开电
    }
    else
    {
        PC_Key_State = 0;
    }
    if ( Switch_Flag == ShutDown_ing )                 //关机
    {

        HostShutDowncmd   = 0x11;
        Switch_Cnt++;
        if ( ( Switch_Cnt < 1500 ) && ( PC_State == 0 ) ) //15s
        {
            Switch_Flag = ShutDown_ed;
            Switch_Cnt = 0;
            DoSetPower_SW3 ( OFF );                  //PC关电
        }
        else if ( Switch_Cnt > 1500 )
        {
            //超时
        }
    }
    else if ( Switch_Flag == PowerOn_ing )             //开机
    {
        Switch_Cnt++;
        HostShutDowncmd   = 0;
        if ( ( Switch_Cnt < 1500 ) && ( PC_State == 1 ) ) //15s
        {
            Switch_Flag = PowerOn_ed;
            Switch_Cnt = 0;
        }
        else if ( Switch_Cnt > 1500 )
        {
            //超时
        }
    }
    else
    {
        Switch_Cnt = 0;
    }

    if ( PC_State )
    {
        DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_KEY1_LED );     //开机则亮灯
    }
    else
    {
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_KEY1_LED );     //关机则灭灯
    }

}
#endif
/*
*函数介绍 ：尾部避障雷达避障检测函数
*参数     ：无
*返回值   ：无
*备注     ：配置雷达避障区域，检测雷达输出，执行规避策略
*/
uint16_t Zone = 0;
uint16_t StopFlag = 0, LastStopFlag = 0;
uint16_t ret = 8;
_iq Angle = 0;
float angle_R = 0, angle_L = 0;
RunStates nDriveMode = RunSta_Idle;
void Radar_Process ( void )
{
//    Angle = _IQdiv ( g_u16SteeringWheelAbsPos, 32767 );
//    if ( Angle > 32767 ) angle_R = ( float ) ( ( Angle - 32767 ) * 180 / 32767 );
//    else angle_L = ( float ) ( ( 32767 - Angle ) * 180 / 32767 );

//    Radar_Zone_Left = SelectLeftRadarZone ();          //左雷达区域选择
//    Radar_Zone_Right = SelectRightRadarZone();         //右雷达区域选择

//    DoSetRadar ( &Radar_Zone_Left, &Radar_Zone_Right);
//    RadarResult = ( RadarOut_R >= RadarOut_L ) ? RadarOut_R : RadarOut_L;
//    RaderReport = RadarResult;

//    nDriveMode = GetRUNSta();//从状态机取当前挡位信息

    RadarResult = RegReal_RadarRev.u8RadarInf;
    switch ( RadarResult )
    {
    case RadarOut1_SlowDown:
    {   //减速区
        StopFlag = 1;
        STOP_0_Ctrl ( OFF );
    }
    break;
    case RadarOut2_SoftStop:
    {   //停车区
        StopFlag = 2;
        STOP_0_Ctrl ( OFF );
    }
    break;
    case RadarOut3_EmerStop:
    {   //急停区（暂时按停车区处理，或者增加踩刹车制动）
        StopFlag = 3;
    }
    break;
    case RadarOut_Null:
    {   //无障碍物
        StopFlag = 0;
        STOP_0_Ctrl ( OFF );
    }
    break;

    default:
    {

    } break;
    }
}
/*
*函数介绍 ：配置雷达避障区域
*参数     ：Spd_1m:_IQ(1)=1m/S,Angle:IQ(0.5)=180度= 转向轮回正
*返回值   ：ret：0x01=配置1=(Z1=1，Z2=Z3=Z4=0)
*备注     ：
*/
uint16_t SelectLeftRadarZone ( void )
{

    if ( ( _IQ ( 0.497222222 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.502777778 ) ) ) //-1 - 1
    {
        ret = 1;
    }
    else if ( ( _IQ ( 0.491666667 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.497222222 ) ) ) //-3 - -1
    {
        ret = 2;
    }
    else if ( ( _IQ ( 0.483333333 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.497222222 ) ) ) // -6 - -3
    {
        ret = 3;
    }
    else if ( ( _IQ ( 0.475 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.483333333 ) ) ) //-9 - -6
    {
        ret = 4;
    }
    else if ( ( _IQ ( 0.466666667 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.475 ) ) ) //-12 - -9
    {
        ret = 5;
    }
    else if ( ( _IQ ( 0.458333333 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.466666667 ) ) ) //-15 - -12
    {
        ret = 6;
    }
    else if ( ( _IQ ( 0.45 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.458333333 ) ) ) //-18 - -15
    {
        ret = 7;
    }
    else if ( ( _IQ ( 0.441666667 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.45 ) ) ) //-21 - -18
    {
        ret = 8;
    }
    else if ( ( _IQ ( 0.433333333 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.441666667 ) ) ) //-24 - -21
    {
        ret = 9;
    }
    else if ( ( _IQ ( 0.425 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.433333333 ) ) ) //-27 - -24
    {
        ret = 10;
    }
    else if ( ( _IQ ( 0.416666667 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.425 ) ) ) //-30 - -27
    {
        ret = 11;
    }
    else if ( ( _IQ ( 0.405555556 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.416666667 ) ) ) //-34 - -30
    {
        ret = 12;
    }
    else if ( ( _IQ ( 0.394444444 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.405555556 ) ) ) //-38 - -34
    {
        ret = 13;
    }
    else if ( ( _IQ ( 0.383333333 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.394444444 ) ) ) //-42 - -38
    {
        ret = 14;
    }
    else if ( ( _IQ ( 0.2 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.383333333 ) ) ) //-46 - -38
    {
        ret = 15;
    }
    else if ( _IQabs ( Angle ) > _IQ ( 0.502777778 ) )
    {
        ret = 16;
    }
    if ( nDriveMode == RunSta_Run_D )
    {
        ret = 16;
    }
    return  ret;
}

uint16_t SelectRightRadarZone ( void )
{
    if ( ( _IQ ( 0.497222222 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.502777778 ) ) ) //-1 - 1
    {
        ret = 1;
    }
    else if ( ( _IQ ( 0.502777778 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.508333333 ) ) ) //1 - 3
    {
        ret = 2;
    }
    else if ( ( _IQ ( 0.508333333 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.516666667 ) ) ) //3 - 6
    {
        ret = 3;
    }
    else if ( ( _IQ ( 0.516666667 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.525 ) ) ) //6 - 9
    {
        ret = 4;
    }
    else if ( ( _IQ ( 0.525 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.533333333 ) ) ) //9 - 12
    {
        ret = 5;
    }
    else if ( ( _IQ ( 0.533333333 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.541666667 ) ) ) //12 - 15
    {
        ret = 6;
    }
    else if ( ( _IQ ( 0.541666667 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.55 ) ) ) //15 - 18
    {
        ret = 7;
    }
    else if ( ( _IQ ( 0.55 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.558333333 ) ) ) //18 - 21
    {
        ret = 8;
    }
    else if ( ( _IQ ( 0.558333333 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.566666667 ) ) ) //21 - 24
    {
        ret = 9;
    }
    else if ( ( _IQ ( 0.566666667 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.575 ) ) ) //24 - 27
    {
        ret = 10;
    }
    else if ( ( _IQ ( 0.575 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.583333333 ) ) ) //27 - 30
    {
        ret = 11;
    }
    else if ( ( _IQ ( 0.583333333 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.594444444 ) ) ) //30 - 34
    {
        ret = 12;
    }
    else if ( ( _IQ ( 0.594444444 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.60555555 ) ) ) //34 - 38
    {
        ret = 13;
    }
    else if ( ( _IQ ( 0.60555555 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.616666667 ) ) ) //38-42
    {
        ret = 14;
    }
    else if ( ( _IQ ( 0.616666667 ) < _IQabs ( Angle ) ) && ( _IQabs ( Angle ) <= _IQ ( 0.8 ) ) ) //38-42
    {
        ret = 15;
    }
    else if ( _IQabs ( Angle ) < _IQ ( 0.497222222 ) )
    {
        ret = 16;
    }
    if ( nDriveMode == RunSta_Run_D )
    {
        ret = 16;
    }
    return ret;
}
//获取雷达检测结果
static RadarOut GetRadarOut ( void )
{
    RadarOut ret;
    return ret;
}
/*
* @brief       灯控制函数
* @param       无
* @retval      无
*
*/
void  LightStateCheck ( void )     //灯光状态 1是绿灯常亮，2是绿灯闪烁，3是黄灯常亮，4是黄红闪烁，5是红灯常亮
{
    if ( FaultCtrl.all || ( RegReal_RotatingMBDFaultStaFB.u32Fault != 0 ) || ( RegReal_TelescopicMBDFaultStaFB.u32Fault != 0 ) \
            || ( RegReal_HuggingMBDFaultStaFB.u32Fault != 0 ) || ( RegReal_GlandMBDFaultStaFB.u32Fault != 0 ) || ( Reg_AGVDataFb1.u8WalkFaultCode != 0 ) \
            || ( Reg_AGVPumpCtrFb.u8WalkFaultCode != 0 ) || ( Reg_AGVBMSStaOrigDataFb.u8FaultCode != 0 ) )
    {
        if ( FaultCtrl.bit.EmergencyStop == 1 ) //2类急停触发
        {
            LightState = 4;
//            RGBMode1 = RGBMode_RY_BLINK;//红黄闪烁
        }
        else
        {
            LightState = 5;
//            RGBMode1 = RGBMode_R;
        }
    }
    else if ( ( Warning_Report.all ) || ( RegReal_RotatingMBDFaultStaFB.u32Warning != 0 ) || ( RegReal_TelescopicMBDFaultStaFB.u32Warning != 0 ) \
              || ( RegReal_HuggingMBDFaultStaFB.u32Warning != 0 ) || ( RegReal_GlandMBDFaultStaFB.u32Warning != 0 ) || ( StopFlag == 1 ) ) //减速区StopFlag == 1
    {
        LightState = 3;
//        RGBMode1 = RGBMode_Y;
    }
//    else if ( ( m_eRunState == RunSta_Run_D ) || ( m_eRunState == RunSta_Run_R ) ) //前进后退
//    {
////        LightState = 2;
////        RGBMode1 = RGBMode_G_Live;////绿呼吸
//    }
//    else if ( ( m_eRunState == RunSta_Idle ) || ( m_eRunState == RunSta_Run_N ) )
//    {
//        LightState = 1;
//        RGBMode1 = RGBMode_G;
//    }
    else if ( Reg_AGVBMSStaOrigDataFb.Soc < 20 )
    {
        LightState = 6;
    }
    else
    {
        LightState = 1;//绿灯
    }
    if ( ( StopFlag == 2 ) ) //停车区
    {
        LightState = 4;
    }
    else if ( StopFlag == 3 ) //急停区
    {
        LightState = 5;
    }
    else if ( PowerOnFlag == 2 )
    {
        LightState = 0;
    }

//    if ( ( StopFlag == 3 ) || ( PC_Rader_Stop == 3 ) )           //后避障雷达灯光
//    {
//        LightState = 5;
//    }
//    else if ( ( StopFlag == 1 ) || ( StopFlag == 2 ) || ( PC_Rader_Stop == 1 ) || ( PC_Rader_Stop == 2 ) )
//    {
//        LightState = 3;
//    }

//    if(m_PowerSwCmd.PowerSwOpt.bit.PcPowerSw == 0)
//    {
//        LightState = 0;
//    }


    switch ( LightState )   //灯光状态 1是绿灯常亮，2是绿灯闪烁，3是黄灯常亮，4是黄红闪烁，5是红灯常亮
    {
    case 0://空闲
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_G );     //
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_R );
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_Y );
        LightCnt = 0;
        break ;
    case 1://空闲
        DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_LED_G );     //绿灯常亮
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_R );
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_Y );
        LightCnt = 0;
        break ;

    case 2://前进后退
        LightCnt++;
        if ( LightCnt % 1000 < 500 )              //绿灯闪烁2hz
        {
            DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_LED_G );
            DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_R );
            DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_Y );
        }
        else
        {
            DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_G );
            DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_R );
            DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_Y );
        }

        break ;

    case 3: //减速区                                            //黄灯常亮
        DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_LED_R );
        DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_LED_G );
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_Y );

        break ;

    case 4://2类急停触发
        LightCnt++;
        if ( LightCnt % 1000 < 500 )              //红黄灯闪烁2hz
        {
            DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_LED_R );
            DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_G );
            DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_Y );
        }
        else
        {
            DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_R );
            DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_G );
            DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_LED_Y );

        }
        //Buzzer_Beep(200,300);
        break ;

    case 5:// //红灯常亮
        DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_LED_R );
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_G );
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_Y );
        //Buzzer_Beep(200,300);
        LightCnt = 0;
        break ;
    case 6://电量不足
        LightCnt++;
        if ( LightCnt % 1000 < 500 )              //红灯闪烁2hz
        {
            DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_LED_R );
            DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_G );
            DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_Y );
        }
        else
        {
            DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_R );
            DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_G );
            DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_Y );

        }
        //Buzzer_Beep(200,300);
        break ;
    }


    switch ( m_sSound_LigntCmd.Ctrl_ID )
    {
    case 1:
        if ( m_sSound_LigntCmd.Switch == 1 )
        {
            RegReal_PartsAndValve1Param.u8PartSwitch.bit.Headlights = 1;
        }
        else
        {
            RegReal_PartsAndValve1Param.u8PartSwitch.bit.Headlights = 0;
        }
        break;

    case 2:
        PC_Rader_Stop = m_sSound_LigntCmd.Switch;      //对避障雷达灯光赋值
        break;

    case 3:
        if ( m_sSound_LigntCmd.Switch == 1 )
        {
            Light_State.bit.HighLight = 1;
        }
        else
        {
            Light_State.bit.HighLight = 0;
        }
        break;

    case 4:
        if ( m_sSound_LigntCmd.Switch == 1 )
        {
            Light_State.bit.LowLight = 1;
        }
        else
        {
            Light_State.bit.LowLight = 0;
        }
        break;

    case 5:
        if ( m_sSound_LigntCmd.Switch == 1 )
        {
            Light_State.bit.MidLight = 1;
        }
        else
        {
            Light_State.bit.MidLight = 0;
        }

        break;

    case 0x10:                                //灯光全关
        Light_State.bit.HighLight = 0;
        Light_State.bit.MidLight = 0;
        Light_State.bit.LowLight = 0;
        RegReal_PartsAndValve1Param.u8PartSwitch.bit.Headlights = 0;
        break;

    case 0x11:                                //灯光全开
        Light_State.bit.HighLight = 1;
        Light_State.bit.MidLight = 1;
        Light_State.bit.LowLight = 1;
        RegReal_PartsAndValve1Param.u8PartSwitch.bit.Headlights = 1;
        break;
    }



    if ( m_sSound_LigntCmd.Ctrl_ID == 1 )       //补光灯控制
    {
        if ( m_sSound_LigntCmd.Switch == 1 )
        {
            RegReal_PartsAndValve1Param.u8PartSwitch.bit.Headlights = 1;
        }
        else
        {
            RegReal_PartsAndValve1Param.u8PartSwitch.bit.Headlights = 0;
        }
    }
}
/*
* @brief       RGB灯控制函数---没用
* @param       无
* @retval      无
*
*/
void RGBCtrl()
{
    LightPWMCnt++;
    if ( LightPWMCnt % 5 == 0 )
    {
        Ctrl_SameColour ( RGBMode1 );
    }
}

static uint8_t KeySta ( void )
{
    GPIO_PinState Pin_LEDKey;
    static uint8_t u8KeyFlg = 0;
    static uint32_t u32KeyCnt = 0;
    uint8_t ret;

    ret = 0;
    DI_ReadByIndex ( &Pin_LEDKey, DI_CH_VID2 );
    if ( Pin_LEDKey == 1 )
    {
        if ( u32KeyCnt < 10 )
        {
            u32KeyCnt++;
        }
        else {
            u8KeyFlg = 1;   //消抖检测
        }
    } else {
        if ( u32KeyCnt > 1 )
        {
            u32KeyCnt--;
        } else {
            if ( u8KeyFlg == 1 ) {
                ret = 1;
                u8KeyFlg = 0;
            }
        }
    }
    return ret;
}


/*汉字16*16每行最多8个，字符16*8每行最多16个*/
#define Size        16
#define Line1       0
#define Line2       Line1 + Size + 0
#define Line3       Line2 + Size + 0
#define Line4       Line3 + Size + 0

#define ShowT       300
#define ShowT_Halt  6000
OLED_Display ShowIndex = OLED_Start;
uint16_t a = 0, b = 0, c = 0; //软件版本号，abc代表版本的三个标志
#if 0
void Monitordisplay ( void )
{
    static uint32_t u32Cnt10mS = 10;
    extern unsigned char Logo[];
    char StrBuf[33];

    if ( 1 == KeySta() )
    {
        if ( u32Cnt10mS > ShowT )
        {
            ShowIndex++;    //翻到下一屏
        }
        u32Cnt10mS = ShowT_Halt;
    }
    if ( u32Cnt10mS > 0 ) {
        u32Cnt10mS--;
    }
    switch ( ShowIndex )
    {
    case OLED_Start:
    {
        if ( u32Cnt10mS == 0 ) {
            u32Cnt10mS = ShowT;
            ShowIndex++;
        }
    }
    break;
    case OLED_Logo:
    {
        if ( u32Cnt10mS % 100 == 99 )
        {
            OLED_Clear();
            OLED_ShowPicture ( 0, 0, 128, 64, Logo, 1 );
            OLED_Refresh ();
        }
        if ( u32Cnt10mS == 0 ) {
            u32Cnt10mS = ShowT;
            ShowIndex++;
        }
    }
    break;
    case OLED_CopInfo:
    {
        if ( u32Cnt10mS % 100 == 99 )
        {
            OLED_Clear();
            OLED_ShowChinese ( 0, 0, 0, 32, 1 );
            OLED_ShowChinese ( 32, 0, 1, 32, 1 );
            OLED_ShowChinese ( 64, 0, 2, 32, 1 );
            OLED_ShowChinese ( 96, 0, 3, 32, 1 );//木牛科技
            OLED_ShowChinese ( 0, 32, 0, 16, 1 );
            OLED_ShowChinese ( 18, 32, 1, 16, 1 );
            OLED_ShowChinese ( 36, 32, 2, 16, 1 );
            OLED_ShowChinese ( 54, 32, 3, 16, 1 );
            OLED_ShowChinese ( 72, 32, 4, 16, 1 );
            OLED_ShowChinese ( 90, 32, 5, 16, 1 );
            OLED_ShowChinese ( 108, 32, 6, 16, 1 );//木牛叉车机器人
            OLED_ShowString ( 6, 48, "MNew_OS_Inside", 16, 1 );
            OLED_Refresh ();
        }
        if ( u32Cnt10mS == 0 ) {
            u32Cnt10mS = ShowT;
            ShowIndex++;
        }
    }
    break;
    case OLED_CtrlVer:
    {
        if ( u32Cnt10mS % 100 == 99 )
        {
            a =  ParamHandle_Control.Reg_SW_Ver / 1000000;
            b = ( ParamHandle_Control.Reg_SW_Ver / 1000 ) % 1000;
            c =  ParamHandle_Control.Reg_SW_Ver % 1000;
            OLED_Clear();
            OLED_ShowChinese ( 0, 0, 7, 16, 1 );
            OLED_ShowChinese ( 18, 0, 8, 16, 1 );
            OLED_ShowChinese ( 36, 0, 9, 16, 1 );//下位机

            sprintf ( StrBuf, "SN :%010u", ParamHandle_Control.Reg_SN );
            OLED_ShowString ( 0, Line2, StrBuf, Size, 1 );
            sprintf ( StrBuf, "HW :V%u.0", ParamHandle_Control.Reg_HW_Ver );
            OLED_ShowString ( 0, Line3, StrBuf, Size, 1 );
            sprintf ( StrBuf, "V: %u.%u.%u", a, b, c );     //版本
            OLED_ShowString ( 0, Line4, StrBuf, Size, 1 );
            OLED_Refresh ();
        }
        if ( u32Cnt10mS == 0 ) {
            u32Cnt10mS = ShowT;
            ShowIndex++;
        }

    }
    break;
    case OLED_SolenoidVer:
    {
        if ( u32Cnt10mS % 100 == 99 )
        {
            a =  ParamHandle_Solenoid.Reg_SW_Ver / 1000000;
            b = ( ParamHandle_Solenoid.Reg_SW_Ver / 1000 ) % 1000;
            c =  ParamHandle_Solenoid.Reg_SW_Ver % 1000;

            OLED_Clear();
            OLED_ShowChinese ( 0, 0, 10, 16, 1 );
            OLED_ShowChinese ( 18, 0, 11, 16, 1 );
            OLED_ShowChinese ( 36, 0, 12, 16, 1 );//液压板
            sprintf ( StrBuf, "SN :%010u", ParamHandle_Solenoid.Reg_SN );
            OLED_ShowString ( 0, Line2, StrBuf, Size, 1 );
            sprintf ( StrBuf, "HW :V%u.0", ParamHandle_Solenoid.Reg_HW_Ver );
            OLED_ShowString ( 0, Line3, StrBuf, Size, 1 );
            sprintf ( StrBuf, "V: %u.%u.%u", a, b, c );     //版本
            OLED_ShowString ( 0, Line4, StrBuf, Size, 1 );
            OLED_Refresh ();
        }
        if ( u32Cnt10mS == 0 ) {
            u32Cnt10mS = ShowT;
            ShowIndex++;
        }
    }
    break;
    case OLED_SteerVer:
    {
        if ( u32Cnt10mS % 100 == 99 )
        {
            a =  ParamHandle_Steer.Reg_SW_Ver / 1000000;
            b = ( ParamHandle_Steer.Reg_SW_Ver / 1000 ) % 1000;
            c =  ParamHandle_Steer.Reg_SW_Ver % 1000;
            OLED_Clear();
            OLED_ShowChinese ( 0, 0, 13, 16, 1 );
            OLED_ShowChinese ( 18, 0, 14, 16, 1 );
            OLED_ShowChinese ( 36, 0, 15, 16, 1 );//转向板
            sprintf ( StrBuf, "SN :%010u", ParamHandle_Steer.Reg_SN );
            OLED_ShowString ( 0, Line2, StrBuf, Size, 1 );
            sprintf ( StrBuf, "HW :V%u.0", ParamHandle_Steer.Reg_HW_Ver );
            OLED_ShowString ( 0, Line3, StrBuf, Size, 1 );
            sprintf ( StrBuf, "V: %u.%u.%u", a, b, c );     //版本
            OLED_ShowString ( 0, Line4, StrBuf, Size, 1 );
            OLED_Refresh ();
        }
        if ( u32Cnt10mS == 0 ) {
            u32Cnt10mS = ShowT;
            ShowIndex++;
        }
    }
    break;

    case OLED_BoardInfo:
    {

        if ( u32Cnt10mS % 100 == 99 )
        {
            OLED_Clear();
            Temperature = Get_Temperature();
            V_24 = Get_BAT_24V();
            V_80 = Get_E_Stop_80V();
            Motor_24 = Get_Motor_24V();
            sprintf ( StrBuf, "Tem:%3d.%02d", Temperature / 100, Temperature % 100 );
            OLED_ShowString ( 0, Line1, StrBuf, Size, 1 );
            OLED_ShowChinese ( 80, Line1, 33, 16, 1 );//单位℃
            sprintf ( StrBuf, "HVo:%3d.%dV,%d%%", V_80 / 100, V_80 % 100, 90 );
            OLED_ShowString ( 0, Line2, StrBuf, Size, 1 );
            sprintf ( StrBuf, "CTL:%3d.%02dV", V_24 / 100, V_24 % 100 );
            OLED_ShowString ( 0, Line3, StrBuf, Size, 1 );
            sprintf ( StrBuf, "Mot:%3d.%02dV", Motor_24 / 100, Motor_24 % 100 );
            OLED_ShowString ( 0, Line4, StrBuf, Size, 1 );
            OLED_Refresh ();
        }
        if ( u32Cnt10mS == 0 ) {
            u32Cnt10mS = ShowT;
            ShowIndex++;
        }
    }
    break;

    case OLED_ErrCode:
    {
        if ( u32Cnt10mS % 100 == 99 )
        {
            OLED_Clear();

            sprintf ( StrBuf, "Error Code" );
            OLED_ShowString ( 0, Line1, StrBuf, Size, 1 );
            sprintf ( StrBuf, "Ctrl:0x%08x", FaultCtrl.all );
            OLED_ShowString ( 0, Line2, StrBuf, Size, 1 );
//            sprintf ( StrBuf, "PRod:0x%08x", Fault_Handle1.Fault_Solenoid.all );
            OLED_ShowString ( 0, Line3, StrBuf, Size, 1 );
//            sprintf ( StrBuf, "Ster:0x%08x", Fault_Handle1.Fault_Steer.all );
            OLED_ShowString ( 0, Line4, StrBuf, Size, 1 );
            OLED_Refresh ();
        }
        if ( u32Cnt10mS == 0 ) {
            u32Cnt10mS = ShowT;
            ShowIndex++;
        }
    }
    break;

    case OLED_LiftSta:
    {
        if ( u32Cnt10mS % 100 == 99 )
        {
            OLED_Clear();

            sprintf ( StrBuf, "LiftState" );
            OLED_ShowString ( 0, Line1, StrBuf, Size, 1 );
            sprintf ( StrBuf, "TPos:0x%08x", SolenoidCtrlCmd.Lift.unPos );
            OLED_ShowString ( 0, Line2, StrBuf, Size, 1 );
            sprintf ( StrBuf, "NPos:0x%08x", g_sSolenoid_Info1[RodID_Lift].Pos );
            OLED_ShowString ( 0, Line3, StrBuf, Size, 1 );
            sprintf ( StrBuf, "state:0x%08x", SolenoidReport.SolenoidEndOfCtrl );
            OLED_ShowString ( 0, Line4, StrBuf, Size, 1 );
            OLED_Refresh ();
        }
        if ( u32Cnt10mS == 0 ) {
            u32Cnt10mS = ShowT;
            ShowIndex++;
        }
    }
    break;
    case OLED_ClawSta:
    {
        if ( u32Cnt10mS % 100 == 99 )
        {
            OLED_Clear();

            sprintf ( StrBuf, "ClawState" );
            OLED_ShowString ( 0, Line1, StrBuf, Size, 1 );
            sprintf ( StrBuf, "TPos:0x%08x", SolenoidCtrlCmd.Claw.unPos );
            OLED_ShowString ( 0, Line2, StrBuf, Size, 1 );
            sprintf ( StrBuf, "NPos:0x%08x", SolenoidReport.unClampOpenDegree );
            OLED_ShowString ( 0, Line3, StrBuf, Size, 1 );
            sprintf ( StrBuf, "state:0x%08x", SolenoidReport.SolenoidEndOfCtrl );
            OLED_ShowString ( 0, Line4, StrBuf, Size, 1 );
            OLED_Refresh ();
        }
        if ( u32Cnt10mS == 0 ) {
            u32Cnt10mS = ShowT;
            ShowIndex++;
        }
    }
    break;

    case OLED_DriveSta:
    {
        if ( u32Cnt10mS % 100 == 99 )
        {
            OLED_Clear();

            OLED_ShowChinese ( 0, Line1, 16, 16, 1 );
            OLED_ShowChinese ( 16, Line1, 17, 16, 1 );//油门
            sprintf ( StrBuf, ":%3d%% %3d%%", 99, 99 );
            OLED_ShowString ( 32, Line1, StrBuf, Size, 1 );
            OLED_ShowChinese ( 0, Line2, 18, 16, 1 );
            OLED_ShowChinese ( 16, Line2, 19, 16, 1 );//刹车
            sprintf ( StrBuf, ":%3d%% %3d%%", 99, 99 );
            OLED_ShowString ( 32, Line2, StrBuf, Size, 1 );

            OLED_ShowChinese ( 0, Line3, 20, 16, 1 );
            OLED_ShowChinese ( 16, Line3, 21, 16, 1 );//档位
            sprintf ( StrBuf, ":  %c    %c", 'D', 'R' );
            OLED_ShowString ( 32, Line3, StrBuf, Size, 1 );
            OLED_ShowChinese ( 0, Line4, 22, 16, 1 );
            OLED_ShowChinese ( 16, Line4, 23, 16, 1 );//转向
            sprintf ( StrBuf, ": %3d  %3d", 270, 320 );
            OLED_ShowString ( 32, Line4, StrBuf, Size, 1 );
            OLED_ShowChinese ( 72, Line4, 32, 16, 1 );//单位°
            OLED_ShowChinese ( 112, Line4, 32, 16, 1 );//单位°

            OLED_Refresh ();
        }
        if ( u32Cnt10mS == 0 ) {
            u32Cnt10mS = ShowT;
            ShowIndex++;
        }
    }
    break;

    case OLED_SolenoidInfo:
    {

        if ( u32Cnt10mS % 100 == 99 )
        {
            OLED_Clear();

            OLED_ShowChinese ( 0, Line1, 24, 16, 1 );
            OLED_ShowChinese ( 16, Line1, 25, 16, 1 );//升降
            sprintf ( StrBuf, ":%d.%dm %d.%dm", 1, 21, 1, 20 );
            OLED_ShowString ( 32, Line1, StrBuf, Size, 1 );
            OLED_ShowChinese ( 0, Line2, 26, 16, 1 );
            OLED_ShowChinese ( 16, Line2, 27, 16, 1 );//抱夹
            sprintf ( StrBuf, ":%d.%dm %d.%dm", 0, 21, 0, 20 );
            OLED_ShowString ( 32, Line2, StrBuf, Size, 1 );

            OLED_ShowChinese ( 0, Line3, 28, 16, 1 );
            OLED_ShowChinese ( 16, Line3, 29, 16, 1 );//俯仰
            sprintf ( StrBuf, ":%d.%dm %d.%dm", 0, 21, 0, 20 );
            OLED_ShowString ( 32, Line3, StrBuf, Size, 1 );
            OLED_ShowChinese ( 0, Line4, 30, 16, 1 );
            OLED_ShowChinese ( 16, Line4, 31, 16, 1 );//旋转
            sprintf ( StrBuf, ": %3d  %3d", 270, 320 );
            OLED_ShowString ( 32, Line4, StrBuf, Size, 1 );
            OLED_ShowChinese ( 72, Line4, 32, 16, 1 );//单位°
            OLED_ShowChinese ( 112, Line4, 32, 16, 1 );//单位°

            OLED_Refresh ();
        }
        if ( u32Cnt10mS == 0 ) {
            u32Cnt10mS = ShowT;
            ShowIndex++;
        }
    }
    break;

    default:
    {
        ShowIndex = OLED_CtrlVer;
    }
    break;
    }

}
#endif
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


