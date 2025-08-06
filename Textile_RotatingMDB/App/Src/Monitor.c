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
* xxxx/xx/xx | 1.0.0 | MuNiu | 创建文件
*
*/
//------------------------------------------------------------------------------

//-------------------- include files ----------------------------------------
#include "Monitor.h"
#include "DataBaseProcess.h"
#include "HostProcess.h"
#include "ADC.h"
#include "TimerPWM.h"
#include "MotorInfo.h"
#include "Foc.h"
#include "MotorCmd_Ctrl.h"
#include "main.h"
#include "CoderTama.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
static int16_t Steer_Vs_Temp[][2] =  //NTC SDNT1608X103F3450FTF
{
    -3000,  279,
    -2500,  363,
    -2000,  464,
    -1500,  585,
    -1000,  724,
    -500,   881,
    0,      1055,
    500,    1242,
    1000,   1439,
    1500,   1642,
    2000,   1847,
    2500,   2048,
    3000,   2243,
    3500,   2428,
    4000,   2601,
    4500,   2762,
    5000,   2908,
    5500,   3040,
    6000,   3159,
    6500,   3265,
    7000,   3359,
    7500,   3443,
    8000,   3516,
    8500,   3581,
    9000,   3638,
    9500,   3687,
    10000,  3731,
    10500,  3770,
    11000,  3804,
    11500,  3834,
    12000,  3860,
    12500,  3884,
};

//-------------------- private functions declare ----------------------------
static int LackUVWCheck ( void );
static int SpdCircleCheck ( void );
static int LodeOverCheck ( void );
static int CtrlCmdDogCheck ( void );
static int ReangleLackCheck ( void );
static int RotateCoderLackCheck ( void );
static int CoderTama1LackCheck ( void );
static int CoderTama2LackCheck ( void );
static void VoltFaultCheck ( void );
static void PressureCheck ( uint16_t EmStop_State, S_MOTOR_INFO *psInfo );
static int TemperatureCheck ( uint16_t TempValue );
static int16_t Get_Temperature ( void );
static int MotorAxleOffsetInit ( void );
static _iq GetCoderAngle ( _iq Angle_Ref, _iq _iqIqLoseLim );
static void CoderCaliCheck ( WheelAngle CoderHandle );
static void SetLowPressure_Warn ( void );
static void SetVolUnder_Warn ( void );
static void SetMotorAxleInvalid_Warn ( void );
static void SetCoderCali_Warn ( void );

//-------------------- public data ------------------------------------------
Fault_REG g_u32FaultNum;
Warning_REG g_u32WarningNum;
uint8_t Foc_EN_Flag = 0;
uint32_t FaultCheckEnable = 0xFFFFFFFF;
uint32_t WarnCheckEnable = 0xFFFFFFFF;
int16_t s16CtrlLife = FullLife_500ms;
int16_t s16ReangleLife = FullLife_100ms;
int16_t s16RotateCoderLife = FullLife_2s;
int16_t s16CoderTama1Life = FullLife_2s;
int16_t s16CoderTama2Life = FullLife_2s;
uint8_t u8VoltOverCnt = 0;

//-------------------- public functions -------------------------------------
/**
 * @brief       获取故障位号
 * @param       无
 * @retval      无
 */
uint16_t FaultNum_Get ( Fault_REG FaultNum )
{
    uint8_t BitCnt;
    uint16_t LEDFaultNum = 0;
    static uint8_t Lockbit = 0;

    if ( FaultNum.all ) {
        for ( BitCnt = Lockbit; BitCnt < 32; BitCnt++ )
        {
            if ( FaultNum.all & BIT ( BitCnt ) ) {
                LEDFaultNum = BitCnt + 1;
                Lockbit = LEDFaultNum;
                break;
            }
        }
        if ( BitCnt == 32 ) { //最后1位仍没有跳出
            for ( BitCnt = 0; BitCnt < 32; BitCnt++ )
            {
                if ( FaultNum.all & BIT ( BitCnt ) )
                {
                    LEDFaultNum = BitCnt + 1;
                    Lockbit = LEDFaultNum;
                    break;
                }
            }
        }
    } else {
        LEDFaultNum = 0;
    }
    return LEDFaultNum;
}
/**
 * @brief       初始化检测
 * @param       无
 * @retval      无
 */
void MonitorInit ( void )
{
    FaultCheckEnable = 0x0003FCF;  //上电自检
    WarnCheckEnable = 0x0000007F;
    /************************电机出轴编码器离线检测****************************/
    if ( ( FaultCheckEnable & BIT ( 2 ) ) )
    {
        g_u32FaultNum.bit.uLackUVW = LackUVWCheck ( );
    }
    else
    {
//        g_u32FaultNum.bit.uLackUVW = 0;
    }
    if ( g_u32FaultNum.bit.uLackUVW == 0 )
    {
        if ( MotorAxleOffsetInit ( ) == 0 )  //电机0点初始化
        {
            Foc_EN_Flag = 1;  //初始化成功/已经初始化完成，才允许执行FOC
        }
        else
        {
            SetMotorAxleInvalid_Warn ( );
        }
    }
}
/**
 * @brief       状态检测
 * @param       无
 * @retval      无
 */
void MonitorProcess ( void )
{
    uint16_t u16Flag_LodeOver = 0, u16Cnt_LodeOver = 0;  //过载检测
   
    /************************电机出轴编码器离线检测****************************/
    if ( ( FaultCheckEnable & BIT ( 3 ) ) && ( u8Standby_Flag ) )
    {
        if ( 0 == g_u32FaultNum.bit.uReangleLack )
        {
            g_u32FaultNum.bit.uReangleLack = ReangleLackCheck ( );
        }
        else
        {
            ;
        }
    }
    else
    {
        g_u32FaultNum.bit.uReangleLack = 0;
    }
    /************************旋转码盘离线检测****************************/
    if ( ( FaultCheckEnable & BIT ( 13 ) ) && ( u8Standby_Flag ) )
    {
        if ( 0 == g_u32FaultNum.bit.uRotateCoderLack )
        {
            g_u32FaultNum.bit.uRotateCoderLack = RotateCoderLackCheck ( );
        }
        else
        {
            ;
        }
    }
    else
    {
        g_u32FaultNum.bit.uReangleLack = 0;
    }
    /************************后轮编码器1离线检测****************************/
    if ( ( FaultCheckEnable & BIT ( 4 ) ) && ( u8Standby_Flag ) )
    {
        if ( 0 == g_u32FaultNum.bit.uCoderTama1_Lack )
        {
            g_u32FaultNum.bit.uCoderTama1_Lack = CoderTama1LackCheck ( );
        }
        else
        {
            ;
        }
    }
    else
    {
        g_u32FaultNum.bit.uCoderTama1_Lack = 0;
    }
    /************************后轮编码器2离线检测****************************/
    if ( ( FaultCheckEnable & BIT ( 5 ) ) && ( u8Standby_Flag ) )
    {
        if ( 0 == g_u32FaultNum.bit.uCoderTama2_Lack )
        {
            g_u32FaultNum.bit.uCoderTama2_Lack = CoderTama2LackCheck ( );
        }
        else
        {
            ;
        }
    }
    else
    {
        g_u32FaultNum.bit.uCoderTama2_Lack = 0;
    }
    /************************下位机心跳检测****************************/
    if ( ( FaultCheckEnable & BIT ( 6 ) ) && ( u8Standby_Flag ) )
    {
        if ( 0 == g_u32FaultNum.bit.uCmdDogLose )
        {
//            g_u32FaultNum.bit.uCmdDogLose = CtrlCmdDogCheck ( );
        }
        else
        {
            ;
        }
    }
    else
    {
        g_u32FaultNum.bit.uCmdDogLose = 0;
    }
    /************************速度环跟踪不良检测(取消)****************************/
    /*if ( ( FaultCheckEnable & BIT ( 7 ) ) && ( Foc_EN_Flag ) )
    {
        if ( 0 == g_u32FaultNum.bit.uSpdCircle )
        {
            g_u32FaultNum.bit.uSpdCircle = SpdCircleCheck ( );
        }
        else
        {
            ;
        }
    }
    else
    {
        g_u32FaultNum.bit.uSpdCircle = 0;
    }*/
    /************************电机过载检测****************************/
    if ( ( FaultCheckEnable & BIT ( 8 ) ) && ( Foc_EN_Flag ) )
    {
        if ( g_u32FaultNum.bit.uSeveralLodeOver == 0 )
        {
            g_u32FaultNum.bit.uSeveralLodeOver = LodeOverCheck ( );
        }
    }
//    if ( ( WarnCheckEnable & BIT ( 4 ) ) && ( Foc_EN_Flag ) )
//    {
//        g_u32WarningNum.bit.uLodeOver = LodeOverCheck ( );
//    }
//    if ( ( FaultCheckEnable & BIT ( 8 ) ) && ( Foc_EN_Flag ) )
//    {
//        if ( (  g_u32WarningNum.bit.uLodeOver ) && ( 0 == u16Flag_LodeOver ) )
//        {
//            u16Flag_LodeOver = 1;
//            u16Cnt_LodeOver++;
//        }
//        else if ( 0 == g_u32WarningNum.bit.uLodeOver )
//        {
//            u16Flag_LodeOver = 0;
//        }
//        else
//        {
//            ;
//        }
//        if ( ( 0 == g_u32FaultNum.bit.uSeveralLodeOver ) && ( u16Cnt_LodeOver >= 3 ) )
//        {
//           g_u32FaultNum.bit.uSeveralLodeOver = 1; 
//        }
//        else if ( g_u32FaultNum.bit.uSeveralLodeOver )
//        {
//            u16Cnt_LodeOver = 0;
//        }
//        else
//        {
//            ;
//        }
//    }
    /************************过温检测****************************/
    if ( ( WarnCheckEnable & BIT ( 5 ) ) && ( u8Standby_Flag ) ) 
    {
        if ( 0 == g_u32WarningNum.bit.uTempWarnning ) 
        {
            g_u32WarningNum.bit.uTempWarnning = TemperatureCheck ( HighTempWarnValue );
        }
    }
    if ( ( FaultCheckEnable & BIT ( 9 ) ) && ( u8Standby_Flag ) )
    {
        if ( ( 0 == g_u32FaultNum.bit.uTempOver ) && ( g_u32WarningNum.bit.uTempWarnning ) )
        {
            g_u32FaultNum.bit.uTempOver = TemperatureCheck ( HighTempErrValue );
        }
        else
        {
            ;
        }
    }
    else
    {
        g_u32FaultNum.bit.uTempOver = 0;
    }
    VoltFaultCheck ( );  //动力电检测
//    if ( WarnCheckEnable & BIT ( 2 ) ) CoderCaliCheck ( WheelAngleHandle1 );  //码盘标定检测
}
/**
 * @brief       电机出轴编码器离线检测
 * @param       无
 * @retval      返回电机出轴编码器是否离线
 */
static int ReangleLackCheck ( void )
{
    uint8_t res = NoFault;

    if ( s16ReangleLife > 0 ) s16ReangleLife--;
    if ( s16ReangleLife == 0 )
    {
        res = Fault;
    }
    else
    {
//        res = NoFault;
    }

    return res;
}
/**
 * @brief       旋转码盘离线检测
 * @param       无
 * @retval      返回旋转码盘是否离线
 */
static int RotateCoderLackCheck ( void )
{
    uint8_t res = NoFault;

    if ( s16RotateCoderLife > 0 ) s16RotateCoderLife--;
    if ( s16RotateCoderLife == 0 )
    {
        res = Fault;
    }
    else
    {
//        res = NoFault;
    }

    return res;
}
/**
 * @brief       后轮编码器1离线检测
 * @param       无
 * @retval      返回后轮编码器1是否离线
 */
static int CoderTama1LackCheck ( void )
{
    uint8_t res = NoFault;

    if ( s16CoderTama1Life > 0 ) s16CoderTama1Life--;
    if ( s16CoderTama1Life == 0 )
    {
        res = Fault;
    }
    else
    {
//        res = NoFault;
    }

    return res;
}
/**
 * @brief       后轮编码器2离线检测
 * @param       无
 * @retval      返回后轮编码器2是否离线
 */
static int CoderTama2LackCheck ( void )
{
    uint8_t res = NoFault;

    if ( s16CoderTama2Life > 0 ) s16CoderTama2Life--;
    if ( s16CoderTama2Life == 0 )
    {
        res = Fault;
    }
    else
    {
//        res = NoFault;
    }

    return res;
}
/**
 * @brief       下位机心跳检测
 * @param       无
 * @retval      返回下位机是否失联
 */
static int CtrlCmdDogCheck ( void )
{
    uint8_t res = NoFault;

    if ( s16CtrlLife > 0 ) s16CtrlLife--;
    if ( s16CtrlLife == 0 )
    {
        res = Fault;
    }
    else
    {
//        res = NoFault;
    }

    return res;
}
/**
 * @brief       动力电检测
 * @param       无
 * @retval      无
 */
static void VoltFaultCheck ( void )
{
    static uint16_t u16VoltValue = 0;
    static uint8_t u8VoltUnderCnt = 0;

    u16VoltValue = V_SUPPLY_A2D ( ADC_GetValue ( ADC_CH_V_IN ) );

    if ( V_SUPPLY_Max < u16VoltValue )
    {
        u8VoltUnderCnt = 0;
        u8VoltOverCnt++;
        if ( u8VoltOverCnt > 100 )  //100ms
        {
            if ( ( FaultCheckEnable & BIT ( 10 ) ) ) g_u32FaultNum.bit.uVolOver = 1;
            g_sMotor_Info.u16MotorVol_State = 0;
        }
    }
    else if ( V_SUPPLY_Min > u16VoltValue )
    {
        u8VoltOverCnt = 0;
        u8VoltUnderCnt++;
        if ( u8VoltUnderCnt > 100 )  //100ms
        {
            if ( WarnCheckEnable & BIT ( 1 ) ) SetVolUnder_Warn ( );
            g_sMotor_Info.u16MotorVol_State = 0;
        }
    }
    else
    {
        u8VoltOverCnt = 0;
        u8VoltUnderCnt = 0;
        g_sMotor_Info.u16MotorVol_State = 1;
    }

//    if ( V_SUPPLY_Min < u16VoltValue )
//    {
//        g_u32WarningNum.bit.uVolUnder = NoFault;  //低压告警恢复
//    }
}
/**
 * @brief       油压检测
 * @param       电机信息
 * @retval      无
 */
static void PressureCheck ( uint16_t EmStop_State, S_MOTOR_INFO *psInfo )
{
    static uint16_t u16Cnt_EmStop = 0, u16Flag_EmStop = 0;
    
    if ( EmStop_State == 0 ) {
        if ( u16Cnt_EmStop < 3000 ) u16Cnt_EmStop++;
        else u16Flag_EmStop = 1;
    } else {
       u16Cnt_EmStop = 0;
       u16Flag_EmStop = 0;        
    }
    if ( ( 0 == psInfo->u16Pressure_State ) && ( u16Flag_EmStop ) )
    {
        SetLowPressure_Warn ( );
    }
}
/**
 * @brief       缺相检测
 * @param       无
 * @retval      返回是否缺相
 */
static int LackUVWCheck ( void )
{
    uint8_t res = NoFault, u8VoltCheckCnt = 0, Delay_1ms = 1, Delay_5ms = 5;
    uint16_t u16VoltValue = 0;
    uint16_t Cnt[3];
    _iq _iqIqLoseLim;

    _iqIqLoseLim = _IQdiv4 ( _IQdiv ( ParamHandle_Steer.Reg_RatedCurrent, 100 ) );

    for ( int i = 0; i < 10; i++ )
    {
        u16VoltValue = V_SUPPLY_A2D ( ADC_GetValue ( ADC_CH_V_IN ) );
        if ( V_SUPPLY_Min < u16VoltValue ) u8VoltCheckCnt++;
        HAL_Delay ( Delay_1ms );
    }

    if ( u8VoltCheckCnt == 10 )
    {
        TimerPWM_ModeSet ( PWM_MODE_OFF );
        TimerPWM_DutySet ( _IQ ( 0 ), _IQ ( 0 ), _IQ ( 0 ) );
        TimerPWM_ModeSet ( PWM_MODE_RUN );

        for ( Cnt[0] = 1; Cnt[0] < 30; Cnt[0]++ )
        {
            TimerPWM_DutySet ( _IQ ( 0.1 ) + _IQrmpy ( _IQ ( 0.01 ), _IQ ( Cnt[0] ) ), _IQ ( 0 ), _IQ ( 0 ) ); //U相缺相检测
            HAL_Delay ( Delay_5ms );

            if ( _IQabs ( g_sMotor_Info.iqCurrentA ) > _iqIqLoseLim ) break;
        }
        TimerPWM_DutySet ( _IQ ( 0 ), _IQ ( 0 ), _IQ ( 0 ) );

        HAL_Delay ( 100 );
        for ( Cnt[1] = 1; Cnt[1] < 30; Cnt[1]++ )
        {
            TimerPWM_DutySet ( _IQ ( 0 ), _IQ ( 0.1 ) + _IQrmpy ( _IQ ( 0.01 ), _IQ ( Cnt[1] ) ), _IQ ( 0 ) ); //V相缺相检测
            HAL_Delay ( Delay_5ms );

            if ( _IQabs ( g_sMotor_Info.iqCurrentB ) > _iqIqLoseLim ) break;
        }
        TimerPWM_DutySet ( _IQ ( 0 ), _IQ ( 0 ), _IQ ( 0 ) );

        HAL_Delay ( 100 );
        for ( Cnt[2] = 1; Cnt[2] < 30; Cnt[2]++ )
        {
            TimerPWM_DutySet ( _IQ ( 0 ), _IQ ( 0 ), _IQ ( 0.1 ) + _IQrmpy ( _IQ ( 0.01 ), _IQ ( Cnt[2] ) ) ); //W相缺相检测
            HAL_Delay ( Delay_5ms );

            if ( _IQabs ( g_sMotor_Info.iqCurrentC ) > _iqIqLoseLim ) break;
        }
        TimerPWM_ModeSet ( PWM_MODE_OFF );
        TimerPWM_DutySet ( _IQ ( 0 ), _IQ ( 0 ), _IQ ( 0 ) );

        if ( ( Cnt[0] == 30 ) || ( Cnt[1] == 30 ) || ( Cnt[2] == 30 ) )
        {
            res = Fault;
        }
    }
    else
    {
//        res = NoFault;
    }

    return res;
}
/**
 * @brief       速度环跟踪不良检测
 * @param       无
 * @retval      返回速度环是否跟踪不良
 */
static int SpdCircleCheck ( void )
{
    extern PI_CONTROLLER pi_spd;
    uint8_t res = NoFault;
    static uint16_t u16SpdCircleCnt = 0;

    if ( ( g_sMotorCmd.eCmdMode >= CmdMode_PosCtrl ) && ( g_sMotorCmd.eCmdMode < CmdMode_TorqueCtrl ) )
    {
//        if ( NormalSpdLoseCtrl || ZeroSpdLoseCtrl || u8StopFalg )
        if ( NormalSpdLoseCtrl || ZeroSpdLoseCtrl )
        {
            u16SpdCircleCnt++;
            if ( u16SpdCircleCnt > 2000 )  //2s
            {
                res = Fault;
            }
        }
        else
        {
            u16SpdCircleCnt = 0;
        }
    }
    else
    {
        u16SpdCircleCnt = 0;
    }

    return res;
}
/**
 * @brief       电机过载检测
 * @param       无
 * @retval      返回电机运行是否过载
 */
static int LodeOverCheck ( void )
{
    uint8_t res = NoFault;
    static uint32_t u32LodeOverCnt = 0;
    _iq _iqIqLoseLim;

    _iqIqLoseLim = _IQdiv ( ParamHandle_Steer.Reg_RatedCurrent * 1.4, 100 );
    if ( g_sMotor_Info.iqEffectCurrent > _iqIqLoseLim )
    {
        u32LodeOverCnt++;
        if ( u32LodeOverCnt > 5000 ) //5000ms
        {
            res = Fault;
            u32LodeOverCnt = 0;
        }
    }
    else
    {
        if ( u32LodeOverCnt > 0 ) u32LodeOverCnt--;
    }

    return res;
}
/**
 * @brief       E2P报错
 * @param       无
 * @retval      无
 * @note        未做上电E2P写入/读取自检，采用使用过程中校验方式
 */
void SetE2prom_Fault ( void )
{
    if ( ( FaultCheckEnable & BIT ( 0 ) ) )
    {
        g_u32FaultNum.bit.uE2promErr = Fault;
    }
}
/**
 * @brief       数据库版本报错
 * @param       无
 * @retval      无
 */
void SetDBVersion_Fault ( void )
{
    if ( ( FaultCheckEnable & BIT ( 1 ) ) )
    {
        g_u32FaultNum.bit.uDBVersion = Fault;
    }
}
/**
 * @brief       数据库版本报错
 * @param       无
 * @retval      无
 */
void SetBoardUpdata_Fault ( void )
{
    if ( ( FaultCheckEnable & BIT ( 11 ) ) )
    {
        g_u32FaultNum.bit.uBoardUpdata = Fault;
    }
}
/**
 * @brief       油压告警
 * @param       无
 * @retval      无
 * @note        无
 */
static void SetLowPressure_Warn ( void )
{
    if ( ( WarnCheckEnable & BIT ( 0 ) ) )
    {
        g_u32WarningNum.bit.uLowPressure = Fault;
    }
}
/**
 * @brief       低压告警
 * @param       无
 * @retval      无
 * @note        无
 */
static void SetVolUnder_Warn ( void )
{
    if ( ( WarnCheckEnable & BIT ( 1 ) ) )
    {
        g_u32WarningNum.bit.uVolUnder = Fault;
    }
}
/**
 * @brief       电机未标定告警
 * @param       无
 * @retval      无
 * @note        无
 */
static void SetMotorAxleInvalid_Warn ( void )
{
    if ( ( WarnCheckEnable & BIT ( 2 ) ) )
    {
        g_u32WarningNum.bit.uMotorAxleInvalid = Fault;
    }
}
/**
 * @brief       左右码盘不匹配告警
 * @param       无
 * @retval      无
 * @note        无
 */
static void SetCoderCali_Warn ( void )
{
    if ( ( WarnCheckEnable & BIT ( 3 ) ) )
    {
        g_u32WarningNum.bit.uCoderCaliWarnning = Fault;
    }
}
/**
 * @brief       温度检测函数
 * @param       无
 * @retval      返回值为是否过温
 */
static int TemperatureCheck ( uint16_t TempValue )
{
    uint8_t res = NoFault;
    int16_t s16Temperature = 0;
    static uint16_t u16HighTempCnt = 0;

    s16Temperature = Get_Temperature();
    if ( s16Temperature > TempValue )
    {
        u16HighTempCnt++;
        if ( u16HighTempCnt > 2000 )
        {
            res = Fault;
            u16HighTempCnt = 0;
        }
    }
    else
    {
        u16HighTempCnt = 0;
//        res = NoFault;
    }

    return res;
}
/**
 * @brief       获取温度值
 * @param       无
 * @retval      返回值精确到0.01摄氏度
 */
static int16_t Get_Temperature ( void )
{
    int i;
    uint16_t TSpm;
    int16_t s16Temperature;

    TSpm = ADC_GetValue ( ADC_CH_TMPE );

    if ( TSpm < Steer_Vs_Temp[0][1] )
    {
        s16Temperature = -4000;
    }
    else if ( TSpm > Steer_Vs_Temp[31][1] )
    {
        s16Temperature = 12500;
    }
    else
    {
        for ( i = 1; i < 34; i++ )
        {
            if ( TSpm < Steer_Vs_Temp[i][1] )
            {
                s16Temperature = Steer_Vs_Temp[i][0] - _IQrmpy ( 500, _IQdiv ( ( Steer_Vs_Temp[i][1] - TSpm ), ( Steer_Vs_Temp[i][1] - Steer_Vs_Temp[i - 1][1] ) ) );
                break;
            }
        }
    }
    return s16Temperature;
}
/**
 * @brief       电机0点初始化
 * @param       无
 * @retval      0:标定成功/标定完成，1:标定失败/未标定
 */
static int MotorAxleOffsetInit ( void )
{
    uint8_t u8VoltCheckCnt = 0, Delay_1ms = 1, Delay_30ms = 30;
    uint16_t u16VoltValue = 0;
    _iq Angle_0, Angle_90, Angle_180, Angle_270, Angle_360, Angle_0a, Angle_90a, Angle_180a, Angle_270a, AngleDiff;
    _iq _iqIqLoseLim;

    if ( abs ( ParamHandle_Steer.Reg_ReverseFlag ) == 1 ) return 0;  //标定完成

    _iqIqLoseLim = _IQdiv ( ParamHandle_Steer.Reg_RatedCurrent, 100 );

    for ( int i = 0; i < 3; i++ )
    {
        u16VoltValue = V_SUPPLY_A2D ( ADC_GetValue ( ADC_CH_V_IN ) );
        if ( V_SUPPLY_Min < u16VoltValue ) u8VoltCheckCnt++;
        HAL_Delay ( Delay_1ms );
    }

    if ( u8VoltCheckCnt == 3 )
    {
        TimerPWM_ModeSet ( PWM_MODE_OFF );
        TimerPWM_DutySet ( _IQ ( 0 ), _IQ ( 0 ), _IQ ( 0 ) );
        TimerPWM_ModeSet ( PWM_MODE_RUN );
        HAL_Delay ( Delay_30ms );

        //电机轴码盘0点偏移，初始值65535，初始化程序检测到65535则进行电机0点初始化
        GetCoderAngle ( _IQ ( 0.75 ), _iqIqLoseLim ); //起点
        Angle_0             = GetCoderAngle ( _IQ ( 0 ), _iqIqLoseLim );
        Angle_90            = GetCoderAngle ( _IQ ( 0.25 ), _iqIqLoseLim );
        Angle_180            = GetCoderAngle ( _IQ ( 0.5 ), _iqIqLoseLim );
        Angle_270            = GetCoderAngle ( _IQ ( 0.75 ), _iqIqLoseLim );
        Angle_360            = GetCoderAngle ( _IQ ( 0 ), _iqIqLoseLim );
        Angle_270a            = GetCoderAngle ( _IQ ( 0.75 ), _iqIqLoseLim );
        Angle_180a            = GetCoderAngle ( _IQ ( 0.5 ), _iqIqLoseLim );
        Angle_90a            = GetCoderAngle ( _IQ ( 0.25 ), _iqIqLoseLim );
        Angle_0a             = GetCoderAngle ( _IQ ( 0 ), _iqIqLoseLim );

        TimerPWM_DutySet ( _IQ ( 0 ), _IQ ( 0 ), _IQ ( 0 ) );
        TimerPWM_ModeSet ( PWM_MODE_OFF );

        AngleDiff = Angle_90 - Angle_0;
        if ( abs ( AngleDiff ) < _IQ ( 0.05 ) ) return 1 ;
        Angle_IQ0_IQ1 ( &AngleDiff );

        if ( AngleDiff > _IQ ( 0.5 ) )
        {
            ParamHandle_Steer.Reg_ReverseFlag = -1;                //0.75为反向
        }
        else
        {
            ParamHandle_Steer.Reg_ReverseFlag = 1;                //0.25为同向
        }

        //求取偏移
        //(EAng-Offset)*Reverse = Ref  => Offset = Eang - Reverse*Ref
        Angle_90            -= ParamHandle_Steer.Reg_ReverseFlag * _IQ ( 0.25 );
        Angle_IQ0_IQ1 ( &Angle_90 );
        Angle_180            -= ParamHandle_Steer.Reg_ReverseFlag * _IQ ( 0.5 );
        Angle_IQ0_IQ1 ( &Angle_180 );
        Angle_270            -= ParamHandle_Steer.Reg_ReverseFlag * _IQ ( 0.75 );
        Angle_IQ0_IQ1 ( &Angle_270 );

        Angle_90a            -= ParamHandle_Steer.Reg_ReverseFlag * _IQ ( 0.25 );
        Angle_IQ0_IQ1 ( &Angle_90a );
        Angle_180a            -= ParamHandle_Steer.Reg_ReverseFlag * _IQ ( 0.5 );
        Angle_IQ0_IQ1 ( &Angle_180a );
        Angle_270a            -= ParamHandle_Steer.Reg_ReverseFlag * _IQ ( 0.75 );
        Angle_IQ0_IQ1 ( &Angle_270a );

        Angle_IQ0_IQ1_Avg ( &Angle_0a, &Angle_90a, &Angle_0a );
        Angle_IQ0_IQ1 ( &Angle_0a );
        Angle_IQ0_IQ1_Avg ( &Angle_180a, &Angle_270a, &Angle_180a );
        Angle_IQ0_IQ1 ( &Angle_180a );
        Angle_IQ0_IQ1_Avg ( &Angle_180a, &Angle_0a, &Angle_0a );
        Angle_IQ0_IQ1 ( &Angle_0a );

        Angle_IQ0_IQ1_Avg ( &Angle_0, &Angle_90, &Angle_0 );
        Angle_IQ0_IQ1 ( &Angle_0 );
        Angle_IQ0_IQ1_Avg ( &Angle_180, &Angle_270, &Angle_180 );
        Angle_IQ0_IQ1 ( &Angle_180 );
        Angle_IQ0_IQ1_Avg ( &Angle_0, &Angle_360, &Angle_0 );
        Angle_IQ0_IQ1 ( &Angle_0 );
        Angle_IQ0_IQ1_Avg ( &Angle_0, &Angle_180, &Angle_0 );
        Angle_IQ0_IQ1 ( &Angle_0 );

        Angle_IQ0_IQ1_Avg ( &Angle_0a, &Angle_0, &Angle_0 );
        Angle_IQ0_IQ1 ( &Angle_0 );

        if ( Angle_0 >= _IQ ( 1 ) ) Angle_0 = 0;
        ParamHandle_Steer.Reg_MotorAxleOffset = Angle_0;

        if ( Param2Eeprom ( ParamHandle_Steer ) == 0 ) return 0;
        else
        {
            SetE2prom_Fault ( );  //写入失败
            return 1;
        }
    }
    else
    {
        return 1;
    }
}
/**
 * @brief       给固定电角度
 * @param       Angle_Ref：目标电角度，_iqIqLoseLim：判定电流稳定阈值
 * @retval      出轴编码器反馈值
 */
static _iq GetCoderAngle ( _iq Angle_Ref, _iq _iqIqLoseLim )
{
    _iq Angle_Fbk, Diff;
    uint16_t i, j;

    TimerPWM_DutySet ( _IQ ( 0 ), _IQ ( 0 ), _IQ ( 0 ) );
    HAL_Delay ( 50 );

    for ( i = 1; i < 600; i++ )
    {

        SetVector ( _IQrmpy ( _IQ ( 0.001 ), _IQ ( i ) ), Angle_Ref ); //90度

        HAL_Delay ( 5 );                             //等待电流稳定

        if ( ( _IQabs ( g_sMotor_Info.iqCurrentA ) > _iqIqLoseLim ) || ( _IQabs ( g_sMotor_Info.iqCurrentB ) > _iqIqLoseLim ) ||
                ( _IQabs ( g_sMotor_Info.iqCurrentC ) > _iqIqLoseLim ) )
        {

            Angle_Fbk            = g_sMotor_Info.iqEAngle;
            HAL_Delay ( 500 );        //等待转子稳定
            for ( j = 1; j < 100; j++ )
            {
                HAL_Delay ( 1 );
                Diff                = g_sMotor_Info.iqEAngle - Angle_Fbk; //防止0度附近采样造成失败

                if ( abs ( Diff ) < _IQ ( 0.5 ) )
                    Angle_Fbk = _IQrmpy ( Angle_Fbk, _IQ ( 0.95 ) ) + _IQrmpy ( g_sMotor_Info.iqEAngle, _IQ ( 0.05 ) );
            }

            break;
        }
    }

    return Angle_Fbk;
}
/**
 * @brief       检测后轮码盘标定结果在使用过程中是否异常
 * @param       无
 * @retval      无
 */
uint16_t u16Cnt_Warnning = 0, u16Cnt_Left = 0, u16Cnt_Right = 0;
_iq iqCoderErr = 0, iqMiddleValue = _IQ( 0.00278 ), iqWarnningValue = _IQ( 0.00834 );
static void CoderCaliCheck ( WheelAngle CoderHandle )
{
//    static uint16_t u16Cnt_Warnning = 0, u16Cnt_Left = 0, u16Cnt_Right = 0;
//    static _iq iqCoderErr = 0, iqMiddleValue = _IQ( 0.00278 ), iqWarnningValue = _IQ( 0.00834 );
    
    iqCoderErr = CoderHandle._iqLeft - CoderHandle._iqRight;
    Angle_IQ0_5 ( &iqCoderErr );
    if ( 0 == g_u32WarningNum.bit.uCoderCaliWarnning )
    {
        if ( abs ( CoderHandle._iqLeft ) < iqMiddleValue )  //小于±1°
        {
            u16Cnt_Left = 0;
            if ( abs ( iqCoderErr ) > iqWarnningValue )  //大于±3°
            {
                if ( u16Cnt_Right++ > 100 )
                {
                    SetCoderCali_Warn ( );
                }
            }
            else
            {
//                g_u32WarningNum.bit.uCoderCaliWarnning = 0;
                u16Cnt_Right = 0;
            }
        }
        else if ( abs ( CoderHandle._iqRight ) < iqMiddleValue )  //小于±1°
        {
            u16Cnt_Right = 0;
            if ( abs ( iqCoderErr ) > iqWarnningValue )  //大于±3°
            {
                if ( u16Cnt_Left++ > 100 )
                {
                    SetCoderCali_Warn ( );
                }
            }
            else
            {
//                g_u32WarningNum.bit.uCoderCaliWarnning = 0;
                u16Cnt_Left = 0;
            }
        }
        else
        {
//            g_u32WarningNum.bit.uCoderCaliWarnning = 0;
            u16Cnt_Left = 0;
            u16Cnt_Right = 0;
        }
    }
    else
    {
        u16Cnt_Left = 0;
        u16Cnt_Right = 0;
    }
}
/**
 * @brief       故障消除逻辑处理
 * @param       无
 * @retval      无
 */
void FaultClearProcess ( void )
{
    static uint16_t u16LastSN_FaultClear;
    
    if ( RegReal_FaultClearCmd.u16SN != u16LastSN_FaultClear )
    {
        u16LastSN_FaultClear = RegReal_FaultClearCmd.u16SN;
        if ( RegReal_FaultClearCmd.u16EN )
        {
            g_u32FaultNum.all = 0;
            g_u32WarningNum.all = 0;
            
            s16ReangleLife = FullLife_100ms;
            s16CoderTama1Life = FullLife_2s;
            s16CoderTama2Life = FullLife_2s;
            s16CtrlLife = FullLife_500ms;
            
            if ( g_u16FultClearStep == 1 ) g_u16FultClearStep = 2;
        }
    }
}
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


