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
* xxxx/xx/xx | 1.0.0 | MuNiu | �����ļ�
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
 * @brief       ��ȡ����λ��
 * @param       ��
 * @retval      ��
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
        if ( BitCnt == 32 ) { //���1λ��û������
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
 * @brief       ��ʼ�����
 * @param       ��
 * @retval      ��
 */
void MonitorInit ( void )
{
    FaultCheckEnable = 0x0003FCF;  //�ϵ��Լ�
    WarnCheckEnable = 0x0000007F;
    /************************���������������߼��****************************/
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
        if ( MotorAxleOffsetInit ( ) == 0 )  //���0���ʼ��
        {
            Foc_EN_Flag = 1;  //��ʼ���ɹ�/�Ѿ���ʼ����ɣ�������ִ��FOC
        }
        else
        {
            SetMotorAxleInvalid_Warn ( );
        }
    }
}
/**
 * @brief       ״̬���
 * @param       ��
 * @retval      ��
 */
void MonitorProcess ( void )
{
    uint16_t u16Flag_LodeOver = 0, u16Cnt_LodeOver = 0;  //���ؼ��
   
    /************************���������������߼��****************************/
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
    /************************��ת�������߼��****************************/
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
    /************************���ֱ�����1���߼��****************************/
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
    /************************���ֱ�����2���߼��****************************/
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
    /************************��λ���������****************************/
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
    /************************�ٶȻ����ٲ������(ȡ��)****************************/
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
    /************************������ؼ��****************************/
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
    /************************���¼��****************************/
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
    VoltFaultCheck ( );  //��������
//    if ( WarnCheckEnable & BIT ( 2 ) ) CoderCaliCheck ( WheelAngleHandle1 );  //���̱궨���
}
/**
 * @brief       ���������������߼��
 * @param       ��
 * @retval      ���ص������������Ƿ�����
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
 * @brief       ��ת�������߼��
 * @param       ��
 * @retval      ������ת�����Ƿ�����
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
 * @brief       ���ֱ�����1���߼��
 * @param       ��
 * @retval      ���غ��ֱ�����1�Ƿ�����
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
 * @brief       ���ֱ�����2���߼��
 * @param       ��
 * @retval      ���غ��ֱ�����2�Ƿ�����
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
 * @brief       ��λ���������
 * @param       ��
 * @retval      ������λ���Ƿ�ʧ��
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
 * @brief       ��������
 * @param       ��
 * @retval      ��
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
//        g_u32WarningNum.bit.uVolUnder = NoFault;  //��ѹ�澯�ָ�
//    }
}
/**
 * @brief       ��ѹ���
 * @param       �����Ϣ
 * @retval      ��
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
 * @brief       ȱ����
 * @param       ��
 * @retval      �����Ƿ�ȱ��
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
            TimerPWM_DutySet ( _IQ ( 0.1 ) + _IQrmpy ( _IQ ( 0.01 ), _IQ ( Cnt[0] ) ), _IQ ( 0 ), _IQ ( 0 ) ); //U��ȱ����
            HAL_Delay ( Delay_5ms );

            if ( _IQabs ( g_sMotor_Info.iqCurrentA ) > _iqIqLoseLim ) break;
        }
        TimerPWM_DutySet ( _IQ ( 0 ), _IQ ( 0 ), _IQ ( 0 ) );

        HAL_Delay ( 100 );
        for ( Cnt[1] = 1; Cnt[1] < 30; Cnt[1]++ )
        {
            TimerPWM_DutySet ( _IQ ( 0 ), _IQ ( 0.1 ) + _IQrmpy ( _IQ ( 0.01 ), _IQ ( Cnt[1] ) ), _IQ ( 0 ) ); //V��ȱ����
            HAL_Delay ( Delay_5ms );

            if ( _IQabs ( g_sMotor_Info.iqCurrentB ) > _iqIqLoseLim ) break;
        }
        TimerPWM_DutySet ( _IQ ( 0 ), _IQ ( 0 ), _IQ ( 0 ) );

        HAL_Delay ( 100 );
        for ( Cnt[2] = 1; Cnt[2] < 30; Cnt[2]++ )
        {
            TimerPWM_DutySet ( _IQ ( 0 ), _IQ ( 0 ), _IQ ( 0.1 ) + _IQrmpy ( _IQ ( 0.01 ), _IQ ( Cnt[2] ) ) ); //W��ȱ����
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
 * @brief       �ٶȻ����ٲ������
 * @param       ��
 * @retval      �����ٶȻ��Ƿ���ٲ���
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
 * @brief       ������ؼ��
 * @param       ��
 * @retval      ���ص�������Ƿ����
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
 * @brief       E2P����
 * @param       ��
 * @retval      ��
 * @note        δ���ϵ�E2Pд��/��ȡ�Լ죬����ʹ�ù�����У�鷽ʽ
 */
void SetE2prom_Fault ( void )
{
    if ( ( FaultCheckEnable & BIT ( 0 ) ) )
    {
        g_u32FaultNum.bit.uE2promErr = Fault;
    }
}
/**
 * @brief       ���ݿ�汾����
 * @param       ��
 * @retval      ��
 */
void SetDBVersion_Fault ( void )
{
    if ( ( FaultCheckEnable & BIT ( 1 ) ) )
    {
        g_u32FaultNum.bit.uDBVersion = Fault;
    }
}
/**
 * @brief       ���ݿ�汾����
 * @param       ��
 * @retval      ��
 */
void SetBoardUpdata_Fault ( void )
{
    if ( ( FaultCheckEnable & BIT ( 11 ) ) )
    {
        g_u32FaultNum.bit.uBoardUpdata = Fault;
    }
}
/**
 * @brief       ��ѹ�澯
 * @param       ��
 * @retval      ��
 * @note        ��
 */
static void SetLowPressure_Warn ( void )
{
    if ( ( WarnCheckEnable & BIT ( 0 ) ) )
    {
        g_u32WarningNum.bit.uLowPressure = Fault;
    }
}
/**
 * @brief       ��ѹ�澯
 * @param       ��
 * @retval      ��
 * @note        ��
 */
static void SetVolUnder_Warn ( void )
{
    if ( ( WarnCheckEnable & BIT ( 1 ) ) )
    {
        g_u32WarningNum.bit.uVolUnder = Fault;
    }
}
/**
 * @brief       ���δ�궨�澯
 * @param       ��
 * @retval      ��
 * @note        ��
 */
static void SetMotorAxleInvalid_Warn ( void )
{
    if ( ( WarnCheckEnable & BIT ( 2 ) ) )
    {
        g_u32WarningNum.bit.uMotorAxleInvalid = Fault;
    }
}
/**
 * @brief       �������̲�ƥ��澯
 * @param       ��
 * @retval      ��
 * @note        ��
 */
static void SetCoderCali_Warn ( void )
{
    if ( ( WarnCheckEnable & BIT ( 3 ) ) )
    {
        g_u32WarningNum.bit.uCoderCaliWarnning = Fault;
    }
}
/**
 * @brief       �¶ȼ�⺯��
 * @param       ��
 * @retval      ����ֵΪ�Ƿ����
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
 * @brief       ��ȡ�¶�ֵ
 * @param       ��
 * @retval      ����ֵ��ȷ��0.01���϶�
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
 * @brief       ���0���ʼ��
 * @param       ��
 * @retval      0:�궨�ɹ�/�궨��ɣ�1:�궨ʧ��/δ�궨
 */
static int MotorAxleOffsetInit ( void )
{
    uint8_t u8VoltCheckCnt = 0, Delay_1ms = 1, Delay_30ms = 30;
    uint16_t u16VoltValue = 0;
    _iq Angle_0, Angle_90, Angle_180, Angle_270, Angle_360, Angle_0a, Angle_90a, Angle_180a, Angle_270a, AngleDiff;
    _iq _iqIqLoseLim;

    if ( abs ( ParamHandle_Steer.Reg_ReverseFlag ) == 1 ) return 0;  //�궨���

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

        //���������0��ƫ�ƣ���ʼֵ65535����ʼ�������⵽65535����е��0���ʼ��
        GetCoderAngle ( _IQ ( 0.75 ), _iqIqLoseLim ); //���
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
            ParamHandle_Steer.Reg_ReverseFlag = -1;                //0.75Ϊ����
        }
        else
        {
            ParamHandle_Steer.Reg_ReverseFlag = 1;                //0.25Ϊͬ��
        }

        //��ȡƫ��
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
            SetE2prom_Fault ( );  //д��ʧ��
            return 1;
        }
    }
    else
    {
        return 1;
    }
}
/**
 * @brief       ���̶���Ƕ�
 * @param       Angle_Ref��Ŀ���Ƕȣ�_iqIqLoseLim���ж������ȶ���ֵ
 * @retval      �������������ֵ
 */
static _iq GetCoderAngle ( _iq Angle_Ref, _iq _iqIqLoseLim )
{
    _iq Angle_Fbk, Diff;
    uint16_t i, j;

    TimerPWM_DutySet ( _IQ ( 0 ), _IQ ( 0 ), _IQ ( 0 ) );
    HAL_Delay ( 50 );

    for ( i = 1; i < 600; i++ )
    {

        SetVector ( _IQrmpy ( _IQ ( 0.001 ), _IQ ( i ) ), Angle_Ref ); //90��

        HAL_Delay ( 5 );                             //�ȴ������ȶ�

        if ( ( _IQabs ( g_sMotor_Info.iqCurrentA ) > _iqIqLoseLim ) || ( _IQabs ( g_sMotor_Info.iqCurrentB ) > _iqIqLoseLim ) ||
                ( _IQabs ( g_sMotor_Info.iqCurrentC ) > _iqIqLoseLim ) )
        {

            Angle_Fbk            = g_sMotor_Info.iqEAngle;
            HAL_Delay ( 500 );        //�ȴ�ת���ȶ�
            for ( j = 1; j < 100; j++ )
            {
                HAL_Delay ( 1 );
                Diff                = g_sMotor_Info.iqEAngle - Angle_Fbk; //��ֹ0�ȸ����������ʧ��

                if ( abs ( Diff ) < _IQ ( 0.5 ) )
                    Angle_Fbk = _IQrmpy ( Angle_Fbk, _IQ ( 0.95 ) ) + _IQrmpy ( g_sMotor_Info.iqEAngle, _IQ ( 0.05 ) );
            }

            break;
        }
    }

    return Angle_Fbk;
}
/**
 * @brief       ���������̱궨�����ʹ�ù������Ƿ��쳣
 * @param       ��
 * @retval      ��
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
        if ( abs ( CoderHandle._iqLeft ) < iqMiddleValue )  //С�ڡ�1��
        {
            u16Cnt_Left = 0;
            if ( abs ( iqCoderErr ) > iqWarnningValue )  //���ڡ�3��
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
        else if ( abs ( CoderHandle._iqRight ) < iqMiddleValue )  //С�ڡ�1��
        {
            u16Cnt_Right = 0;
            if ( abs ( iqCoderErr ) > iqWarnningValue )  //���ڡ�3��
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
 * @brief       ���������߼�����
 * @param       ��
 * @retval      ��
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


