/**
* 版权所有(C)
*
* ********
*
* @file Int_Ctrl.c
* @brief 
* @details
* @author MuNiu
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
//-------------------- include files ----------------------------------------
#include "MotorInfo.h"
#include "ReangleRS485.h"
#include "CoderTama.h"
#include "ADC.h"
#include "VVVF.h"
#include "DataBaseProcess.h"
#include "Monitor.h"
#include "Foc.h"

//-------------------- local definitions ------------------------------------
#define ENCODER_RESOLUTION     _IQ(1)

//-------------------- private data -----------------------------------------
_iq iqOffset_CurA = 0, iqOffset_CurB = 0, iqOffset_CurC = 0;

//-------------------- private functions ------------------------------------
static void Current_Offset_Clc ( void );

//-------------------- public data ------------------------------------------
S_MOTOR_INFO g_sMotor_Info;
Encoder_Struct Encoder_Handle;

//-------------------- public functions -------------------------------------
/*! \fn		    void MotorInfo_Init( void )
 *  \brief 		电机状态初始化
 *  \param 		none
 *  \return 	none
 */
void MotorInfo_Init ( void )
{
    g_sMotor_Info.u16Check_State = 0xFFFF;
    
    Current_Offset_Clc ( );
}
/*! \fn				void MotorInfo_OutSpdCalc( S_MOTOR_INFO *psInfo,uint16_t CalcFreq )
 *  \brief 		电机转速计算
 *  \param 		psInfo:电机状态，CalcFreq:计算频率
 *  \return 	none
 */
    _iq iqSpdFilterLim = _IQ(0.05);  //3000*_IQ(1)/(CalcFreq * 60):3000空载最大转速，_IQ(1)一圈满量程
    _iq iqNowAddPos;
    _iq iqNewSpdRPM;
    _iq OuTPos, OuTSpd;
void MotorInfo_OutSpdCalc ( S_MOTOR_INFO *psInfo,uint16_t CalcFreq )
{
    static _iq iqOldAddPos = 0;
    static _iq iqOldSpdRPM = 0;
//    static _iq iqSpdFilterLim = _IQ(0.05);  //3000*_IQ(1)/(CalcFreq * 60):3000空载最大转速，_IQ(1)一圈满量程
//    _iq iqNowAddPos;
//    _iq iqNewSpdRPM;
//    _iq OuTPos, OuTSpd;
    
    iqNowAddPos = GetReangleValue ( );
    OuTPos = iqNowAddPos - iqOldAddPos;
    if ( OuTPos < -_IQ(0.5) )
    {
        OuTPos+=_IQ(1);
    }
    if ( OuTPos > _IQ(0.5) )
    {
        OuTPos-=_IQ(1);
    }
    iqNewSpdRPM = OuTPos * CalcFreq * 60;  //IQ(1):1rpm
//    OuTSpd = abs ( iqNewSpdRPM - iqOldSpdRPM );
//    if( abs( OuTPos ) < iqSpdFilterLim )//速度突变滤除
    {
//        if ( _IQabs ( psInfo->iqOutSpd ) > _IQ( 60 ) ) psInfo->iqOutSpd = _IQrmpy ( psInfo->iqOutSpd,_IQ( 0.5 ) )+_IQrmpy ( iqNewSpdRPM, _IQ( 0.5 ) );
//        else {
//            psInfo->iqOutSpd = _IQrmpy ( psInfo->iqOutSpd, _IQ( 0.9 ) ) +_IQrmpy( iqNewSpdRPM, _IQ( 0.1 ) );
//        }
        if ( abs ( iqNewSpdRPM ) > _IQ( 1000 ) ) {
            psInfo->iqSpd_PU = _IQrmpy(psInfo->iqSpd_PU,_IQ(0.9))+_IQrmpy(_IQdiv(iqNewSpdRPM,_IQ(3000)),_IQ(0.1));
        } else {
            psInfo->iqSpd_PU = _IQrmpy(psInfo->iqSpd_PU,_IQ(0.95))+_IQrmpy(_IQdiv(iqNewSpdRPM,_IQ(3000)),_IQ(0.05));
        }
    }
    
    iqOldAddPos = iqNowAddPos;
//    iqOldSpdRPM = iqNewSpdRPM;
}

/*! \fn				void MotorInfo_PosAbsCalc( S_MOTOR_INFO *psInfo,uint16_t CalcFreq )
 *  \brief 		后轮编码器位置计算
 *  \param 		psInfo:电机状态，CalcFreq:计算频率
 *  \return 	none
 */
void MotorInfo_PosAbsCalc ( S_MOTOR_INFO *psInfo, uint16_t CalcFreq )
{
    int32_t s32CoderValue;
    
    CoderTama_SendQuery ( );  //发送读取后轮编码器指令（RS485,115200）
    s32CoderValue = CoderTama_GetCode ( );
    
//    psInfo->iqPosAbs = s32CoderValue;
}
/*! \fn				void MotorInfo_GetCurrent( S_MOTOR_INFO *psInfo,uint16_t CalcFreq )
 *  \brief 		读取并计算三项电流
 *  \param 		psInfo:电机状态，CalcFreq:计算频率
 *  \return 	none
 */
void MotorInfo_GetCurrent( S_MOTOR_INFO *psInfo, uint16_t CalcFreq )
{
    int16_t s16DataCh1, s16DataCh2, s16DataCh3;
    
//    if ( Foc_EN_Flag )
//    {
//        switch ( eCurrentMode )
//        {
//            case CURRENT_MODE_AB:
//            {
//                s16DataCh1 = (int16_t)ADC_GetValue( ADC_CH_I_1 )-2047;
//                s16DataCh2 = (int16_t)ADC_GetValue( ADC_CH_I_2 )-2047;
//                s16DataCh3 = 0 - ( s16DataCh1 + s16DataCh2 );
//            }
//            break;
//            case CURRENT_MODE_BC:
//            {
//                s16DataCh1 = 0 - ( s16DataCh2 + s16DataCh3 );
//                s16DataCh2 = (int16_t)ADC_GetValue( ADC_CH_I_2 )-2047;
//                s16DataCh3 = (int16_t)ADC_GetValue( ADC_CH_I_3 )-2047;
//            }
//            break;
//            case CURRENT_MODE_AC:
//            {
//                s16DataCh1 = (int16_t)ADC_GetValue( ADC_CH_I_1 )-2047;
//                s16DataCh2 = 0 - ( s16DataCh1 + s16DataCh3 );
//                s16DataCh3 = (int16_t)ADC_GetValue( ADC_CH_I_3 )-2047;
//            }
//            break;
//            default:
//                break;
//        }
//    }
//    else
//    {
//        s16DataCh1 = (int16_t)ADC_GetValue( ADC_CH_I_1 )-2047;
//        s16DataCh2 = (int16_t)ADC_GetValue( ADC_CH_I_2 )-2047;
//        s16DataCh3 = (int16_t)ADC_GetValue( ADC_CH_I_3 )-2047;
//    }
    
//    psInfo->iqCurrentA = _IQrmpy( _IQdiv( ( s16DataCh1 ), 2048 ), ParamHandle_Steer.Reg_SampleMaxCur );  //获取A、B相电流，采样量程-27.5A-27.5A，假定无零偏,_IQ(1)代表1A,_IQ( 27.5 )
//    psInfo->iqCurrentB = _IQrmpy( _IQdiv( ( s16DataCh2 ), 2048 ), ParamHandle_Steer.Reg_SampleMaxCur );
//    psInfo->iqCurrentC = _IQrmpy( _IQdiv( ( s16DataCh3 ), 2048 ), ParamHandle_Steer.Reg_SampleMaxCur );
    
    s16DataCh1 = (int16_t)ADC_GetValue( ADC_CH_I_1 );
    s16DataCh2 = (int16_t)ADC_GetValue( ADC_CH_I_2 );
    s16DataCh3 = (int16_t)ADC_GetValue( ADC_CH_I_3 );
    
    psInfo->iqCurrentA = _IQrmpy( _IQdiv( ( s16DataCh1 - iqOffset_CurA ), 2048 ), ParamHandle_Steer.Reg_SampleMaxCur );  //获取A、B相电流，采样量程-27.5A-27.5A，假定无零偏,_IQ(1)代表1A,_IQ( 27.5 )
    psInfo->iqCurrentB = _IQrmpy( _IQdiv( ( s16DataCh2 - iqOffset_CurB ), 2048 ), ParamHandle_Steer.Reg_SampleMaxCur );
    psInfo->iqCurrentC = _IQrmpy( _IQdiv( ( s16DataCh3 - iqOffset_CurC ), 2048 ), ParamHandle_Steer.Reg_SampleMaxCur );
    
    psInfo->iqPressure = (int16_t)ADC_GetValue( ADC_CH_Pressure );
}
/*! \fn				void Current_Offset_Clc ( void )
 *  \brief 		计算三相电流OFFSET
 *  \param 		none
 *  \return 	none
 */
static void Current_Offset_Clc ( void )
{
    static _iq iqArray[3][10], iqCurrent_Offset[3];
    static _iq  iqSum[3], iqDataMax[3] = { 0, 0, 0 }, iqDataMin[3] = { 4095, 4095, 4095 };

    for ( int i = 0; i < 10; i++ )
    {
        HAL_Delay ( 1 );
        iqArray[0][i] = ADC_GetValue( ADC_CH_I_1 );
        iqArray[1][i] = ADC_GetValue( ADC_CH_I_2 );
        iqArray[2][i] = ADC_GetValue( ADC_CH_I_3 );
    }
    
    for ( int j = 0; j < 10; j++ )
    {
        for ( int k = 0; k < 3; k++ )
        {
            if ( iqArray[k][j] > iqDataMax[k] )
            {
                iqDataMax[k] = iqArray[k][j];
            }
            if ( iqArray[k][j] < iqDataMin[k] )
            {
                iqDataMin[k] = iqArray[k][j];
            }
            iqSum[k] += iqArray[k][j];
        }
    }
    
    iqOffset_CurA = _IQdiv8 ( iqSum[0] - iqDataMax[0] - iqDataMin[0] ); 
    iqOffset_CurB = _IQdiv8 ( iqSum[1] - iqDataMax[1] - iqDataMin[1] );
    iqOffset_CurC = _IQdiv8 ( iqSum[2] - iqDataMax[2] - iqDataMin[2] );
}
/*! \fn				void MotorInfo_EAngleCalc( S_MOTOR_INFO *psInfo,uint16_t CalcFreq )
 *  \brief 		读取电角度
 *  \param 		psInfo:电机状态，CalcFreq:计算频率
 *  \return 	none
 */
void MotorInfo_EAngleCalc ( S_MOTOR_INFO *psInfo, uint16_t CalcFreq )
{
    static _iq iqVaule;
    
    iqVaule = _IQ(1) / ParamHandle_Steer.Reg_PolePairs;
    psInfo->iqPosAdd = GetReangleValue();
    psInfo->iqEAngle = _IQdiv( GetReangleValue() % iqVaule, iqVaule );         //_IQ(1)代表360个电角度,_IQ(1) / Motor_PolePairs;
//    psInfo->iqEAngle = ElcAngCal.ElecAngle - ElcAngCal.Motor_Angle_Offset;          //电角度需要考虑偏置和电机码盘方向
//    psInfo->iqEAngle = ElcAngCal.ElecAngle;
}
void MotorInfo_Pressure ( S_MOTOR_INFO *psInfo )
{
    static uint16_t u16Cnt_Pressure = 0;
    
    if ( psInfo->iqPressure > 818 ) //6A
    {
        if (u16Cnt_Pressure<300) u16Cnt_Pressure++;
        else
        {
             psInfo->u16Pressure_State = 1;
        }
    }
    else
    {
        if ( u16Cnt_Pressure > 0 ) u16Cnt_Pressure--;
        else
        {
            psInfo->u16Pressure_State = 0;
        }
    }
}

void Encoder_PosClc( Encoder_Struct *encoder, uint8_t u8Enable_Clc ) {
    if ( u8Enable_Clc ) {
    encoder->last_position = encoder->current_position;
    encoder->current_position = GetReangleValue ( );
    
    // 计算位置增量，处理溢出情况
    int16_t delta = (int16_t)(  encoder->current_position - encoder->last_position );
    
    // 更新总脉冲数
    if ( sTeles_Ctrl.PosMinCali == 1 ) {
        sTeles_Ctrl.PosMinCali = 0;
        encoder->total_ticks = 0;
    } else {
        encoder->total_ticks += delta;
    }
    
    // 更新圈数
    encoder->revolution_count = encoder->total_ticks * ParamHandle_Steer.Reg_PolePairs / ENCODER_RESOLUTION;
    
    // 计算线性距离 (米)
//    encoder->linear_distance = _IQsat ( _IQmpy ( encoder->total_ticks, 2000 ), 65535, 0 );
        
    g_sMotor_Info.iqPosAbs = encoder->revolution_count;
    } else {
       encoder->current_position = GetReangleValue ( ); 
    }
}

//-----------------------End of file------------------------------------------