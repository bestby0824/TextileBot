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

//-------------------- include files ----------------------------------------
#include "AdcProcess.h"
uint16_t DA_i;
int16_t DA_num[ADC_RQ_CH_NUM];
ADC_Value_Struct ADC_Value_Num;

//-------------------- local definitions ------------------------------------
static int16_t Value_Vs_Temp[][2] =
{   -4000,  156,
    -3500,  211,
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
//-------------------- private data -----------------------------------------

//-------------------- private functions declare ----------------------------

//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------

static int16_t CalibrateADCBias(int16_t samples)
{
    int16_t ret;
    ret = samples - 3227;
    return ret;
}
/*
Get_Temperature
Fn  :获取温度值
ret :返回值精确到0.01摄氏度

*/
int16_t s16Temperature;
uint16_t TSpm;
int16_t Get_Temperature ( void )
{
    int i;

    TSpm = DA_num [ ADC_CH_TEMP_IN ];// 
//    TSpm += + CalibrateADCBias(ADC_Value_Num.ADC_2_5V_REF);/*textile*/
    if ( TSpm < Value_Vs_Temp[0][1] )
    {
        s16Temperature = -4000;
    }
    else if ( TSpm > Value_Vs_Temp[33][1] )
    {
        s16Temperature = 12500;
    }
    else
    {
        for ( i = 1; i < 34; i++ )
        {
            if ( TSpm < Value_Vs_Temp[i][1] )
            {
                s16Temperature = Value_Vs_Temp[i][0] - _IQrmpy ( 500, _IQdiv ( ( Value_Vs_Temp[i][1] - TSpm ), ( Value_Vs_Temp[i][1] - Value_Vs_Temp[i - 1][1] ) ) );
                break;
            }
        }
    }
    return s16Temperature;
}
/*
Get_Version_AD
Fn  :获取硬件小版本号
ret :无

*/
uint16_t Get_Version_AD ( void )
{
    uint16_t ret;
    uint16_t u16ADvalue;

    //  R1:1.0k  R2:可调（0K - ret=0，其他待定）
    u16ADvalue = DA_num [ADC_CH_Version]; //+ CalibrateADCBias(ADC_Value_Num.ADC_2_5V_REF);/*textile*/
    for ( int i = 10; i > 0; i-- )
    {
        if ( u16ADvalue > ( i - 1 ) * 400 ) ret = ( i - 1 );
    }

    return ret;
}
/*
Get_Motor_24V
Fn  :获取动力24V电压值
ret :返回值精确到0.01V
*/
uint16_t Get_Motor_24V ( void )
{
    uint16_t ret;
    //  R1:30.1k  R2:2.2k
    //  ret (0.01V) = adc/4096*3.3/2.2*32.3 *100 (0.01V) = adc* 1.1829
//    ret = _IQrmpy ( ADC_GetValue ( ADC_CH_BLDC_24V ), _IQ ( 1.1829 ) );// 1.06917
    return ret;
}

/*
Get_E_Stop_80V
Fn  :获取叉车80V主电池组电压值
ret :返回值精确到0.01V

*/
uint16_t Get_E_Stop_80V ( void )
{
    uint16_t ret;

    //  R1:30k  R2:1k
    //  ret (0.01V) = adc/4096*3.3/1*31 *100 (0.01V) = adc*2.4975
//    ret = _IQrmpy ( ADC_GetValue ( ADC_CH_E_Stop_80V ), _IQ ( 2.4975 ) );// 2.52655
    return ret;
}

/*
Get_BAT_24V
Fn  :获取备用24V电池组电压值
ret :返回值精确到0.01V
*/
uint16_t Get_BAT_24V ( void )
{
    uint16_t ret;

    //  R1:10k  R2:1k
    //  ret (0.01V) = adc/4096*3.3/1*11 *100 (0.01V) = adc*0.8862
//    ret = _IQrmpy ( ADC_GetValue ( ADC_CH_BAT_24V ), _IQ ( 0.8862 ) );

    return ret;
}

/*
ADC_CH_FORK_24V
Fn  :获取叉车24V电压值
ret :返回值精确到0.01V
*/
uint16_t Get_FORK_24V ( void )
{
    uint16_t ret;
    //  R1:30k  R2:2.2k
    //  ret (0.01V) = adc/4096*3.3/2.2*32.2 *100 (0.01V) = adc*1.1792
//    ret = _IQrmpy ( ADC_GetValue ( ADC_CH_FORK_24V ), _IQ ( 1.1792 ) );
    return ret;
}

/*
Get_E_Stop_1A
Fn  :获取急停电流值（未触发情况下1A持续电流）
ret :返回值带符号，精确到1mA

*/
int16_t Get_E_Stop_1A ( void )
{
    int16_t ret;
    //  2.5V为中心，400mV/A => 2500mA/V
    //  ret (0.01A) = (adc-3103)/4096*3.3*2500 (0.01V) = (adc-3103)*2.0142
    //  ret = _IQrmpy ( ( ( int16_t ) ADC_GetValue ( ADC_CH_E_Stop_1A ) - 3103 ), _IQ ( 2.0142 ) );
    return ret;
}
/*textile*/
void ADC_Value()
{

    for ( DA_i = 0; DA_i < ADC_RQ_CH_NUM; DA_i++ )
    {
        DA_num[DA_i] =  (int16_t)ADC_GetValue ( DA_i );
    }
    memcpy ( &ADC_Value_Num, DA_num, sizeof ( ADC_Value_Num ) );    
//    ADC_Value_Num.ADC_Ctrl_24V_I = ( 3220 - ADC_Value_Num.ADC_Ctrl_24V_I ) * 0.8;
//    ADC_Value_Num.ADC_BAT_I = ( 3220 - ADC_Value_Num.ADC_BAT_I ) * 0.8;
//    ADC_Value_Num.ADC_BLDC_I = ( 3220 - ADC_Value_Num.ADC_BLDC_I ) * 0.8 ;
//    ADC_Value_Num.ADC_FORK_I = ( 3220 - ADC_Value_Num.ADC_FORK_I ) * 0.8 ;
//    ADC_Value_Num.ADC_VALVE_I = ( 3220 - ADC_Value_Num.ADC_VALVE_I ) * 0.8;
//    ADC_Value_Num.ADC_PC_I = ( 3220 - ADC_Value_Num.ADC_PC_I ) * 0.8;
}

//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


