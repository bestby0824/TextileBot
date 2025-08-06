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
#include "EncoderProcess.h"
#include "TimerBase.h"

//-------------------- local definitions ------------------------------------
#define T_Num           10
//-------------------- private data -----------------------------------------
_iq _iqSpd_M1 = 0;
_iq _iqSpd_M2 = 0;
//-------------------- private functions declare ----------------------------

//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
int16_t g_nLWheelPos = 0;//左码盘位置（非转向轮 0 - 10000
int16_t g_nRWheelPos = 0;//右码盘位置（非转向轮 0 - 10000
int16_t g_Car_Pos = 0;    //整车前轮位置(0-360,0-4096)
_iq EncProc_GerSpdFromEncoder()
{
    if ( _iqSpd_M1 > _IQ ( 0.2 ) )
    {
        return _iqSpd_M1;
    }
    else
    {
        return _iqSpd_M1;
    }
}

_iq tmp, tmp2;
_iq g_Spd1 = 0;
_iq g_Spd2 = 0;

_iq g_Spd1_T = 0;
_iq g_Spd2_T = 0;
int16_t Timer1Counter = 0;
int16_t Timer3Counter = 0;
/**
  * @brief  左右轮速度计算
  * @param  none
  * @retval None
  */
void SpdClc ( uint16_t Freq )
{
    static int16_t Timer1Counter_Old = 0;
    static int16_t Timer3Counter_Old = 0;

    int16_t Diff1 = 0, Diff2 = 0;
//    Timer1Counter = ( int16_t ) __HAL_TIM_GET_COUNTER ( &ENC_TimHandle1 );
    Timer3Counter = ( int16_t ) __HAL_TIM_GET_COUNTER ( &ENC_TimHandle2 );
//    g_nLWheelPos = ENCLines - Timer1Counter;
    g_nRWheelPos = ENCLines - Timer3Counter;
    g_Car_Pos = g_nRWheelPos;//( g_nLWheelPos + g_nRWheelPos ) * 0.5;
//    Diff1 = ( Timer1Counter - Timer1Counter_Old );
    Diff2 = ( Timer3Counter - Timer3Counter_Old );
//    if ( abs ( Diff1 ) > ( ENCLines >> 1 ) )
//        Diff1 += Diff1 > 0 ? ( -1 ) * ENCLines : ENCLines;//20000

    if ( abs ( Diff2 ) > ( ENCLines >> 1 ) )
        Diff2 += Diff2 > 0 ? ( -1 ) * ENCLines : ENCLines;

//    tmp = _IQdiv ( Diff1 * Freq * 60, ENCLines ); //_IQ(1)代表1rpm
//    g_Spd1 = -_IQdiv ( tmp, _iqMaxSpd_rpm );   //_IQ(1)代表 _iqMaxSpd_rpm

    tmp = _IQdiv ( Diff2 * Freq * 60, ENCLines ); //_IQ(1)代表1rpm
    tmp2 = tmp;
    g_Spd2 = -_IQdiv ( tmp, _iqMaxSpd_rpm );   //_IQ(1)代表 _iqMaxSpd_rpm  1rpm

    //_iqSpd_M1 = _IQmpy ( _iqSpd_M1, _IQ ( 0.9 ) ) + _IQmpy ( _IQdiv2 ( g_Spd1 + g_Spd2 ), _IQ ( 0.1 ) ); 
    _iqSpd_M1 = _IQmpy ( _iqSpd_M1, _IQ ( 0.9 ) ) + _IQmpy ( g_Spd2, _IQ ( 0.1 ) ); 
//    Timer1Counter_Old = Timer1Counter;
    Timer3Counter_Old = Timer3Counter;
}
/**
  * @brief  速度计算（无用）
  * @param  none
  * @retval None
  */
void SpdClc_T ( uint16_t Freq )
{
    static int16_t Timer1Counter_Old = 0;
    static int16_t Timer3Counter_Old = 0;
    static int32_t T1_50K_Old;
    static int32_t T2_50K_Old;
    int32_t T1_50K;
    int32_t T2_50K;
    static int32_t T1_50K_Diff, T2_50K_Diff;
    int16_t Timer1Counter = 0;
    int16_t Timer3Counter = 0;
    static int16_t Diff1 = 0, Diff2 = 0;
    Timer1Counter = ( int16_t ) __HAL_TIM_GET_COUNTER ( &ENC_TimHandle1 );
    Timer3Counter = ( int16_t ) __HAL_TIM_GET_COUNTER ( &ENC_TimHandle2 );
    g_nLWheelPos = Timer1Counter;
    g_nRWheelPos = Timer3Counter;
    T1_50K = TimerBase_GetTicks_50KHz();
    T2_50K = T1_50K;
    if ( abs ( Timer1Counter_Old - Timer1Counter ) > T_Num )
    {
        if ( abs ( Timer1Counter_Old - Timer1Counter ) > 5000 )
        {
            return;
        }
        Diff1 = ( Timer1Counter - Timer1Counter_Old );
        if ( abs ( Diff1 ) > ( ENCLines >> 1 ) )
            Diff1 += Diff1 > 0 ? ( -1 ) * ENCLines : ENCLines;


        //tmp = _IQdiv ( Diff1 * Freq/(T1_50K - T1_50K_Old) * 60, ENCLines ); //_IQ(1)代表1rpm
        tmp = _IQdiv ( Diff1 * Freq * 60, ENCLines * ( T1_50K - T1_50K_Old ) ); //_IQ(1)代表1rpm
        g_Spd1_T = _IQdiv ( tmp, _iqMaxSpd_rpm );   //_IQ(1)代表 _iqMaxSpd_rpm
        Timer1Counter_Old = Timer1Counter;
        T1_50K_Diff = T1_50K - T1_50K_Old;
        T1_50K_Old = T1_50K;
    } else if ( abs ( T1_50K - T1_50K_Old ) > abs ( T1_50K_Diff ) )
    {
        tmp = _IQdiv ( Diff1 * Freq * 60, ENCLines * ( T1_50K - T1_50K_Old ) ); //_IQ(1)代表1rpm
        g_Spd1_T = _IQdiv ( tmp, _iqMaxSpd_rpm );   //_IQ(1)代表 _iqMaxSpd_rpm
    }

    if ( abs ( Timer3Counter_Old - Timer3Counter ) > T_Num )
    {
        if ( abs ( Timer3Counter_Old - Timer3Counter ) > 10000 )
        {
            return;
        }

        Diff2 = ( Timer3Counter - Timer3Counter_Old );

        if ( abs ( Diff2 ) > ( ENCLines >> 1 ) )
            Diff2 += Diff2 > 0 ? ( -1 ) * ENCLines : ENCLines;
        //tmp = _IQdiv ( Diff1 * Freq/(T1_50K - T1_50K_Old) * 60, ENCLines ); //_IQ(1)代表1rpm
        tmp = _IQdiv ( Diff2 * Freq * 60, ENCLines * ( T2_50K - T2_50K_Old ) ); //_IQ(1)代表1rpm
        g_Spd2_T = _IQdiv ( tmp, _iqMaxSpd_rpm );   //_IQ(1)代表 _iqMaxSpd_rpm

        Timer3Counter_Old = Timer3Counter;
        T2_50K_Diff = T2_50K - T2_50K_Old;
        T2_50K_Old = T2_50K;
    } else if ( abs ( T2_50K - T2_50K_Old ) > abs ( T2_50K_Diff ) )
    {
        tmp = _IQdiv ( Diff2 * Freq * 60, ENCLines * ( T2_50K - T2_50K_Old ) ); //_IQ(1)代表1rpm
        g_Spd2_T = _IQdiv ( tmp, _iqMaxSpd_rpm );   //_IQ(1)代表 _iqMaxSpd_rpm
    }
    _iqSpd_M2 = _IQmpy ( _iqSpd_M2, _IQ ( 0.9 ) ) + _IQmpy ( _IQdiv2 ( g_Spd1_T + g_Spd2_T ), _IQ ( 0.1 ) ); ;

}
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


