/**
* 版权所有(C)
*
* ********
*
* @file VVVF.c
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
#include "Foc.h"
#include "AngleMath.h"
#include "ADC.h"
#include "VVVF.h"
#include "Uart.h"
#include "ReangleRS485.h"
#include "TimerPWM.h"
#include "MotorInfo.h"
#include "MotorCmd_Ctrl.h"
#include "DataBaseProcess.h"
#include "Monitor.h"

//-------------------- local definitions ------------------------------------
#define SpeedLoopPrescaler   10                              // Speed loop prescaler

//-------------------- private data -----------------------------------------
PI_CONTROLLER pi_iq  = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER pi_id  = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER pi_spd = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER pi_pos = PI_CONTROLLER_DEFAULTS;
IPARK Ipark;
PARK Park;
CLARK Clark;

uint32_t FocCnt = 0, u32Cnt = 0;
uint8_t TXBuf[8];
uint8_t u8StopFalg = 0;
uint16_t u16Cnt_Posctrl = 0, u16Cnt_Posctrltime = 200;
//-------------------- private functions ------------------------------------
static void MaxSpdClc ( _iq EncoderPos );

//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
void FOC_PID_Init ( void )
{
    pi_pos.Kp = ParamHandle_Steer.Reg_PI_POS_KP;
    pi_pos.Ki = ParamHandle_Steer.Reg_PI_POS_KI;
    pi_spd.Kp = ParamHandle_Steer.Reg_PI_SPD_KP*8;  //速度环20ms
    pi_spd.Ki = ParamHandle_Steer.Reg_PI_SPD_KI;
    pi_id.Kp = ParamHandle_Steer.Reg_PI_ID_KP;
    pi_id.Ki = ParamHandle_Steer.Reg_PI_ID_KI;
    pi_iq.Kp = ParamHandle_Steer.Reg_PI_IQ_KP;  //电流环1ms
    pi_iq.Ki = ParamHandle_Steer.Reg_PI_IQ_KI;

    pi_pos.Umax = _IQdiv ( ParamHandle_Steer.Reg_SpeedMax, ParamHandle_Steer.Reg_NoLoadSpeed );
    pi_pos.Umin = -pi_pos.Umax;

    pi_spd.Umax = _IQdiv ( ParamHandle_Steer.Reg_CurrentMax, 100 );
    pi_spd.Umin = -pi_spd.Umax;

    pi_iq.Umax = ParamHandle_Steer.Reg_IdIqMax - _IQ ( 0.01 );
//    pi_iq.Umax = ParamHandle_Steer.Reg_IdIqMax;
    pi_iq.Umin = -pi_iq.Umax;
    pi_id.Umax = pi_iq.Umax;
    pi_id.Umin = pi_iq.Umin;
}
/*------------------------------------------------------------------------------
	CLARKE Transformation Macro Definition
------------------------------------------------------------------------------*/
void ClarkTra ( CLARK *clark ) //CLARK变换
{
    clark->Alpha = clark->As;
    clark->Beta = _IQrmpy ( ( clark->As + _IQmpy2 ( clark->Bs ) ), _IQ ( 0.57735026918963 ) );
}

void ParkTra ( PARK *park ) //park变换
{
    park->Ds = _IQrmpy ( park->Alpha, park->Cosine ) + _IQrmpy ( park->Beta, park->Sine );
    park->Qs = _IQrmpy ( park->Beta, park->Cosine ) - _IQrmpy ( park->Alpha, park->Sine );
}
/**********************************************************************************************************
park逆变换，输入Uq、Ud得到Ualpha、Ubeta
Uα = Ud · cosθ - Uq · sinθ
Uβ = Ud · sinθ + Uq · cosθ
**********************************************************************************************************/
void InvParkTra ( IPARK *ipark )      // 反PARK变换
{
    ipark->Alpha = _IQrmpy ( ipark->Ds, ipark->Cosine ) - _IQrmpy ( ipark->Qs, ipark->Sine );
    ipark->Beta  = _IQrmpy ( ipark->Ds, ipark->Sine ) + _IQrmpy ( ipark->Qs, ipark->Cosine );
}
void Current_Loop ( PI_CONTROLLER * v )
{
    v->up = v->Ref - v->Fbk;

    v->ui = ( _IQrmpy ( v->Ki, v->up ) + v->i1 );
    v->ui = ( ( v->Out == v->v1 ) || ( _IQabs ( v->ui ) < _IQabs ( v->i1 ) ) ) ? v->ui : _IQrmpy ( v->i1, _IQ ( 0.99 ) );
    v->i1 = v->ui;  /*限幅生效则积分停止增长且产生0.01的阻尼*/

    v->v1 = _IQrmpy ( v->Kp, ( v->up + v->ui ) );
    v->Out = _IQsat ( v->v1, v->Umax, v->Umin );
}
void Speed_Loop ( PI_CONTROLLER * v )
{
    v->up = v->Ref - v->Fbk;

    v->ui = ( _IQrmpy ( v->Ki, v->up ) + v->i1 );
    v->ui = ( ( v->Out == v->v1 ) || ( _IQabs ( v->ui ) < _IQabs ( v->i1 ) ) ) ? v->ui : _IQrmpy ( v->i1, _IQ ( 0.99 ) );
    v->i1 = v->ui;  /*限幅生效则积分停止增长且产生0.01的阻尼*/

    v->d2 = _IQrmpy ( v->Kd, _IQrmpy ( v->c1, ( _IQrmpy ( v->Ref, v->Km ) - v->Fbk ) ) ) - v->d2;
    v->ud = v->d2 + v->d1;
    v->d1 = _IQrmpy ( v->ud, v->c2 );

    v->v1 = _IQrmpy ( v->Kp, ( v->up + v->ui + v->ud ) );
    v->Out = _IQsat ( v->v1, v->Umax, v->Umin );
}
void Pos_Loop ( PI_CONTROLLER * v )
{
    v->up = v->Ref - v->Fbk;

    v->ui = ( _IQrmpy ( v->Ki, v->up ) + v->i1 );
    v->ui = ( ( v->Out == v->v1 ) || ( _IQabs ( v->ui ) < _IQabs ( v->i1 ) ) ) ? v->ui : _IQrmpy ( v->i1, _IQ ( 0.99 ) );
    v->i1 = v->ui;  /*限幅生效则积分停止增长且产生0.01的阻尼*/

    v->v1 = _IQrmpy ( v->Kp, ( v->up + v->ui ) );
    v->Out = _IQsat ( v->v1, v->Umax, v->Umin );
}
_iq _iqtestl,_iqtest2;
void FOC_Core ( S_MOTOR_INFO *psInfo, S_MOTOR_CMD *psCmd )
{
    static uint16_t u16SpeedLoopCount = 0;           // Speed loop counter

    if ( ( psCmd->eRunState == CmdRuning ) && ( psInfo->u16MotorVol_State ) )
//        if (_iqtest2)
    {
        TimerPWM_ModeSet ( PWM_MODE_RUN );
        //========================计算等效相电流=============================================
//        psInfo->iqEffectCurrent = _IQdiv ( ( abs ( psInfo->iqCurrentA ) + abs ( psInfo->iqCurrentB ) + abs ( psInfo->iqCurrentC ) ), _IQ ( 1.732 ) );
        //========================Clark变换=============================================
        Clark.As = _IQrmpy ( -psInfo->iqCurrentA, _IQ ( 0.5 ) ) + _IQrmpy ( Clark.As, _IQ ( 0.5 ) ); // Phase A curr
        Clark.Bs = _IQrmpy ( -psInfo->iqCurrentB, _IQ ( 0.5 ) ) + _IQrmpy ( Clark.Bs, _IQ ( 0.5 ) ); // Phase B curr
        ClarkTra ( &Clark );
        //========================Park变换=============================================
        Park.Angle = _IQrmpy ( ( psInfo->iqEAngle - ParamHandle_Steer.Reg_MotorAxleOffset ), _IQ ( ParamHandle_Steer.Reg_ReverseFlag ) );
        Angle_IQ0_IQ1 ( &Park.Angle );
        Park.Alpha = Clark.Alpha;
        Park.Beta  = Clark.Beta;
        Park.Sine   = _IQsinPU ( Park.Angle );
        Park.Cosine = _IQcosPU ( Park.Angle );
        ParkTra ( &Park );
        //========================位置环+速度环=============================================
        if ( ++u16SpeedLoopCount >= SpeedLoopPrescaler )  //速度环和位置环2KHZ频率执行
        {
            u16SpeedLoopCount = 0;

            pi_pos.Ref = _IQsat ( psCmd->iqTargetPos, ParamHandle_Steer.Reg_PosMax, ParamHandle_Steer.Reg_PosMin );
            pi_pos.Fbk = psInfo->iqPosAbs;
            MaxSpdClc ( pi_pos.Fbk );
            Pos_Loop ( &pi_pos );
//            if ( ( ( u16RunningDir == CtrlDir_Foward ) && ( pi_pos.up < 0 ) ) || \
//                ( ( u16RunningDir == CtrlDir_Reverse ) && ( pi_pos.up > -ParamHandle_Steer.Reg_PosDeadZone ) ) )
            if ( abs ( pi_pos.up ) < ParamHandle_Steer.Reg_PosDeadZone )
            {
                if ( u16Cnt_Posctrl < u16Cnt_Posctrltime ) u16Cnt_Posctrl++;
                else
                {
                    psCmd->eRunState = EndOfCtrl;
                    pi_pos.Out = 0;
                    pi_spd.Ref = 0;
                }
            }
            else
            {
                u16Cnt_Posctrl = 0;
                pi_spd.Ref = -pi_pos.Out;
            }
//            pi_spd.Ref = _iqtestl;
            pi_spd.Fbk = psInfo->iqSpd_PU;
            Speed_Loop ( &pi_spd );
        }
        //========================电流环(id+iq)=============================================
        pi_id.Ref = 0;
//        pi_id.Fbk = _IQrmpy ( Park.Ds, _IQ ( 0.95 ) ) + _IQrmpy ( pi_id.Fbk, _IQ ( 0.05 ) );
        pi_id.Fbk = Park.Ds;
        pi_iq.Ref = _IQrmpy ( pi_spd.Out, _IQ ( ParamHandle_Steer.Reg_ReverseFlag ) );
//        pi_iq.Fbk = _IQrmpy ( Park.Qs, _IQ ( 0.95 ) ) + _IQrmpy ( pi_iq.Fbk, _IQ ( 0.05 ) );
        pi_iq.Fbk = Park.Qs;
        Current_Loop ( &pi_id );
        Current_Loop ( &pi_iq );
        //========================反Park变换=============================================
        Ipark.Ds = pi_id.Out;
        Ipark.Qs = pi_iq.Out;
        Ipark.Sine = Park.Sine;
        Ipark.Cosine = Park.Cosine;
        InvParkTra ( &Ipark );
        //========================调用SVPWM()=============================================
        SVPWMStruct.Ualpha = Ipark.Alpha;
        SVPWMStruct.Ubeta = Ipark.Beta;
        SVPWM ( );
    }
    else
    {
        pi_pos.ui = 0;
        pi_pos.i1 = 0;
        pi_spd.ui = 0;
        pi_spd.i1 = 0;
        pi_spd.Ref = 0;
        pi_spd.Fbk = 0;
        pi_id.ui = 0;
        pi_id.i1 = 0;
        pi_iq.ui = 0;
        pi_iq.i1 = 0;
        pi_id.Out = 0;
        pi_iq.Out = 0;
        pi_spd.Out = 0;
        pi_pos.Out = 0;
        pi_iq.Fbk = 0;
        psInfo->iqEffectCurrent = 0;

        if ( g_u32FaultNum.bit.uCmdDogLose )  //通讯异常
        {
            TimerPWM_ModeSet ( PWM_MODE_OFF );
        }
        else
        {
            TimerPWM_ModeSet ( PWM_MODE_STOP );
        }
    }
    psInfo->iqEffectCurrent = _IQdiv ( ( abs ( psInfo->iqCurrentA ) + abs ( psInfo->iqCurrentB ) + abs ( psInfo->iqCurrentC ) ), _IQ ( 1.732 ) );

    if ( FocCnt++ % 20 == 0 )
    {
//        TXBuf[0] = 1;
//        TXBuf[1] = 2;
//        TXBuf[2] = 3;
//        TXBuf[3] = 4;
//        TXBuf[4] = 5;
//        TXBuf[5] = 6;
//        TXBuf[6] = 7;
//        TXBuf[7] = 8;

//        TXBuf[0] = _IQtoIQ12 ( pi_iq.Ref )>> 8;
//        TXBuf[1] = _IQtoIQ12 ( pi_iq.Ref );
//        TXBuf[2] = _IQtoIQ12 ( pi_iq.Fbk ) >> 8;
//        TXBuf[3] = _IQtoIQ12 ( pi_iq.Fbk );
//        TXBuf[4] = _IQtoIQ12 ( pi_id.Ref ) >> 8;
//        TXBuf[5] = _IQtoIQ12 ( pi_id.Ref );
//        TXBuf[6] = _IQtoIQ12 ( pi_id.Fbk ) >> 8;
//        TXBuf[7] = _IQtoIQ12 ( pi_id.Fbk );

        TXBuf[0] = _IQtoIQ12 ( pi_spd.Ref ) >> 8;
        TXBuf[1] = _IQtoIQ12 ( pi_spd.Ref );
        TXBuf[2] = _IQtoIQ12 ( pi_spd.Fbk ) >> 8;
        TXBuf[3] = _IQtoIQ12 ( pi_spd.Fbk );
        TXBuf[4] = _IQtoIQ12 ( pi_pos.Ref )>> 8;
        TXBuf[5] = _IQtoIQ12 ( pi_pos.Ref );
        TXBuf[6] = _IQtoIQ12 ( pi_pos.Fbk ) >> 8;
        TXBuf[7] = _IQtoIQ12 ( pi_pos.Fbk );

//        TXBuf[0] = _IQtoIQ12 ( psInfo->iqPosAbs )>> 8;
//        TXBuf[1] = _IQtoIQ12 ( psInfo->iqPosAbs );
//        TXBuf[2] = _IQtoIQ12 ( psInfo->iqEAngle ) >> 8;
//        TXBuf[3] = _IQtoIQ12 ( psInfo->iqEAngle );
//        TXBuf[4] = psInfo->iqPressure >> 8;
//        TXBuf[5] = psInfo->iqPressure;
//        TXBuf[6] = _IQtoIQ10 ((abs (pi_iq.Fbk))) >>8;
//        TXBuf[7] = _IQtoIQ10(abs(pi_iq.Fbk));

//        TXBuf[0] = _IQtoIQ10 (psInfo->iqCurrentA) >>8;
//        TXBuf[1] = _IQtoIQ10 (psInfo->iqCurrentA) ;
//        TXBuf[2] = _IQtoIQ10 (psInfo->iqCurrentB) >>8;
//        TXBuf[3] = _IQtoIQ10 (psInfo->iqCurrentB) ;
//        TXBuf[4] = _IQtoIQ10 (psInfo->iqCurrentC) >>8;
//        TXBuf[5] = _IQtoIQ10 (psInfo->iqCurrentC) ;
//        TXBuf[6] = _IQtoIQ10 ((abs (pi_iq.Fbk))) >>8;;
//        TXBuf[7] = _IQtoIQ10 ((abs (pi_iq.Fbk)));
//        TXBuf[0] = pi_pos.Ref >> 8;
//        TXBuf[1] = pi_pos.Ref;
//        TXBuf[2] = pi_pos.Fbk >> 8;
//        TXBuf[3] = pi_pos.Fbk;
//        TXBuf[4] = Encoder_Handle.linear_distance >> 8;
//        TXBuf[5] = Encoder_Handle.linear_distance;
//        TXBuf[6] = pi_pos.Fbk >> 8;
//        TXBuf[7] = pi_pos.Fbk;

        Uart_Send ( ( uint8_t * ) TXBuf, 8 );
    }

}
/**
 * @brief       调用反Park和SVPWM函数，使转子转到固定电角度
 * @param       Value：给定电流，Angle：目标电角度
 * @retval      无
 */
void SetVector ( _iq Value, _iq Angle )
{
    IPARK Ipark2;

    Ipark2.Ds = Value;
    Ipark2.Qs = Value;

    Ipark2.Sine = _IQsinPU ( Angle );
    Ipark2.Cosine = _IQcosPU ( Angle );

    InvParkTra ( &Ipark2 );

    /*输出SVPWM，内含反Clark变换*/
    SVPWMStruct.Ualpha = Ipark2.Alpha;
    SVPWMStruct.Ubeta = Ipark2.Beta;
    SVPWM();
}
/**
 * @brief       码盘最大速度给定
 * @param       EncoderPos:码盘位置
 * @retval      无
 *速度：0.5-0.1，位置0.3-0.5
 */
static void MaxSpdClc ( _iq EncoderPos )
{
    static _iq iqSpdBase = _IQ ( 0.1 ), iqPosBase = _IQ ( 0.3 );
    
    if ( EncoderPos < iqPosBase ) {
        pi_pos.Kp = ParamHandle_Steer.Reg_PI_POS_KP*2;
        pi_pos.Umax = _IQsat ( _IQ ( 0.4 ) + _IQmpy ( ( iqPosBase - EncoderPos ), _IQ ( 4 ) ), _IQ ( 0.6 ), _IQ ( 0.4 ) );
        pi_pos.Umin = -pi_pos.Umax;
    } else {
//        pi_pos.Kp = _IQsat ( ( ParamHandle_Steer.Reg_PI_POS_KP*2 - _IQmpy ( ( EncoderPos - iqPosBase ), _IQ(512) ) ), \
//            ParamHandle_Steer.Reg_PI_POS_KP*2, ParamHandle_Steer.Reg_PI_POS_KP/4 );
        pi_pos.Kp = _IQsat ( ( pi_pos.Kp - _IQ ( 0.005 ) ), ParamHandle_Steer.Reg_PI_POS_KP, ParamHandle_Steer.Reg_PI_POS_KP/4 );
        pi_pos.Umax = _IQsat ( ( _IQ ( 0.4 ) - _IQmpy ( ( EncoderPos - iqPosBase ), _IQ ( 4 ) ) ), _IQ ( 0.4 ), _IQ ( 0.07 ) );
        pi_pos.Umin = -pi_pos.Umax;
    }
}
