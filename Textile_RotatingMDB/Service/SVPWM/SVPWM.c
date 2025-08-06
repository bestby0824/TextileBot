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
#include "SVPWM.h"
#include "TimerPWM.h"
#include "Uart.h"
#include "VVVF.h"
#include "AngleMath.h"
#include "Int_Ctrl.h"
#include "Foc.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------

//-------------------- private functions ------------------------------------

//-------------------- public data ------------------------------------------
SVPWMDef SVPWMStruct;
_iq Ta, Tb, Tc;
uint8_t ThetaFlag = 0;
uint32_t ThetaElc;
uint16_t Theta,ThetaGer;
int16_t Theta_err,ThetaSend,Theta_errSend;
float ErrK=0,num[6];
uint8_t Cnt = 0;
_iq iqThetaTest = 0;
extern IPARK Ipark;
//-------------------- public functions -------------------------------------
/*! \fn				void SVPWM( void )
 *  \brief 		SVPWM process
 *  \param 		none
 *  \return 	none
 */
void SVPWM ( void )
{
    SVPWMStruct.tmp1= SVPWMStruct.Ubeta;
    SVPWMStruct.tmp2= _IQdiv2(SVPWMStruct.Ubeta) + (_IQrmpy(_IQ(0.866),SVPWMStruct.Ualpha));
    SVPWMStruct.tmp3= SVPWMStruct.tmp2 - SVPWMStruct.tmp1;
//    SVPWMStruct.VecSector = (SVPWMStruct.tmp1 > 0.0) + ((SVPWMStruct.tmp2 > 0.0) << 1) + ((SVPWMStruct.tmp3 > 0.0) << 2); // N=4*C+2*B+A

    SVPWMStruct.VecSector=3;
    SVPWMStruct.VecSector=(SVPWMStruct.tmp2> 0)?( SVPWMStruct.VecSector-1):SVPWMStruct.VecSector;
    SVPWMStruct.VecSector=(SVPWMStruct.tmp3> 0)?( SVPWMStruct.VecSector-1):SVPWMStruct.VecSector;
    SVPWMStruct.VecSector=(SVPWMStruct.tmp1< 0)?(7-SVPWMStruct.VecSector) :SVPWMStruct.VecSector;
    
    if(SVPWMStruct.VecSector==1 || SVPWMStruct.VecSector==4) {
        SVPWMStruct.Ta= SVPWMStruct.tmp2;
        SVPWMStruct.Tb= SVPWMStruct.tmp1-SVPWMStruct.tmp3;
        SVPWMStruct.Tc=-SVPWMStruct.tmp2;
    } else if(SVPWMStruct.VecSector==2 || SVPWMStruct.VecSector==5) {
        SVPWMStruct.Ta= SVPWMStruct.tmp3+SVPWMStruct.tmp2;
        SVPWMStruct.Tb= SVPWMStruct.tmp1;
        SVPWMStruct.Tc=-SVPWMStruct.tmp1;
    } else {
        SVPWMStruct.Ta= SVPWMStruct.tmp3;
        SVPWMStruct.Tb=-SVPWMStruct.tmp3;
        SVPWMStruct.Tc=-(SVPWMStruct.tmp1+SVPWMStruct.tmp2);
    }

//    SVPWMStruct.Theta = _IQDeg2PI_Hz ( VVVFStruct.iqAngle_Step, VVVFStruct.iqMotor_Spd );
    
//    if ( ( SVPWMStruct.Theta >= 0 ) && ( SVPWMStruct.Theta < _IQDeg2PI ( 60 ) ) )  //Ta、Tb、Tc：_IQ(0)~_IQ(1)
//    {
//        SVPWMStruct.Ta = _IQdiv2 ( _IQ ( 1 ) + _IQ16cos ( SVPWMStruct.Theta - _IQDeg2PI ( 30 ) ) );
//        SVPWMStruct.Tb = _IQ ( 1 ) + _IQ16sin ( SVPWMStruct.Theta ) - SVPWMStruct.Ta;
//        SVPWMStruct.Tc = _IQ ( 1 ) - SVPWMStruct.Ta;
//    }
//    else if ( ( SVPWMStruct.Theta >= _IQDeg2PI ( 60 ) ) && ( SVPWMStruct.Theta < _IQDeg2PI ( 120 ) ) )
//    {
//        SVPWMStruct.Ta = _IQ16sin ( SVPWMStruct.Theta + _IQDeg2PI ( 60 ) ) + _IQdiv2 ( _IQ ( 1 ) - _IQ16sin ( SVPWMStruct.Theta ) );
//        SVPWMStruct.Tb = _IQdiv2 ( _IQ ( 1 ) + _IQ16sin ( SVPWMStruct.Theta ));
//        SVPWMStruct.Tc = _IQ ( 1 ) - SVPWMStruct.Tb;
//    }
//    else if ( ( SVPWMStruct.Theta >= _IQDeg2PI ( 120 ) ) && ( SVPWMStruct.Theta < _IQDeg2PI ( 180 ) ) )
//    {
//        SVPWMStruct.Ta = _IQdiv2 ( _IQ ( 1 ) - _IQ16sin ( SVPWMStruct.Theta - _IQDeg2PI ( 60 ) ) );
//        SVPWMStruct.Tb = _IQ ( 1 ) - SVPWMStruct.Ta;
//        SVPWMStruct.Tc = -_IQ16sin ( SVPWMStruct.Theta + _IQDeg2PI ( 60 ) ) + SVPWMStruct.Ta;
//    }
//    else if ( ( SVPWMStruct.Theta >= _IQDeg2PI ( 180 ) ) && ( SVPWMStruct.Theta < _IQDeg2PI ( 240 ) ) )
//    {
//        SVPWMStruct.Ta = _IQdiv2 ( _IQ ( 1 ) + _IQ16cos ( SVPWMStruct.Theta - _IQDeg2PI ( 30 ) ) );
//        SVPWMStruct.Tb = _IQ16sin ( SVPWMStruct.Theta - _IQDeg2PI ( 60 ) ) + SVPWMStruct.Ta;
//        SVPWMStruct.Tc = _IQ ( 1 ) - SVPWMStruct.Ta;
//    }
//    else if ( ( SVPWMStruct.Theta >= _IQDeg2PI ( 240 ) ) && ( SVPWMStruct.Theta < _IQDeg2PI ( 300 ) ) )
//    {
//        SVPWMStruct.Ta = _IQ16sin ( _IQDeg2PI ( 60 ) - SVPWMStruct.Theta ) + _IQdiv2 ( _IQ ( 1 ) + _IQ16sin ( SVPWMStruct.Theta ) );
//        SVPWMStruct.Tb = _IQdiv2( _IQ ( 1 )  + _IQ16sin ( SVPWMStruct.Theta ) );
//        SVPWMStruct.Tc = _IQ ( 1 ) - SVPWMStruct.Tb;
//    }
//    else if ( ( SVPWMStruct.Theta >= _IQDeg2PI ( 300 ) ) && ( SVPWMStruct.Theta < _IQDeg2PI ( 360 ) ) )
//    {
//        SVPWMStruct.Ta = _IQdiv2 ( _IQ ( 1 ) + _IQ16cos ( SVPWMStruct.Theta + _IQDeg2PI ( 30 ) ) );
//        SVPWMStruct.Tb = _IQ ( 1 ) - SVPWMStruct.Ta;
//        SVPWMStruct.Tc = -_IQ16sin ( SVPWMStruct.Theta ) + SVPWMStruct.Tb;
//    }
//    else
//    {
//        ;
//    }

//    SVPWMStruct.Ta -= _IQ(0.5);  //-_IQ(0.5)~_IQ(0.5)
//    SVPWMStruct.Tb -= _IQ(0.5);
//    SVPWMStruct.Tc -= _IQ(0.5);
//    
//    SVPWMStruct.Ta = SVPWMStruct.Ta * ( VVVFStruct.iqMotor_Cur1A_PWM + VVVFStruct.iqMotor_SpdKe_PWM ) / _IQ ( 1 );
//    SVPWMStruct.Tb = SVPWMStruct.Tb * ( VVVFStruct.iqMotor_Cur1A_PWM + VVVFStruct.iqMotor_SpdKe_PWM ) / _IQ ( 1 );
//    SVPWMStruct.Tc = SVPWMStruct.Tc * ( VVVFStruct.iqMotor_Cur1A_PWM + VVVFStruct.iqMotor_SpdKe_PWM ) / _IQ ( 1 );
//    
//    SVPWMStruct.Ta += _IQ(0.5);
//    SVPWMStruct.Tb += _IQ(0.5);
//    SVPWMStruct.Tc += _IQ(0.5);

//    SVPWMStruct.Ta = _IQsat(SVPWMStruct.Ta,_IQ(1),_IQ(0));
//    SVPWMStruct.Tb = _IQsat(SVPWMStruct.Tb,_IQ(1),_IQ(0));
//    SVPWMStruct.Tc = _IQsat(SVPWMStruct.Tc,_IQ(1),_IQ(0));
//    
//    ATIM_TIMX_CPLM_CHY_CCRY1 = _IQsat ( _IQdiv ( ( _IQmpy ( SVPWMStruct.Ta, TIM1->ARR ) ), _IQ ( 1 ) ), TIM1->ARR, 0 );
//    ATIM_TIMX_CPLM_CHY_CCRY2 = _IQsat ( _IQdiv ( ( _IQmpy ( SVPWMStruct.Tb, TIM1->ARR ) ), _IQ ( 1 ) ), TIM1->ARR, 0 );
//    ATIM_TIMX_CPLM_CHY_CCRY3 = _IQsat ( _IQdiv ( ( _IQmpy ( SVPWMStruct.Tc, TIM1->ARR ) ), _IQ ( 1 ) ), TIM1->ARR, 0 );
    
    TimerPWM_DutySet ( SVPWMStruct.Ta, SVPWMStruct.Tb, SVPWMStruct.Tc );
}
//-----------------------End of file------------------------------------------