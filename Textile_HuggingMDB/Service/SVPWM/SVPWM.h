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

#ifndef _SVPWM_H_
#define _SVPWM_H_
//-------------------- include files ----------------------------------------
#include "stm32f4xx_hal.h"
#include "IQmath.h"

//-------------------- public definitions -----------------------------------
#define FRE_20KHZ            20000
#define PI                   3.141592654
#define _IQDeg2PI(x)         x*PI/180*65535  //0-360:(0-2PI)*_IQ(1)
#define _IQDeg2PI_Hz(x,y)    (x+1)*PI/(FRE_20KHZ*6/y)*65535
#define _IQDeg_Hz(x,y)       (x+1)*65535/(FRE_20KHZ*12/y)  //0-360电角度代表_IQ(1)
#define PWM_DT_COMP            _IQ(0.02)   //0.02 = 1/50

//-------------------- public data -----------------------------------------
typedef struct 	{
    _iq  Theta;
    _iq  Ualpha; 			// Input: reference alpha-axis phase voltage
    _iq  Ubeta;			// Input: reference beta-axis phase voltage
    _iq  Ta;				// Output: reference phase-a switching function
    _iq  Tb;				// Output: reference phase-b switching function
    _iq  Tc;				// Output: reference phase-c switching function
    _iq  tmp1;			// Variable: temp variable
    _iq  tmp2;			// Variable: temp variable
    _iq  tmp3;			// Variable: temp variable
    uint16_t VecSector;		// Space vector sector
} SVPWMDef;
typedef struct 
{
    _iq Ud;  // d-axis current
    _iq Uq;  // q-axis current
} UDQ_Def;
extern SVPWMDef SVPWMStruct;
extern uint8_t Step;
extern int16_t Theta_err;
extern uint16_t Theta,ThetaGer;

//-------------------- public functions -------------------------------------
void SVPWM ( void );

//-------------------- inline functions -------------------------------------
#define PWM_DT_Compensation(Duty)	if(Duty>_IQ(0.5)) Duty += PWM_DT_COMP;\
                                  else if(Duty<_IQ(0.5)) Duty -= PWM_DT_COMP;

#endif/* _SVPWM_H_ */
//-----------------------End of file------------------------------------------
/** @}*/











