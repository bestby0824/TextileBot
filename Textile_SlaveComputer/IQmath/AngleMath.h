/**
* 版权所有(C)
*
* @file ANGLEMATH.h
* @brief
* @details
* @author
* @version 0.0.1
* @date 2021-01-12
* @par Description:
* @par Change History:
*
* <日期> | <版本> | <作者> | <描述>
*
* 2021/01/12 | 0.0.1 | HWW | 创建文件
*
*/
//------------------------------------------------------------------------------
    
#ifndef _ANGLEMATH_H_
#define _ANGLEMATH_H_
//-------------------- include files ----------------------------------------
#include"Driver.h"
#include"IQmath.h"

//-------------------- public definitions -----------------------------------
#define Ver3_0              0
#define Ver3_5              1
#define Ver3_5_New          2

#define ForkLiftVer         Ver3_5_New

#define Ver_Old              0
#define Ver_New              1
#define ForkLiftPCBVer       1
//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
void Angle_IQ0_5(_iq *Angle);
void Angle_IQ0_IQ1(_iq *Angle);
void Angle_IQ0_IQ1_Avg(_iq *Angle1,_iq *Angle2,_iq *AngleOut);
//-------------------- inline functions -------------------------------------

#endif /* ANGLEMATH_H_ */
//-----------------------End of file------------------------------------------
/** */
