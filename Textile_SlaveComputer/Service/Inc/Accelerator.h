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

//-----------------------------------------------------------------------------
#ifndef _Accelerator__H_
#define _Accelerator__H_

#include"Driver.h"
#include "AngleMath.h"
#include "Adc.h"
//-------------------- include files ----------------------------------------

//-------------------- public definitions -----------------------------------

//-------------------- public data ------------------------------------------
extern _iq g_iqGasPerc_Sta;
extern _iq g_iqBrakePerc_Sta;
//-------------------- public functions -------------------------------------

void Accl_Init ();
int8_t Accl_SetGas(_iq _iqGasPerc);
int8_t Accl_SetBrake(_iq _iqBrakePerc);

#endif // _Accelerator__H_

//-----------------------End of file------------------------------------------
/** @}*/
