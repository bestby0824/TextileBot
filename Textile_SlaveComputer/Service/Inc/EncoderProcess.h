/**
* ��Ȩ����(C) 
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
* <����> | <�汾> | <����> | <����>
*
* xxxx/xx/xx | 1.0.0 | HWW | �����ļ�
*
*/
//------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
#ifndef _EncProc__H_
#define _EncProc__H_
//-------------------- include files ----------------------------------------
#include "AngleMath.h"
#include "TimerEncoder.h"
//-------------------- public definitions -----------------------------------

//-------------------- public data ------------------------------------------
#define _iqMaxSpd_rpm           _IQ(3469)  //��н��ٶ�16Km/hԼ110rpm
#define Wheel_D         _IQ(0.23)                                            //��ֱ��0.23m
#define Wheel_C         _IQrmpy(Wheel_D,_IQ(3.1416))                        //���ܳ� _IQ(1) = 1m

#define SpdMax_Line     _IQdiv(_IQrmpy(_iqMaxSpd_rpm, Wheel_C),_IQ(60))      //���ٶ� _IQ(1) = 1m/S

//-------------------- public functions -------------------------------------
void SpdClc ( uint16_t Freq );
void SpdClc_T ( uint16_t Freq );
_iq EncProc_GerSpdFromEncoder();

extern _iq g_Spd1;
extern _iq g_Spd2;
extern int16_t g_nLWheelPos;				//������λ�ã���ת���� 0 - 10000
extern int16_t g_nRWheelPos;				//������λ�ã���ת���� 0 - 10000
extern  _iq g_Spd1_T;
extern  _iq g_Spd2_T;
extern int16_t Timer1Counter ,Timer3Counter;
#endif // _EncProc__H_

//-----------------------End of file------------------------------------------
/** @}*/
