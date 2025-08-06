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
#ifndef _GlobalDefine__H_
#define _GlobalDefine__H_

#include"Driver.h"
#include "AngleMath.h"
//-------------------- include files ----------------------------------------

//-------------------- public definitions -----------------------------------

//-------------------- public data ------------------------------------------
extern _iq g_qHalfWheelTrack;	//�ּ��/2  	d
extern _iq g_qWheelBase;	//��� 				L
extern _iq g_qGravity;	//�������ٶ�	G
extern _iq g_qFriction; //Ħ��ϵ��		��
extern _iq g_qHightOfMass; //���ĸ߶�		h

extern _iq g_qWheelPerimeter;	//���ܳ�
extern _iq g_qMaxSteerAngle;		//���ת���


//�����м����
extern _iq g_qMaxAngleSpeed ;			//����ٶ�m/s
extern _iq g_qSafeVelocity ; 			//��ȫ�ٶ�		sv
extern _iq g_qRollOverParam;			//�෭ϵ��G*D*L (��Ҫ����H) 
extern _iq g_qSideSlipParam;			//�໬ϵ��G*��*L 
extern _iq g_qHOMThreshold;
//-------------------- public functions -------------------------------------


void GlobalDef_Init();


#endif // _GlobalDefine__H_

//-----------------------End of file------------------------------------------
/** @}*/
