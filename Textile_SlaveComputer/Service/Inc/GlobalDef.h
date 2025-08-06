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
#ifndef _GlobalDefine__H_
#define _GlobalDefine__H_

#include"Driver.h"
#include "AngleMath.h"
//-------------------- include files ----------------------------------------

//-------------------- public definitions -----------------------------------

//-------------------- public data ------------------------------------------
extern _iq g_qHalfWheelTrack;	//轮间距/2  	d
extern _iq g_qWheelBase;	//轴距 				L
extern _iq g_qGravity;	//重力加速度	G
extern _iq g_qFriction; //摩擦系数		μ
extern _iq g_qHightOfMass; //质心高度		h

extern _iq g_qWheelPerimeter;	//轮周长
extern _iq g_qMaxSteerAngle;		//最大转向角


//计算中间变量
extern _iq g_qMaxAngleSpeed ;			//最大速度m/s
extern _iq g_qSafeVelocity ; 			//安全速度		sv
extern _iq g_qRollOverParam;			//侧翻系数G*D*L (需要除以H) 
extern _iq g_qSideSlipParam;			//侧滑系数G*μ*L 
extern _iq g_qHOMThreshold;
//-------------------- public functions -------------------------------------


void GlobalDef_Init();


#endif // _GlobalDefine__H_

//-----------------------End of file------------------------------------------
/** @}*/
