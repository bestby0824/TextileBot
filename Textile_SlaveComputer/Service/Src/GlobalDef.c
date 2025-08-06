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
#include "GlobalDef.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------

//-------------------- private functions declare ----------------------------

//-------------------- public data ------------------------------------------
//车量参数
_iq g_qHalfWheelTrack = _IQ ( 0.555 );	//轮间距/2  	d
_iq g_qWheelBase = _IQ ( 1.725 );			//轴距 				L
_iq g_qGravity = _IQ ( 9.8 );					//重力加速度	G
_iq g_qFriction = _IQ ( 0.3 ); 				//摩擦系数		μ
_iq g_qHightOfMass = _IQ ( 1 ); 				//质心高度		h 默认状态
_iq g_qMaxHightOfMass = _IQ ( 2.0 ); 		//最大质心高度	h 默认状态
_iq g_qWheelPerimeter = _IQ ( 2.041 );	//轮周长
_iq g_qMaxSteerAngle = _IQ ( 0.91 );		//最大转向角


//计算中间变量
_iq g_qMaxAngleSpeed = _IQ ( 10 );			//最大速度m/s
_iq g_qSafeVelocity = _IQ ( 0 ); 			//安全速度		sv
_iq g_qRollOverParam = _IQ ( 0 );			//侧翻系数G*D*L (需要除以H)
_iq g_qSideSlipParam = _IQ ( 0 );			//侧滑系数G*μ*L
_iq g_qHOMThreshold = _IQ ( 0 );			 //质心高度门限，高于此门限计算侧翻，低于此门限计算侧滑
//-------------------- public functions -------------------------------------

/*! \fn
 *  \brief .
 *  \param
 *  \return
 */

void GlobalDef_Init()
{
    g_qRollOverParam = _IQmpy ( g_qGravity, g_qHalfWheelTrack );
    g_qRollOverParam = _IQmpy ( g_qRollOverParam, g_qWheelBase );

    g_qSideSlipParam = _IQmpy ( g_qGravity, g_qHalfWheelTrack );
    g_qSideSlipParam = _IQmpy ( g_qSideSlipParam, g_qFriction );

    _iq qTemp = _IQ ( 0 );
    qTemp = _IQdiv ( g_qRollOverParam, g_qMaxHightOfMass );	//sqr(G*D*L/h/tan(w))
    qTemp = _IQmpy ( qTemp, _IQcos ( g_qMaxSteerAngle ) );
    qTemp = _IQdiv ( qTemp, _IQsin ( g_qMaxSteerAngle ) );
    g_qSafeVelocity = _IQsqrt ( qTemp );

}

//-----------------------End of file------------------------------------------
/** @} */ /* End of group */