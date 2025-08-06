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

//-------------------- include files ----------------------------------------
#include "GlobalDef.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------

//-------------------- private functions declare ----------------------------

//-------------------- public data ------------------------------------------
//��������
_iq g_qHalfWheelTrack = _IQ ( 0.555 );	//�ּ��/2  	d
_iq g_qWheelBase = _IQ ( 1.725 );			//��� 				L
_iq g_qGravity = _IQ ( 9.8 );					//�������ٶ�	G
_iq g_qFriction = _IQ ( 0.3 ); 				//Ħ��ϵ��		��
_iq g_qHightOfMass = _IQ ( 1 ); 				//���ĸ߶�		h Ĭ��״̬
_iq g_qMaxHightOfMass = _IQ ( 2.0 ); 		//������ĸ߶�	h Ĭ��״̬
_iq g_qWheelPerimeter = _IQ ( 2.041 );	//���ܳ�
_iq g_qMaxSteerAngle = _IQ ( 0.91 );		//���ת���


//�����м����
_iq g_qMaxAngleSpeed = _IQ ( 10 );			//����ٶ�m/s
_iq g_qSafeVelocity = _IQ ( 0 ); 			//��ȫ�ٶ�		sv
_iq g_qRollOverParam = _IQ ( 0 );			//�෭ϵ��G*D*L (��Ҫ����H)
_iq g_qSideSlipParam = _IQ ( 0 );			//�໬ϵ��G*��*L
_iq g_qHOMThreshold = _IQ ( 0 );			 //���ĸ߶����ޣ����ڴ����޼���෭�����ڴ����޼���໬
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