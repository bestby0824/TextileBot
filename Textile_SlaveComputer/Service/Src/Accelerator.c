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
#include "Accelerator.h"
#include "ErrorCode.h"
#include "GlobalDef.h"
//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
uint16_t g_unBrakePos = 0;
//-------------------- private functions declare ----------------------------

//-------------------- public data ------------------------------------------
_iq _iqPara_u = _IQ ( 0.3 );
_iq _iqPara_g = _IQ ( 9.8 );
_iq _iqPara_L = _IQ ( 1.725 );
_iq _iqPara_d = _IQ ( 0.555 );
_iq _iqPara_h = _IQ ( 1 );
_iq g_iqGasPerc_Sta = 0;
_iq g_iqBrakePerc_Sta = 0;
_iq g_qBaseVlotage = _IQ ( 0.08 );		//����ƽ
//-------------------- public functions -------------------------------------



void Accl_Init ()
{
//    HAL_GPIO_WritePin ( GPIOF, GPIO_PIN_7, GPIO_PIN_RESET );
//    Throttle_DAC_EN();
//    Throttle_DAC_Value ( g_qBaseVlotage );
}
/*! \fn
 *  \brief .
 *  \param
 *  \return
 */
int8_t Accl_SetGas ( _iq _iqGasPerc )
{

    if ( _iqGasPerc == 0 )  //���Ž���
    {
//        Throttle_DAC_DISEN();
        _iqGasPerc = _IQ ( 0.1 );
    }
    else
    {
//        Throttle_DAC_EN();
    }

    _iqGasPerc = _IQsat ( _iqGasPerc, _IQ ( 1.0 ), 0 );

//    Throttle_DAC_Value ( _iqGasPerc );

    g_iqGasPerc_Sta = _iqGasPerc;
    return Err_None;
}

int8_t Accl_SetBrake ( _iq _iqBrakePerc )
{
    /*���޸� SetBrakeValue ( _iqBrakePerc ); //  g_iqBrakePos��_IQ(1)ȫ��ɲ����_IQ(0)��̧��״̬*/
    g_iqBrakePerc_Sta = _iqBrakePerc;
    return Err_None;
}

_iq Accl_GetMaxSteerAngleBySpeed ( _iq qAngle, _iq qSpeed ) // qSpeed��λm/S
{
    _iq tmp1, tmp2;
    //�����ƣ����������
    if ( _iqPara_u < _IQdiv ( _iqPara_d, _iqPara_h ) )
    {
        tmp1 = _IQdiv ( _IQmpy ( _IQmpy ( _iqPara_u, _iqPara_g ), _iqPara_L ), _IQmpy ( qSpeed, qSpeed ) );
    }
    else
    {
        tmp1 = _IQdiv ( _IQmpy ( _IQmpy ( _iqPara_d, _iqPara_g ), _iqPara_L ), _IQmpy ( _iqPara_h, _IQmpy ( qSpeed, qSpeed ) ) );
    }
    tmp2 = _IQatan ( tmp1 );
    return tmp2;


}

//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


