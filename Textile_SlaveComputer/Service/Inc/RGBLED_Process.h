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
#ifndef _RGBLED_Process__H_
#define _RGBLED_Process__H_
//-------------------- include files ----------------------------------------
#include "Spi_RGBLED.h"
#include "IQmath.h"
//-------------------- public definitions -----------------------------------
enum {
    
    IndexR = 0,
    IndexB,
    IndexG,
};
typedef enum {
    RGBMode_R_Live,             //�����
    RGBMode_G_Live,             //�̺���
    RGBMode_B_Live,             //������
    RGBMode_Y_Live,
    RGBMode_R,                  //�쳣��        4   ��ɫ
    RGBMode_G,                                    //hs
    RGBMode_B,                                    // lvs
    RGBMode_Y,                                   // zise
    RGBMode_RY_BLINK,           //�����˸      8
    RGBMode_Y_BLINK,            //����˸
    RGBMode_R_BLINK,            //����˸
    RGBMode_OFF,
    RGBMode_TEST,
}RGBMode;

//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
void Ctrl_SameColour ( RGBMode Mode );


#endif // _RGBLED_Process_H_

//-----------------------End of file------------------------------------------
/** @}*/
