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
    RGBMode_R_Live,             //红呼吸
    RGBMode_G_Live,             //绿呼吸
    RGBMode_B_Live,             //蓝呼吸
    RGBMode_Y_Live,
    RGBMode_R,                  //红常亮        4   蓝色
    RGBMode_G,                                    //hs
    RGBMode_B,                                    // lvs
    RGBMode_Y,                                   // zise
    RGBMode_RY_BLINK,           //红黄闪烁      8
    RGBMode_Y_BLINK,            //黄闪烁
    RGBMode_R_BLINK,            //红闪烁
    RGBMode_OFF,
    RGBMode_TEST,
}RGBMode;

//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
void Ctrl_SameColour ( RGBMode Mode );


#endif // _RGBLED_Process_H_

//-----------------------End of file------------------------------------------
/** @}*/
