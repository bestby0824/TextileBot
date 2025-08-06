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
#include "RGBLED_Process.h"

//-------------------- local definitions ------------------------------------
_iq _iqBrightness = _IQ ( 0.95 );
uint16_t CircleT = 300;
//-------------------- private data -----------------------------------------
uint8_t u8RGB_Value[RgbLed_Num * 3];
//-------------------- private functions declare ----------------------------
static void SetAll_SameColour ( _iq _iqBrightness, uint8_t u8Value_G, uint8_t u8Value_R, uint8_t u8Value_B );
static void SetRadarF ( _iq _iqBrightness, uint8_t u8Value_G, uint8_t u8Value_R, uint8_t u8Value_B );
static void SetRadarB ( _iq _iqBrightness, uint8_t u8Value_G, uint8_t u8Value_R, uint8_t u8Value_B );
static void SetRadarFB ( _iq _iqBrightness, uint8_t u8Value_G, uint8_t u8Value_R, uint8_t u8Value_B );

//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------

/*! \fn     void Ctrl_SameColour ( RGBMode Mode )
 *  \brief  调用频率100Hz,用来设置LED彩色灯带闪烁状态
 *  \param  RGBMode Mode
 *  \return
 */

void Ctrl_SameColour ( RGBMode Mode )
{
    static uint32_t u32ColorCnt, TestNum = 0;
    _iq _iqBrightness_Ref;
    uint8_t u8Value_G_Ref, u8Value_R_Ref, u8Value_B_Ref;

    if ( u32ColorCnt < CircleT ) {
        u32ColorCnt++;
        if ( u32ColorCnt == ( CircleT >> 1 ) ) {
            TestNum++;
        }
    } else {
        u32ColorCnt = 0;
    }
    switch ( Mode )
    {
    case RGBMode_R_Live:
    {
        CircleT = 300;
        _iqBrightness_Ref = _IQabs ( _IQmpy ( _iqBrightness, _IQdiv ( u32ColorCnt - _IQdiv2 ( CircleT ), _IQdiv2 ( CircleT ) ) ) );
        u8Value_R_Ref = 255;
        u8Value_G_Ref = 0;
        u8Value_B_Ref = 0; //绿呼吸

    }
    break;
    case RGBMode_G_Live:
    {
        CircleT = 300;
        _iqBrightness_Ref = _IQabs ( _IQmpy ( _iqBrightness, _IQdiv ( u32ColorCnt - _IQdiv2 ( CircleT ), _IQdiv2 ( CircleT ) ) ) );
        u8Value_R_Ref = 0;
        u8Value_G_Ref = 255;
        u8Value_B_Ref = 0;

    }
    break;

    case RGBMode_B_Live:
    {
        CircleT = 300;
        _iqBrightness_Ref = _IQabs ( _IQmpy ( _iqBrightness, _IQdiv ( u32ColorCnt - _IQdiv2 ( CircleT ), _IQdiv2 ( CircleT ) ) ) );
        u8Value_R_Ref = 0;
        u8Value_G_Ref = 0;
        u8Value_B_Ref = 255;

    }
    break;
    case RGBMode_Y_Live:
    {
        CircleT = 300;
        _iqBrightness_Ref = _IQabs ( _IQmpy ( _iqBrightness, _IQdiv ( u32ColorCnt - _IQdiv2 ( CircleT ), _IQdiv2 ( CircleT ) ) ) );
        u8Value_R_Ref = 255;
        u8Value_G_Ref = 150;
        u8Value_B_Ref = 0;

    }
    break;
    case RGBMode_R:
    {
        _iqBrightness_Ref = _iqBrightness;
        u8Value_R_Ref = 255;
        u8Value_G_Ref = 0;
        u8Value_B_Ref = 0; //红常亮
    }
    break;
    case RGBMode_G:
    {
        _iqBrightness_Ref = _iqBrightness;
        u8Value_R_Ref = 0;
        u8Value_G_Ref = 255;
        u8Value_B_Ref = 0; //绿常亮
    }
    break;
    case RGBMode_B:
    {
        _iqBrightness_Ref = _iqBrightness;
        u8Value_R_Ref = 0;
        u8Value_G_Ref = 0;
        u8Value_B_Ref = 255; //蓝常亮
    }
    break;
    case RGBMode_Y:              //黄常亮
    {
        _iqBrightness_Ref = _iqBrightness;
        u8Value_R_Ref = 255;
        u8Value_G_Ref = 150;
        u8Value_B_Ref = 0;
    }
    break;
    case RGBMode_RY_BLINK:
    {
        CircleT = 30;
        _iqBrightness_Ref = _iqBrightness;
        if ( TestNum % 2 == 1 )
        {
            u8Value_R_Ref = 255;
            u8Value_G_Ref = 0;
            u8Value_B_Ref = 0;
        } else
        {
            u8Value_R_Ref = 255;
            u8Value_G_Ref = 150;
            u8Value_B_Ref = 0;
        }
    }
    break;
    case RGBMode_Y_BLINK:
    {
        CircleT = 30;
        _iqBrightness_Ref = _iqBrightness;
        if ( TestNum % 2 == 1 )
        {
            u8Value_R_Ref = 0;
            u8Value_G_Ref = 0;
            u8Value_B_Ref = 0;
        } else
        {
            u8Value_R_Ref = 255;
            u8Value_G_Ref = 150;
            u8Value_B_Ref = 0;
        }
    }
    break;
    case RGBMode_R_BLINK:
    {
        CircleT = 30;
        _iqBrightness_Ref = _iqBrightness;
        if ( TestNum % 2 == 1 )
        {
            u8Value_R_Ref = 0;
            u8Value_G_Ref = 0;
            u8Value_B_Ref = 0;
        } else
        {
            u8Value_R_Ref = 255;
            u8Value_G_Ref = 0;
            u8Value_B_Ref = 0;
        }
    }
    break;
    case RGBMode_OFF:
    {
        _iqBrightness_Ref = _iqBrightness;
        u8Value_R_Ref = 0;
        u8Value_G_Ref = 0;
        u8Value_B_Ref = 0; //灭
    }
    break;

    case RGBMode_TEST:
    {
        CircleT = 300;
        _iqBrightness_Ref = _IQabs ( _IQmpy ( _iqBrightness, _IQdiv ( u32ColorCnt - _IQdiv2 ( CircleT ), _IQdiv2 ( CircleT ) ) ) );
        if ( TestNum % 4 == 0 )
        {
            u8Value_R_Ref = 255;
            u8Value_G_Ref = 0;
            u8Value_B_Ref = 0;
        } else if ( TestNum % 4 == 1 )
        {
            u8Value_R_Ref = 0;
            u8Value_G_Ref = 255;
            u8Value_B_Ref = 0;
        } else if ( TestNum % 4 == 2 )
        {
            u8Value_R_Ref = 0;
            u8Value_G_Ref = 0;
            u8Value_B_Ref = 255;
        } else
        {
            u8Value_R_Ref = 255;
            u8Value_G_Ref = 255;
            u8Value_B_Ref = 255;
        }

    }
    break;

    }

//    if(RGBMode_RUN_F == Mode)
//    {
//        SetRadarF ( _iqBrightness_Ref, u8Value_G_Ref, u8Value_R_Ref, u8Value_B_Ref );
//    }
//    else if(RGBMode_RUN_B == Mode)
//    {
//        SetRadarB ( _iqBrightness_Ref, u8Value_G_Ref, u8Value_R_Ref, u8Value_B_Ref );
//    }
//    else if(RGBMode_RUN_FB == Mode)
//    {
//        SetRadarFB ( _iqBrightness_Ref, u8Value_G_Ref, u8Value_R_Ref, u8Value_B_Ref );
//    }else
    {
        SetAll_SameColour ( _iqBrightness_Ref, u8Value_G_Ref, u8Value_R_Ref, u8Value_B_Ref );
    }
}

static void SetAll_SameColour ( _iq _iqBrightness, uint8_t u8Value_G, uint8_t u8Value_R, uint8_t u8Value_B )
{
    uint8_t u8Set_G, u8Set_R, u8Set_B;
    uint16_t i;
    u8Set_G = _IQmpy ( _iqBrightness, u8Value_G );
    u8Set_R = _IQmpy ( _iqBrightness, u8Value_R );
    u8Set_B = _IQmpy ( _iqBrightness, u8Value_B );

    for ( i = 0; i < RgbLed_Num; i++ )
    {
        u8RGB_Value[i * 3 + IndexG] = u8Set_G;
        u8RGB_Value[i * 3 + IndexR] = u8Set_R;
        u8RGB_Value[i * 3 + IndexB] = u8Set_B;
    }
    Spi_RgbLed_Send ( u8RGB_Value, RgbLed_Num * 3 );
}
static void SetRadarF ( _iq _iqBrightness, uint8_t u8Value_G, uint8_t u8Value_R, uint8_t u8Value_B )
{
    uint8_t u8Set_G, u8Set_R, u8Set_B;
    uint16_t i;
    u8Set_G = _IQmpy ( _iqBrightness, u8Value_G );
    u8Set_R = _IQmpy ( _iqBrightness, u8Value_R );
    u8Set_B = _IQmpy ( _iqBrightness, u8Value_B );

    for ( i = 0; i < RgbLed_Num; i++ )
    {
        u8RGB_Value[i * 3 + IndexG] = u8Set_G;
        u8RGB_Value[i * 3 + IndexR] = u8Set_R;
        u8RGB_Value[i * 3 + IndexB] = u8Set_B;
    }
    for ( i = 0; i < 2; i++ )
    {
        u8RGB_Value[i * 3 + IndexG] = 200;
        u8RGB_Value[i * 3 + IndexR] = 200;
        u8RGB_Value[i * 3 + IndexB] = 200;
    }
    Spi_RgbLed_Send ( u8RGB_Value, RgbLed_Num * 3 );
}
static void SetRadarB ( _iq _iqBrightness, uint8_t u8Value_G, uint8_t u8Value_R, uint8_t u8Value_B )
{
    uint8_t u8Set_G, u8Set_R, u8Set_B;
    uint16_t i;
    u8Set_G = _IQmpy ( _iqBrightness, u8Value_G );
    u8Set_R = _IQmpy ( _iqBrightness, u8Value_R );
    u8Set_B = _IQmpy ( _iqBrightness, u8Value_B );

    for ( i = 0; i < RgbLed_Num; i++ )
    {
        u8RGB_Value[i * 3 + IndexG] = u8Set_G;
        u8RGB_Value[i * 3 + IndexR] = u8Set_R;
        u8RGB_Value[i * 3 + IndexB] = u8Set_B;
    }
    for ( i = 5; i < 7; i++ )
    {
        u8RGB_Value[i * 3 + IndexG] = 200;
        u8RGB_Value[i * 3 + IndexR] = 200;
        u8RGB_Value[i * 3 + IndexB] = 200;
    }
    Spi_RgbLed_Send ( u8RGB_Value, RgbLed_Num * 3 );
}
static void SetRadarFB ( _iq _iqBrightness, uint8_t u8Value_G, uint8_t u8Value_R, uint8_t u8Value_B )
{
    uint8_t u8Set_G, u8Set_R, u8Set_B;
    uint16_t i;
    u8Set_G = _IQmpy ( _iqBrightness, u8Value_G );
    u8Set_R = _IQmpy ( _iqBrightness, u8Value_R );
    u8Set_B = _IQmpy ( _iqBrightness, u8Value_B );

    for ( i = 0; i < RgbLed_Num; i++ )
    {
        u8RGB_Value[i * 3 + IndexG] = u8Set_G;
        u8RGB_Value[i * 3 + IndexR] = u8Set_R;
        u8RGB_Value[i * 3 + IndexB] = u8Set_B;
    }
    for ( i = 0; i < 2; i++ )
    {
        u8RGB_Value[i * 3 + IndexG] = 200;
        u8RGB_Value[i * 3 + IndexR] = 200;
        u8RGB_Value[i * 3 + IndexB] = 200;
    }
    for ( i = 5; i < 7; i++ )
    {
        u8RGB_Value[i * 3 + IndexG] = 200;
        u8RGB_Value[i * 3 + IndexR] = 200;
        u8RGB_Value[i * 3 + IndexB] = 200;
    }
    Spi_RgbLed_Send ( u8RGB_Value, RgbLed_Num * 3 );
}

static void Set_WaterFall_Colour ( _iq _iqBrightness, uint8_t u8Value_G, uint8_t u8Value_R, uint8_t u8Value_B )
{
    uint8_t u8Set_G, u8Set_R, u8Set_B;
    uint16_t i;
    u8Set_G = _IQmpy ( _iqBrightness, u8Value_G );
    u8Set_R = _IQmpy ( _iqBrightness, u8Value_R );
    u8Set_B = _IQmpy ( _iqBrightness, u8Value_B );

    for ( i = 0; i < RgbLed_Num; i++ )
    {
        u8RGB_Value[i * 3 + IndexG] = u8Set_G;
        u8RGB_Value[i * 3 + IndexR] = u8Set_R;
        u8RGB_Value[i * 3 + IndexB] = u8Set_B;
    }
    Spi_RgbLed_Send ( u8RGB_Value, RgbLed_Num * 3 );
}

//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


