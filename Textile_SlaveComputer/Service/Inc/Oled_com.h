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
* 2022/11/16 | 1.0.0 | HWW | 创建文件
*
*/
//------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
#ifndef _Oled_com__H_
#define _Oled_com__H_
//-------------------- include files ----------------------------------------
#include "IIC_Sim.h"

//-------------------- public definitions -----------------------------------
typedef enum{
    Oled_Sta_PreCharg = 0,
    Oled_Sta_PWRON,
    Oled_Sta_LightON,
    Oled_Sta_LineStart,
    Oled_Sta_RowStart_L,
    Oled_Sta_RowStart_H,
    Oled_Sta_Data,
    Oled_Sta_Halt,
}OledSta;
//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
void Oled_Init(void);
void OledStateMachine( void );
void OLED_Clear ( void );
void OLED_Refresh ( void );
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);
void OLED_DrawLine(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t mode);
void OLED_DrawCircle(uint8_t x,uint8_t y,uint8_t r);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size1,uint8_t mode);
void OLED_ShowChar6x8(uint8_t x,uint8_t y,uint8_t chr,uint8_t mode);
void OLED_ShowString(uint8_t x,uint8_t y,char *chr,uint8_t size1,uint8_t mode);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size1,uint8_t mode);
void OLED_ShowChinese(uint8_t x,uint8_t y,uint8_t num,uint8_t size1,uint8_t mode);
void OLED_ScrollDisplay(uint8_t num,uint8_t space,uint8_t mode);
void OLED_ShowPicture(uint8_t x,uint8_t y,uint8_t sizex,uint8_t sizey,uint8_t BMP[],uint8_t mode);

void OLED_ClearPoint(uint8_t x,uint8_t y);
void OLED_ColorTurn(uint8_t i);
#endif // _XINT_H_

//-----------------------End of file------------------------------------------
/** @}*/
