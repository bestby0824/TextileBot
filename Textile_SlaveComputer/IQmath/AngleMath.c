/**
* 版权所有(C)
*
* @file AngleMath.c
* @brief
* @details
* @author
* @version 0.0.1
* @date 2021-01-12
* @par Description:
* @par Change History:
*
* <日期> | <版本> | <作者> | <描述>
*
* 2021/01/12 | 0.0.1 | HWW | 创建文件
*
*/
//-------------------- pragmas ----------------------------------------------

//-------------------- include files ----------------------------------------
#include "AngleMath.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------

//-------------------- private functions declare ----------------------------


//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------

void Angle_IQ0_5(_iq *Angle)
{
    uint8_t Cnt;
    Cnt = 0;
    while((*Angle > _IQ(0.5))&&(Cnt<100))	{
        *Angle -= _IQ(1);
        Cnt++;
    }
    while((*Angle < _IQ(-0.5))&&(Cnt<100))	{
        *Angle += _IQ(1);
        Cnt++;
    }
}
void Angle_IQ0_IQ1(_iq *Angle)
{
    uint8_t Cnt;
    Cnt = 0;
    while((*Angle > _IQ(1))&&(Cnt<100))	{
        *Angle -= _IQ(1);
        Cnt++;
    }
    while((*Angle < _IQ(0))&&(Cnt<100))	{
        *Angle += _IQ(1);
        Cnt++;
    }
}

void Angle_IQ0_IQ1_Avg(_iq *Angle1,_iq *Angle2,_iq *AngleOut)
{

    if(abs(*Angle1-*Angle2)>_IQ(0.5)) *AngleOut = _IQdiv(*Angle1+*Angle2 -_IQ(1),_IQ(2));
    else *AngleOut = _IQdiv(*Angle1+*Angle2,_IQ(2));

}
