
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

    if(abs(*Angle1-*Angle2)>_IQ(0.5)) *AngleOut = _IQdiv2(*Angle1+*Angle2 -_IQ(1));
    else *AngleOut = _IQdiv2(*Angle1+*Angle2);

}
