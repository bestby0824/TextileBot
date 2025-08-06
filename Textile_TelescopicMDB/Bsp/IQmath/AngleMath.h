
//------------------------------------------------------------------------------

#ifndef _ANGLEMATH_H_
#define _ANGLEMATH_H_
//-------------------- include files ----------------------------------------
#include "stm32f4xx_hal.h"
#include "IQmath.h"

//-------------------- public definitions -----------------------------------



//-------------------- public data ------------------------------------------



//-------------------- public functions -------------------------------------
void Angle_IQ0_5(_iq *Angle);
void Angle_IQ0_IQ1(_iq *Angle);
void Angle_IQ0_IQ1_Avg(_iq *Angle1,_iq *Angle2,_iq *AngleOut);
//-------------------- inline functions -------------------------------------

#endif /* ANGLEMATH_H_ */
//-----------------------End of file------------------------------------------
/** */
