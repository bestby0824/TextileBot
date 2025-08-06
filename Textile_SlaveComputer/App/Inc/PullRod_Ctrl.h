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
#ifndef _SolenoidCtrl__H_
#define _SolenoidCtrl__H_
//-------------------- include files ----------------------------------------
#include "pullrodProcess.h"
#include "Host_Com.h"
//-------------------- public definitions -----------------------------------

typedef enum {
    Dev_Foward = 0,
    Dev_Reverse,
    Dev_Stop,
} SolenoidDev_Cmd;

#define Q_HeightMax       _IQ(6)
#define Q_ClawMax         _IQ(1)
#define Q_BowRiseMax      _IQ(1)
#define Q_RotateMax       _IQ(1)
#define Q_BrakeMax        _IQ(1)

#define Q_SensMax        {  \
    Q_HeightMax,            \
    Q_ClawMax,              \
    Q_BowRiseMax,           \
    Q_RotateMax,            \
    Q_BrakeMax,             \
}
//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
void Solenoid_Init(void);
void Solenoid_Ctrl(void);


#endif // _XXX_H_

//-----------------------End of file------------------------------------------
/** @}*/
