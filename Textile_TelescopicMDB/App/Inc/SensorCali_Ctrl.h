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
* xxxx/xx/xx | 1.0.0 | MUNIU | 创建文件
*/
//------------------------------------------------------------------------------
//-----------------------------------------------------------------------------
#ifndef _SensorCali_Ctrl__H_
#define _SensorCali_Ctrl__H_
//-------------------- include files ----------------------------------------
#include "stm32f4xx_hal.h"
#include "HostProcess.h"

//-------------------- public definitions -----------------------------------
typedef enum
{
    Sensor_None = 0,
    Sensor_MotorCoder,
    Sensor_PosCoder,    
} Sensor_Type;

typedef struct
{
    uint16_t u16MotorCaliEnable;
    uint16_t u16PosCoderCaliEnable;
} SensorCali_TypeDef;
extern SensorCali_TypeDef SensorCali_Handle;
extern RegDef_SensorCaliFB SensorCaliFB;
//-------------------- public functions -------------------------------------
void SensorCali_Ctrl ( void );


#endif // _SensorCali_Ctrl__H_

//-----------------------End of file------------------------------------------
/** @}*/
