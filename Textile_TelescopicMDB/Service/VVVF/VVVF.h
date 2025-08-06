/**
* 版权所有(C)
*
* ********
*
* @file VVVF.h
* @brief
* @details
* @author MuNiu
* @version 1.0.0
* @date xxxx-xx-xx
* @par Description:
* @par Change History:
*
* <日期> | <版本> | <作者> | <描述>
*
* xxxx/xx/xx | 1.0.0 | MuNiu | 创建文件
*
*/

#ifndef _VVVF_H_
#define _VVVF_H_
//-------------------- include files ----------------------------------------
#include "stm32f4xx_hal.h"
#include "IQmath.h"

//-------------------- public definitions -----------------------------------
#define Motor_PolePairs          5
#define Motor_SpdMax             3000
#define Motor_Ke                 524  //24/Motor_SpdMax*65535

//-------------------- public data -----------------------------------------
typedef struct 	{
    uint8_t u8Enable;
    _iq iqMotor_Spd;
    _iq iqAngle_Fre;
    _iq iqAngle_Step;
    _iq iqAngle_MaxStep;
    _iq iqAngle_MaxStepLast;
    _iq iqMotor_Cur1A_PWM;  //电流和占空比关系：I = 0.009 * PWM^2 - 0.1
    _iq iqMotor_SpdKe_PWM;
    _iq iqMotor_VVVFMaxSpd;
} VVVFDef;

//-------------------- public data -----------------------------------------
typedef struct 	{
    int32_t AngleSum;
    int32_t AngleAdd;
    int32_t Angle_ErrSum;
    uint32_t AngleTimeCnt;
    uint8_t MotorElcAngStep;
    uint8_t MotorElcAngCnt;
    uint8_t RunDir;
    uint16_t LastAngle;
    uint16_t i;
    uint16_t j; 
    int16_t Theta_ErrAvg_P;
    int16_t Theta_ErrAvg_N;
    int16_t Theta_Err;
    _iq Motor_Angle_Offset;
    _iq ElecAngle;
    
} ElcAngDef;

extern VVVFDef VVVFStruct;
extern ElcAngDef ElcAngCal;
//-------------------- public functions -------------------------------------
void VVVF_Ctrl ( void );
void MotorElcAng ( uint8_t PolesNum );
//-------------------- inline functions -------------------------------------
#define VVVFDef_DEFAULTS {                \
0,           /*u8Enable*/                 \
0,           /*iqMotor_Spd*/              \
0,           /*iqAngle_Fre*/              \
0,           /*iqAngle_Step*/             \
0,           /*iqAngle_MaxStep*/          \
0,           /*iqAngle_MaxStepLast*/      \
_IQ(0.10),   /*u8Motor_Cur1A_PWM*/        \
0,           /*iqMotor_SpdKe_PWM*/        \
120,         /*iqMotor_VVVFMaxSpd*/       \
}
/* u8Motor_Cur1A_PWM:_IQ(0.15)对应3A，可在标定码盘时使用 */

#define ElcAngDef_DEFAULTS {              \
0,\
0,\
0,\
0,\
0,\
0,\
0,\
0,\
0,\
0,\
0,\
0,\
0,\
22076,    /*Motor_Angle_Offset(测试写死)*/  \
0,\
}

#endif/* _VVVF_H_ */
//-----------------------End of file------------------------------------------
/** @}*/











