/**
* 版权所有(C)
*
* ********
*
* @file Int_Ctrl.c
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

#ifndef _MOTORINFO_H_
#define _MOTORINFO_H_
//-------------------- include files ----------------------------------------
#include "stm32f4xx_hal.h"
#include "AngleMath.h"

//-------------------- public definitions -----------------------------------
#define Freq_50        50
#define Freq_500       500
#define Freq_1K        1000
#define Freq_2K        2000
#define Freq_4K        4000
#define Freq_20K       20000 

//-------------------- public data -----------------------------------------
typedef enum
{
    CURRENT_MODE_AB,
    CURRENT_MODE_BC,
    CURRENT_MODE_AC

} E_CURRENT_MODE;
extern E_CURRENT_MODE eCurrentMode;

typedef struct
{
    _iq iqCurrentA;        //各项电流,定点小数,_IQ(1)表示 1A
    _iq iqCurrentB;
    _iq iqCurrentC;
    _iq iqEffectCurrent;   //等效相电流

    _iq iqSpd_PU;          //电机轴转速,定点小数,_IQ(1)表示最大转速
    _iq iqOutSpd;          //出轴转速,定点小数,_IQ(1)表示1RPM
    _iq iqEAngle;          //电机电角度,定点小数,_IQ(0)~_IQ(1)表示0~360度
    _iq iqPosAbs;          //电机输出轴绝对位置角度,定点小数,_IQ(0)~_IQ(1)表示0~360度
    _iq iqPosAdd;          //电机输出轴相累计位置角度,定点小数,_IQ(1)表示360度

    _iq iqVoltA;           //各项电压,定点小数,_IQ(1)表示 1V
    _iq iqVoltB;
    _iq iqVoltC;
    _iq iqPressure;
    _iq iqVoltIN;          //输入电压,定点小数,_IQ(1)表示 1V

    int16_t  s16Temp;       //MOS温度,有符号整型,0.01摄氏度
    uint16_t u16Pressure_State;
    uint16_t u16MotorVol_State;
    uint16_t u16Check_State;
    uint16_t u16Switch_State;
} S_MOTOR_INFO;
extern S_MOTOR_INFO g_sMotor_Info;

typedef struct {
    uint16_t current_initpos;
    uint16_t current_position;     // 当前编码器位置(原始值)
    uint16_t last_position;        // 上次编码器位置
    int32_t total_ticks;           // 累计脉冲数(考虑溢出)
    int32_t revolution_count;      // 旋转圈数
    _iq linear_distance;        // 线性移动距离(米)
} Encoder_Struct;
extern Encoder_Struct Encoder_Handle;

typedef struct {
    _iq Angle_Coder_Current;
    _iq Angle_Coder_Last;
    _iq Angle_Coder_Total;
    _iq Angle_Delta;
    _iq Angle_Current;
    _iq Length_Pos;
} TrigFunc_Struct;
extern TrigFunc_Struct TrigFunc_Handle;

extern _iq iqNowAddPos;
//-------------------- public functions -------------------------------------
void MotorInfo_OutSpdCalc ( S_MOTOR_INFO *psInfo, uint16_t CalcFreq );
void MotorInfo_PosAbsCalc ( S_MOTOR_INFO *psInfo, uint16_t CalcFreq );
void MotorInfo_GetCurrent ( S_MOTOR_INFO *psInfo, uint16_t CalcFreq );
void MotorInfo_EAngleCalc ( S_MOTOR_INFO *psInfo, uint16_t CalcFreq );
void MotorInfo_Pressure ( S_MOTOR_INFO *psInfo );
void MotorInfo_Switch ( S_MOTOR_INFO *psInfo );
void MotorInfo_Init ( void );
void Encoder_PosClc( Encoder_Struct *encoder, uint8_t u8Enable_Clc );
void TrigFunc_Clc ( TrigFunc_Struct *TrigFunc, uint8_t u8Enable_Clc );

//-------------------- inline functions -------------------------------------


#endif/* _MOTORINFO_H_ */
//-----------------------End of file------------------------------------------
/** @}*/











