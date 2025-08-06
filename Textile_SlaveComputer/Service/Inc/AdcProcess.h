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
#ifndef _AdcProcess__H_
#define _AdcProcess__H_
//-------------------- include files ----------------------------------------
#include "Driver.h"
#include "Adc.h"
#include "string.h"

#define  ADC_Sens_10A        _IQ(0.0030518)
#define  ADC_Sens_20A        _IQ(0.0015259)
#define  ADC_Sens_30A        _IQ(0.001022354)              

//-------------------- public definitions -----------------------------------

//-------------------- public data ------------------------------------------
typedef struct 
{
    int16_t ADC_Version; 
    int16_t ADC_SENSOR_24V_V;   //传感器电源电压检测
    int16_t ADC_TEMP_IN;        //温度检测
    int16_t ADC_PC_24V_V;       //上位机电源电压检测
    int16_t ADC_PC_24V_I;       //上位机电源电流检测
    int16_t ADC_2_5V_REF;       //2.5V参考
    int16_t ADC_M_I_MCU;        //电磁铁电流采样
    int16_t ADC_MOTOR1_24V_V;   //电机1电源电压检测
    int16_t ADC_MOTOR2_24V_V;   //电机2电源电压检测
    
    int16_t  ADC_CH_0_Stop_V;
    int16_t ADC_SENSOR_24V_I;   //传感器电源电流检测
    int16_t ADC_MOTOR1_24V_I;   //电机1电源电流检测
    int16_t ADC_MOTOR2_24V_I;   //电机2电源电流检测
}ADC_Value_Struct;     

//typedef enum {
//    ADCID_Version = 0,
//    ADCID_SENSOR_24V_V,    
//    ADCID_TEMP_IN,
//    ADCID_PC_24V_V,
//    ADCID_PC_24V_I,
//    ADCID_2_5V_REF,
//    ADCID_M_I_MCU,
//    ADCID_MOTOR1_24V_V,
//    ADCID_MOTOR2_24V_V,
//    ADCID_SENSOR_24V_I,
//    ADCID_MOTOR1_24V_I,
//    ADCID_VALVE_I,
//    ADCID_MOTOR2_24V_I,
//    ADCID_Num,
//} ADC_ValueID;

//-------------------- public functions -------------------------------------
int16_t Get_Temperature( void );
uint16_t Get_Version_AD ( void );
uint16_t Get_Motor_24V( void );
uint16_t Get_E_Stop_80V( void );
uint16_t Get_BAT_24V( void );
uint16_t Get_FORK_24V ( void );
int16_t Get_E_Stop_1A( void );
void ADC_Value();

extern ADC_Value_Struct ADC_Value_Num;

#endif // _AdcProcess__H_

//-----------------------End of file------------------------------------------
/** @}*/
