/**
* 版权所有(C) 
*
* ********
*
* @file 
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
//------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
#ifndef _INT_CTRL__H_
#define _INT_CTRL__H_

//-------------------- include files ----------------------------------------
#include "stm32f4xx_hal.h"
#include "AngleMath.h"

//-------------------- public definitions -----------------------------------
#define TIM_BASE1_FREQ            10000           //10KHZ
#define TIM_BASE2_FREQ            1000            //1KHZ
#define TIM_BASE_FREQ             20000           //20KHZ
#define TIM_PWM_FREQ              20000           //20KHZ

#define IRQ_Priority_Reangle_CODER 0    //转向电机编码器
#define IRQ_Priority_TAMA_CODER    0    //后轮编码器
#define IRQ_Priority_CAN           1    //CAN接收中断
#define IRQ_Priority_TIM_PWM       1
#define IRQ_Priority_TIM_BASE1     2
#define IRQ_Priority_TIM_BASE2     3
#define IRQ_Priority_TIM_BASE      4
#define IRQ_Priority_ADC_DMA       5    //ADC_DMA中断
#define IRQ_Priority_485           6
#define IRQ_Priority_UART          7    //调试串口

//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
void ADC_CurrentSampleCpl_Callback ( void );

#endif // _INT_CTRL__H_

//-----------------------End of file------------------------------------------
/** @}*/
