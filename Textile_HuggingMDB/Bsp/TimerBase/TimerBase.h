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

//------------------------------------------------------------------------------

#ifndef TIMER_BASE_H_
#define TIMER_BASE_H_
//-------------------- include files ----------------------------------------
#include "stm32f4xx_hal.h"

//-------------------- public definitions -----------------------------------
#define TIMER_BASE1               TIM2
#define TIMER_BASE1_CLK_EN()      __HAL_RCC_TIM2_CLK_ENABLE()
#define TIMER_BASE1_CLK_FREQ      84000000
#define TIMER_BASE1_IRQ           TIM2_IRQn
#define TIMER_BASE1_IRQHandler    TIM2_IRQHandler

#define TIMER_BASE2               TIM5
#define TIMER_BASE2_CLK_EN()      __HAL_RCC_TIM5_CLK_ENABLE()
#define TIMER_BASE2_CLK_FREQ      84000000
#define TIMER_BASE2_IRQ           TIM5_IRQn
#define TIMER_BASE2_IRQHandler    TIM5_IRQHandler

#define TIMER_PWM_Source               TIM3
#define TIMER_PWM_Source_CLK_EN()      __HAL_RCC_TIM3_CLK_ENABLE()
#define TIMER_PWM_Source_CLK_FREQ      84000000
#define TIMER_PWM_Source_IRQ           TIM3_IRQn
#define TIMER_PWM_Source_IRQHandler    TIM3_IRQHandler

#define TIMER_HighFreq50K               TIM4
#define TIMER_HighFreq50K_CLK_EN()      __HAL_RCC_TIM4_CLK_ENABLE()
#define TIMER_HighFreq50K_CLK_FREQ      84000000
#define TIMER_HighFreq50K_IRQ           TIM4_IRQn
#define TIMER_HighFreq50K_IRQHandler    TIM4_IRQHandler
//-------------------- public data -----------------------------------------
extern uint8_t u8Tick1kHz_Flag;

//-------------------- public functions -------------------------------------
void TimerBase1_Init(uint32_t freq);
void TimerBase2_Init(uint32_t freq);
void TIMER_PWM_Source_Init(uint32_t freq);

uint32_t TimerBase1_GetTicks(void);
uint32_t TimerBase2_GetTicks(void);
void TimerBase1_Update_Callback(void);

//-------------------- inline functions -------------------------------------

#endif /* TIMER_BASE_H_ */
//-----------------------End of file------------------------------------------
/** @}*/
