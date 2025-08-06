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
#ifndef _Timerbase__H_
#define _Timerbase__H_
//-------------------- include files ----------------------------------------
#include "Xint.h"
//-------------------- public definitions -----------------------------------
#define TIMER_BASE               TIM7
#define TIMER_BASE_CLK_EN()      __HAL_RCC_TIM7_CLK_ENABLE()
#define TIMER_BASE_CLK_FREQ      84000000
#define TIMER_BASE_IRQ           TIM7_IRQn
#define TIMER_BASE_IRQHandler    TIM7_IRQHandler

#define TIMER_FAST               TIM5
#define TIMER_FAST_CLK_EN()      __HAL_RCC_TIM5_CLK_ENABLE()
#define TIMER_FAST_CLK_FREQ      84000000
#define TIMER_FAST_IRQ           TIM5_IRQn
#define TIMER_FAST_IRQHandler    TIM5_IRQHandler
//-------------------- public data ------------------------------------------


//-------------------- public functions -------------------------------------
void TimerBase_Init ( uint32_t freq );
void TIMER_Fast_Init ( uint32_t freq );
uint32_t TimerBase_GetTicks_10KHz ( void );
uint32_t TimerBase_GetTicks_50KHz ( void );
//-------------------- inline functions -------------------------------------

#endif /* TIMER_BASE_H_ */

//-----------------------End of file------------------------------------------
/** @}*/
