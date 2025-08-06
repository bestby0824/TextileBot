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
#ifndef _TimerEncoder__H_
#define _TimerEncoder__H_
//-------------------- include files ----------------------------------------
#include "Driver.h"
#include "AngleMath.h"
//-------------------- public definitions -----------------------------------
#define TIMER_ENC1               TIM1
#define TIMER_ENC1_CLK_EN()      __HAL_RCC_TIM1_CLK_ENABLE()
#define TIMER_ENC1_CLK_FREQ      168000000

#define TIMER_ENC1_GPIO_AF       GPIO_AF1_TIM1

#define TIMER_ENC1_A_GPIO_PIN         GPIO_PIN_9
#define TIMER_ENC1_B_GPIO_PIN         GPIO_PIN_11
//#define TIMER_ENC1_Z_GPIO_PIN         GPIO_PIN_8

#define TIMER_ENC1_A_GPIO_PORT         GPIOE
#define TIMER_ENC1_B_GPIO_PORT         GPIOE
//#define TIMER_ENC1_Z_GPIO_PIN         GPIO_PIN_8

#define TIMER_ENC2               TIM3
#define TIMER_ENC2_CLK_EN()      __HAL_RCC_TIM3_CLK_ENABLE()
#define TIMER_ENC2_CLK_FREQ      168000000

#define TIMER_ENC2_GPIO_AF       GPIO_AF2_TIM3

#define TIMER_ENC2_A_GPIO_PIN         GPIO_PIN_6
#define TIMER_ENC2_B_GPIO_PIN         GPIO_PIN_7
//#define TIMER_ENC2_Z_GPIO_PIN         GPIO_PIN_8

#define TIMER_ENC2_A_GPIO_PORT         GPIOA
#define TIMER_ENC2_B_GPIO_PORT         GPIOA
//#define TIMER_ENC2_Z_GPIO_PIN         GPIO_PIN_8


typedef struct {
    int16_t ENC_New;
    int16_t ENC_Old;
    int16_t Diff;
    _iq _iqENC_AngleAdd;
    _iq _iqENC_Angle;
    uint32_t u32EncUpdateCnt;
    uint8_t ENC_EN;
} ENC_Handler;

#define ENC_Handler_Default  {     \
	0,/*int16_t ENC_New;*/ \
	0,/*int16_t ENC_Old;*/ \
	0,/*int16_t Diff;*/ \
	0,/*_iq _iqENC_AngleAdd;*/\
	0,/*_iq _iqENC_Angle;*/ \
  0,/*u32EncUpdateCnt;*/\
	0/*uint8_t ENC_EN;*/ \
}

#define ENCLines        7149
#define ENCLinesHalf        ENCLines >> 1
//-------------------- public data ------------------------------------------
extern TIM_HandleTypeDef ENC_TimHandle1,ENC_TimHandle2;
//-------------------- public functions -------------------------------------
void TimerENC1_Init(uint32_t Lines);
void TimerENC2_Init(uint32_t Lines);

#endif // _XXX_H_

//-----------------------End of file------------------------------------------
/** @}*/
