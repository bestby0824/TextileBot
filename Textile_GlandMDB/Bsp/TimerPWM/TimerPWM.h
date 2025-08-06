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
#ifndef TIMER_PWM_H_
#define TIMER_PWM_H_

//-------------------- include files ----------------------------------------
#include "stm32f4xx_hal.h"
#include "IQmath.h"
//-------------------- public definitions -----------------------------------
#define PWM_DEAD_TIME_NS       1000           //死区时间(ns)
#define TIMER_PWM_CLK_FREQ     168000000

#define PWM_DT_COMP            _IQ(0.02)   //0.02 = 0.8/50
#define PWM_DT_Compensation(Duty)	if(Duty>_IQ(0.5)) Duty += PWM_DT_COMP;\
                                  else if(Duty<_IQ(0.5)) Duty -= PWM_DT_COMP;

#define ATIM_TIMX_CPLM                           TIM1
#define ATIM_TIMX_CPLM_CHY1                      TIM_CHANNEL_1
#define ATIM_TIMX_CPLM_CHY_CCRY1                 ATIM_TIMX_CPLM->CCR1
#define ATIM_TIMX_CPLM_CHY2                      TIM_CHANNEL_2
#define ATIM_TIMX_CPLM_CHY_CCRY2                 ATIM_TIMX_CPLM->CCR2
#define ATIM_TIMX_CPLM_CHY3                      TIM_CHANNEL_3
#define ATIM_TIMX_CPLM_CHY_CCRY3                 ATIM_TIMX_CPLM->CCR3
#define ATIM_TIMX_CPLM_CHY4                      TIM_CHANNEL_4
#define ATIM_TIMX_CPLM_CHY_CCRY4                 ATIM_TIMX_CPLM->CCR4
#define ATIM_TIMX_CPLM_CLK_ENABLE()             do{ __HAL_RCC_TIM1_CLK_ENABLE(); }while(0)

#define ATIM_TIMX_CPLM_CHY_GPIO_PORT             GPIOA
#define ATIM_TIMX_CPLM_CHY_GPIO_PIN1             GPIO_PIN_8
#define ATIM_TIMX_CPLM_CHY_GPIO_PIN2             GPIO_PIN_9
#define ATIM_TIMX_CPLM_CHY_GPIO_PIN3             GPIO_PIN_10
#define ATIM_TIMX_CPLM_CHY_GPIO_PIN4             GPIO_PIN_11
#define ATIM_TIMX_CPLM_CHY_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

#define ATIM_TIMX_CPLM_CHYN_GPIO_PORT            GPIOB
#define ATIM_TIMX_CPLM_CHYN_GPIO_PIN1            GPIO_PIN_13
#define ATIM_TIMX_CPLM_CHYN_GPIO_PIN2            GPIO_PIN_14
#define ATIM_TIMX_CPLM_CHYN_GPIO_PIN3            GPIO_PIN_15

#define ATIM_TIMX_CPLM_CHYN_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

#define ATIM_TIMX_CPLM_CHY_GPIO_AF             GPIO_AF1_TIM1


//-------------------- public data ------------------------------------------
typedef enum
{
    PWM_MODE_OFF,     //上下桥臂全关(不受计数器控制)
    PWM_MODE_STOP,    //上桥臂全关，下桥臂全开(不受计数器控制)
    PWM_MODE_BRAKE,   //上桥臂全关(不受计数器控制)，下桥臂PWM(受计数器控制)
    PWM_MODE_RUN,     //正常运行模式
} E_PWM_MODE;
typedef enum
{
    GPIO_OUT_MODE_PP,
    GPIO_OUT_MODE_AF,
    GPIO_OUT_MODE_H_PP_L_AF,

} E_GPIO_OUT_MODE;
//-------------------- public functions -------------------------------------
void TimerPWM_Init( uint32_t freq );
void TimerPWM_DutySet( _iq dutyA, _iq dutyB, _iq dutyC );
void TimerPWM_ModeSet(E_PWM_MODE mode);
//-------------------- inline functions -------------------------------------

#endif /* TIMER_PWM_H_ */
//-----------------------End of file------------------------------------------
/** @}*/
