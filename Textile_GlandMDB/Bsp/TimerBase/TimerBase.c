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
//------------------------------------------------------------------------------

//-------------------- pragmas ----------------------------------------------

//-------------------- include files ----------------------------------------
#include "TimerBase.h"
#include "Int_Ctrl.h"
#include "ReangleRS485.h"
#include "MotorInfo.h"
//#include "CoderTama.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
static uint32_t u32TimerBase1Tick = 0, u32TimerBase2Tick = 0;
static uint32_t TIMER_PWM_SourceTick = 0, TIMER_HighFreq50KTick = 0;

//-------------------- private functions declare ----------------------------

//-------------------- public data ------------------------------------------
uint8_t u8Tick1kHz_Flag = 0;

//-------------------- public functions -------------------------------------
 /**
  * @brief  Initializes the Timer2
  * @param  freq
  * @retval None
  * @notes  None
  */
void TimerBase1_Init(uint32_t freq)
{
    TIMER_BASE1_CLK_EN();

    TIMER_BASE1->CR1 = 0x00 << TIM_CR1_ARPE_Pos |  //预装载禁止
                       0x00 << TIM_CR1_CMS_Pos  |  //计数方式，边沿对齐
                       0x00 << TIM_CR1_DIR_Pos  |  //计数方向，向上
                       0x00 << TIM_CR1_OPM_Pos  |  //单脉冲模式，禁止
                       0x00 << TIM_CR1_URS_Pos  |  //更新请求源
                       0x00 << TIM_CR1_UDIS_Pos |  //使能更新事件
                       0x00 << TIM_CR1_CEN_Pos  ;  //禁止计数器

    TIMER_BASE1->CNT  = 0;
    TIMER_BASE1->PSC  = 0;    //计数器不分频
    TIMER_BASE1->ARR  = TIMER_BASE1_CLK_FREQ/freq;
    TIMER_BASE1->DIER = 0x01; //使能更新中断

    HAL_NVIC_SetPriority(TIMER_BASE1_IRQ, IRQ_Priority_TIM_BASE1, 0);
    HAL_NVIC_EnableIRQ(TIMER_BASE1_IRQ);

    TIMER_BASE1->CR1 |= 0x01; //使能计数器
}

/**
 * @brief  get ticks of timer2
 * @param  none
 * @retval the ticks of timer2
 * @notes  None
 */
uint32_t TimerBase1_GetTicks(void)
{
    return u32TimerBase1Tick;
}

/**
  * @brief  This function handles TIMER_BASE1 Handler.(10KHZ)
  * @param  None
  * @retval None
  * @notes  None
  */
void TIMER_BASE1_IRQHandler(void)
{
    NVIC_ClearPendingIRQ(TIMER_BASE1_IRQ);

    if(TIMER_BASE1->SR&0x01)
    {
        u32TimerBase1Tick ++;

        TimerBase1_Update_Callback();
    }

    TIMER_BASE1->SR = 0;
}

 /**
  * @brief  Initializes the Timer5
  * @param  freq
  * @retval None
  * @notes  None
  */
void TimerBase2_Init(uint32_t freq)
{
    TIMER_BASE2_CLK_EN();

    TIMER_BASE2->CR1 = 0x00 << TIM_CR1_ARPE_Pos |  //预装载禁止
                      0x00 << TIM_CR1_CMS_Pos  |  //计数方式，边沿对齐
                      0x00 << TIM_CR1_DIR_Pos  |  //计数方向，向上
                      0x00 << TIM_CR1_OPM_Pos  |  //单脉冲模式，禁止
                      0x00 << TIM_CR1_URS_Pos  |  //更新请求源
                      0x00 << TIM_CR1_UDIS_Pos |  //使能更新事件
                      0x00 << TIM_CR1_CEN_Pos  ;  //禁止计数器

    TIMER_BASE2->CNT  = 0;
    TIMER_BASE2->PSC  = 0;    //计数器不分频
    TIMER_BASE2->ARR  = TIMER_BASE2_CLK_FREQ/freq;
    TIMER_BASE2->DIER = 0x01; //使能更新中断

    HAL_NVIC_SetPriority(TIMER_BASE2_IRQ, IRQ_Priority_TIM_BASE2, 0);
    HAL_NVIC_EnableIRQ(TIMER_BASE2_IRQ);

    TIMER_BASE2->CR1 |= 0x01; //使能计数器
}

/**
 * @brief  get ticks of timer5
 * @param  none
 * @retval the ticks of timer5
 * @notes  None
 */
uint32_t TimerBase2_GetTicks(void)
{
    return u32TimerBase2Tick;
}

/**
  * @brief  This function handles TIMER_BASE2 Handler.(10KHZ)
  * @param  None
  * @retval None
  * @notes  None
  */
void TIMER_BASE2_IRQHandler(void)
{
    NVIC_ClearPendingIRQ(TIMER_BASE2_IRQ);

    if(TIMER_BASE2->SR&0x01)
    {
        u32TimerBase2Tick ++;
        u8Tick1kHz_Flag = 1;
    }
    TIMER_BASE2->SR = 0;
}

//-------------------- public functions -------------------------------------
/*! \fn				void TIMER_PWM_Source_Init(uint32_t freq)
 *  \brief 		Initializes the Timer
 *  \param 		freq
 *  \return 	none
 */
void TIMER_PWM_Source_Init(uint32_t freq)
{
    TIMER_PWM_Source_CLK_EN();

    TIMER_PWM_Source->CR1 = 0x00 << TIM_CR1_ARPE_Pos |  //预装载禁止
                      0x00 << TIM_CR1_CMS_Pos  |  //计数方式，边沿对齐
                      0x00 << TIM_CR1_DIR_Pos  |  //计数方向，向上
                      0x00 << TIM_CR1_OPM_Pos  |  //单脉冲模式，禁止
                      0x00 << TIM_CR1_URS_Pos  |  //更新请求源
                      0x00 << TIM_CR1_UDIS_Pos |  //使能更新事件
                      0x00 << TIM_CR1_CEN_Pos  ;  //禁止计数器
    
    TIMER_PWM_Source->CR2= 0x2 << TIM_CR2_MMS_Pos;      //主模式输出信号选择，Update事件

    TIMER_PWM_Source->CNT  = 0;
    TIMER_PWM_Source->PSC  = 419;    //计数器不分频
    TIMER_PWM_Source->ARR  = TIMER_PWM_Source_CLK_FREQ/420/freq - 1;
    TIMER_PWM_Source->DIER = 0x01; //使能更新中断

    HAL_NVIC_SetPriority(TIMER_PWM_Source_IRQ, IRQ_Priority_TIM_BASE, 0);
    HAL_NVIC_EnableIRQ(TIMER_PWM_Source_IRQ);

    TIMER_PWM_Source->CR1 |= 0x01; //使能计数器
}


/**
  * @brief  This function handles TIMER_PWM_Source Handler.(20KHZ)
  * @param  None
  * @retval None
  */
void TIMER_PWM_Source_IRQHandler(void)
{
    NVIC_ClearPendingIRQ(TIMER_PWM_Source_IRQ);

    if(TIMER_PWM_Source->SR&0x01)
    {   
        TIMER_PWM_SourceTick ++;
        if ( TIMER_PWM_SourceTick % 5 == 0 )
        {
            ReangleRS485_Send_Process();
        }
//        ReangleRS485_Send_Process();
    }

    TIMER_PWM_Source->SR = 0;
}

//-------------------- private functions ------------------------------------

//-----------------------End of file------------------------------------------
/** @} */ /* End of group */

