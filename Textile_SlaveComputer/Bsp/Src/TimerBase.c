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

//-------------------- include files ----------------------------------------
#include "TimerBase.h"
#include "Dido.h"
#include "IIC_Sim.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------

//-------------------- private functions declare ----------------------------

//-------------------- public data ------------------------------------------
static uint32_t TimerBaseTick_10KHz = 0;
static uint32_t TimerBaseTick_50KHz = 0;
//-------------------- public functions -------------------------------------
/*! \fn				void TimerBase_Init(uint32_t freq)
 *  \brief 		Initializes the Timer
 *  \param 		freq
 *  \return 	none
 */
void TimerBase_Init ( uint32_t freq )
{
    TIMER_BASE_CLK_EN();

    TIMER_BASE->CR1 = 0x00 << TIM_CR1_ARPE_Pos |  //预装载禁止
                      0x00 << TIM_CR1_CMS_Pos  |  //计数方式，边沿对齐
                      0x00 << TIM_CR1_DIR_Pos  |  //计数方向，向上
                      0x00 << TIM_CR1_OPM_Pos  |  //单脉冲模式，禁止
                      0x00 << TIM_CR1_URS_Pos  |  //更新请求源
                      0x00 << TIM_CR1_UDIS_Pos |  //使能更新事件
                      0x00 << TIM_CR1_CEN_Pos  ;  //禁止计数器

    TIMER_BASE->CNT  = 0;
    TIMER_BASE->PSC  = 0;    //计数器不分频
    TIMER_BASE->ARR  = TIMER_BASE_CLK_FREQ / freq;
    TIMER_BASE->DIER = 0x01; //使能更新中断

    HAL_NVIC_SetPriority ( TIMER_BASE_IRQ, IRQ_Priority_TIM_BASE, 0 );
    HAL_NVIC_EnableIRQ ( TIMER_BASE_IRQ );

    TIMER_BASE->CR1 |= 0x01; //使能计数器
}

/*! \fn				uint32_t TimerBase_GetTicks_10KHz(void)
 *  \brief 		get ticks of timer
 *  \param 		none
 *  \return 	the ticks of timer
 */
uint32_t TimerBase_GetTicks_10KHz ( void )
{
    return TimerBaseTick_10KHz;
}
uint32_t TimerBase_GetTicks_50KHz ( void )
{
    return TimerBaseTick_50KHz;
}
/**
  * @brief  This function handles TIMER_BASE Handler.(10KHZ)
  * @param  None
  * @retval None
  */
void TIMER_BASE_IRQHandler ( void )
{
    NVIC_ClearPendingIRQ ( TIMER_BASE_IRQ );

    if ( TIMER_BASE->SR & 0x01 )
    {
        TimerBaseTick_10KHz ++;
        TimerBase_Update_Callback();
    }

    TIMER_BASE->SR = 0;
}




void TIMER_Fast_Init ( uint32_t freq )
{
    TIMER_FAST_CLK_EN();

    TIMER_FAST->CR1 = 0x00 << TIM_CR1_ARPE_Pos |  //预装载禁止
                      0x00 << TIM_CR1_CMS_Pos  |  //计数方式，边沿对齐
                      0x00 << TIM_CR1_DIR_Pos  |  //计数方向，向上
                      0x00 << TIM_CR1_OPM_Pos  |  //单脉冲模式，禁止
                      0x00 << TIM_CR1_URS_Pos  |  //更新请求源
                      0x00 << TIM_CR1_UDIS_Pos |  //使能更新事件
                      0x00 << TIM_CR1_CEN_Pos  ;  //禁止计数器

    TIMER_FAST->CNT  = 0;
    TIMER_FAST->PSC  = 0;    //计数器不分频
    TIMER_FAST->ARR  = TIMER_FAST_CLK_FREQ / freq;
    TIMER_FAST->DIER = 0x01; //使能更新中断

    HAL_NVIC_SetPriority ( TIMER_FAST_IRQ, IRQ_Priority_TIM_FAST, 1 );
    HAL_NVIC_EnableIRQ ( TIMER_FAST_IRQ );

    TIMER_FAST->CR1 |= 0x01; //使能计数器
}


/**
  * @brief  This function handles TIMER_FAST Handler.(10KHZ)
  * @param  None
  * @retval None
  */
void TIMER_FAST_IRQHandler ( void )
{
    NVIC_ClearPendingIRQ ( TIMER_FAST_IRQ );

    if ( TIMER_FAST->SR & 0x01 )
    {
        TimerBaseTick_50KHz ++;
        if ( IIC_EN_Flg[IIC1] ) IIC1_IRQHandler ( );
    }

    TIMER_FAST->SR = 0;
}


//-------------------- private functions ------------------------------------

//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


