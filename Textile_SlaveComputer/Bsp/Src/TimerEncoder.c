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
#include "TimerEncoder.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------

//-------------------- private functions declare ----------------------------

//-------------------- public data ------------------------------------------
TIM_HandleTypeDef ENC_TimHandle1, ENC_TimHandle2;
//-------------------- public functions -------------------------------------
void TimerENC1_Init ( uint32_t Lines )
{

    TIM_Encoder_InitTypeDef InitCfigENC;
    TIM_MasterConfigTypeDef sMasterConfig;

    /***************************************GPIO config****************************************/
    GPIO_InitTypeDef   GPIO_InitStruct;

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = TIMER_ENC1_GPIO_AF;

    GPIO_InitStruct.Pin = TIMER_ENC1_A_GPIO_PIN;
    HAL_GPIO_Init ( TIMER_ENC1_A_GPIO_PORT, &GPIO_InitStruct );
    GPIO_InitStruct.Pin = TIMER_ENC1_B_GPIO_PIN;
    HAL_GPIO_Init ( TIMER_ENC1_B_GPIO_PORT, &GPIO_InitStruct );

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    HAL_GPIO_Init ( GPIOE, &GPIO_InitStruct );
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    HAL_GPIO_Init ( GPIOE, &GPIO_InitStruct );

    /**************************************TIMER config***************************************/
    TIMER_ENC1_CLK_EN();

    ENC_TimHandle1.Instance = TIMER_ENC1;
    ENC_TimHandle1.Init.Prescaler = 0;
    ENC_TimHandle1.Init.CounterMode = TIM_COUNTERMODE_UP;
    ENC_TimHandle1.Init.Period = Lines;
    ENC_TimHandle1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    ENC_TimHandle1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    InitCfigENC.EncoderMode = TIM_ENCODERMODE_TI12;//4倍频，1024*4=4096
    InitCfigENC.IC1Polarity = TIM_ICPOLARITY_RISING;
    InitCfigENC.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    InitCfigENC.IC1Prescaler = TIM_ICPSC_DIV1;
    InitCfigENC.IC1Filter = 0;
    InitCfigENC.IC2Polarity = TIM_ICPOLARITY_RISING;
    InitCfigENC.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    InitCfigENC.IC2Prescaler = TIM_ICPSC_DIV1;
    InitCfigENC.IC2Filter = 0;

    HAL_TIM_Encoder_Init ( &ENC_TimHandle1, &InitCfigENC );

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization ( &ENC_TimHandle1, &sMasterConfig );

    HAL_TIM_Encoder_Start ( &ENC_TimHandle1, TIM_CHANNEL_ALL );
}
void TimerENC2_Init ( uint32_t Lines )
{

    TIM_Encoder_InitTypeDef InitCfigENC;
    TIM_MasterConfigTypeDef sMasterConfig;

    /***************************************GPIO config****************************************/
    GPIO_InitTypeDef   GPIO_InitStruct;

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = TIMER_ENC2_GPIO_AF;

    GPIO_InitStruct.Pin = TIMER_ENC2_A_GPIO_PIN;
    HAL_GPIO_Init ( TIMER_ENC2_A_GPIO_PORT, &GPIO_InitStruct );
    GPIO_InitStruct.Pin = TIMER_ENC2_B_GPIO_PIN;
    HAL_GPIO_Init ( TIMER_ENC2_B_GPIO_PORT, &GPIO_InitStruct );

    /**************************************TIMER config***************************************/
    TIMER_ENC2_CLK_EN();

    ENC_TimHandle2.Instance = TIMER_ENC2;
    ENC_TimHandle2.Init.Prescaler = 0;
    ENC_TimHandle2.Init.CounterMode = TIM_COUNTERMODE_UP;
    ENC_TimHandle2.Init.Period = Lines;
    ENC_TimHandle2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    ENC_TimHandle2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;


    InitCfigENC.EncoderMode = TIM_ENCODERMODE_TI12;//4倍频，1024*4=4096
    InitCfigENC.IC1Polarity = TIM_ICPOLARITY_RISING;
    InitCfigENC.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    InitCfigENC.IC1Prescaler = TIM_ICPSC_DIV1;
    InitCfigENC.IC1Filter = 0;
    InitCfigENC.IC2Polarity = TIM_ICPOLARITY_RISING;
    InitCfigENC.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    InitCfigENC.IC2Prescaler = TIM_ICPSC_DIV1;
    InitCfigENC.IC2Filter = 0;

    HAL_TIM_Encoder_Init ( &ENC_TimHandle2, &InitCfigENC );

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization ( &ENC_TimHandle2, &sMasterConfig );

    HAL_TIM_Encoder_Start ( &ENC_TimHandle2, TIM_CHANNEL_ALL );

}
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


