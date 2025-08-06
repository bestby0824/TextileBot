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
#include "TimerPWM.h"
#include "MotorInfo.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
TIM_HandleTypeDef Tim1Handle;                             /* 定时器x句柄 */
TIM_BreakDeadTimeConfigTypeDef sConfigBK;                 /* 死区时间设置 */

//-------------------- private functions declare ----------------------------
static uint16_t GetDtgByDeadTimeNS(uint16_t deadtime_ns);
static void SetGpioOutMode(E_GPIO_OUT_MODE mode);

//-------------------- public data ------------------------------------------
E_CURRENT_MODE eCurrentMode = CURRENT_MODE_AB;

//-------------------- public functions -------------------------------------
/*! \fn				void TimerPWM_Init(freq)
 *  \brief 		Initializes the Timer
 *  \param 		freq
 *  \return 	none
 */
void TimerPWM_Init( uint32_t freq )
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_OC_InitTypeDef sConfig;

    ATIM_TIMX_CPLM_CLK_ENABLE();            /* TIMx 时钟使能 */
    ATIM_TIMX_CPLM_CHY_GPIO_CLK_ENABLE();   /* 通道X对应IO口时钟使能 */
    ATIM_TIMX_CPLM_CHYN_GPIO_CLK_ENABLE();  /* 通道X互补通道对应IO口时钟使能 */

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;  //设置成推挽输出
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = ATIM_TIMX_CPLM_CHY_GPIO_AF;

    GPIO_InitStruct.Pin = ATIM_TIMX_CPLM_CHY_GPIO_PIN1|ATIM_TIMX_CPLM_CHY_GPIO_PIN2|ATIM_TIMX_CPLM_CHY_GPIO_PIN3|ATIM_TIMX_CPLM_CHY_GPIO_PIN4;
    HAL_GPIO_Init(ATIM_TIMX_CPLM_CHY_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ATIM_TIMX_CPLM_CHYN_GPIO_PIN1|ATIM_TIMX_CPLM_CHYN_GPIO_PIN2|ATIM_TIMX_CPLM_CHYN_GPIO_PIN3;
    HAL_GPIO_Init(ATIM_TIMX_CPLM_CHYN_GPIO_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(ATIM_TIMX_CPLM_CHY_GPIO_PORT,ATIM_TIMX_CPLM_CHY_GPIO_PIN1|ATIM_TIMX_CPLM_CHY_GPIO_PIN2|ATIM_TIMX_CPLM_CHY_GPIO_PIN3|ATIM_TIMX_CPLM_CHY_GPIO_PIN4,GPIO_PIN_RESET);//全部拉低
    HAL_GPIO_WritePin(ATIM_TIMX_CPLM_CHYN_GPIO_PORT,ATIM_TIMX_CPLM_CHYN_GPIO_PIN1|ATIM_TIMX_CPLM_CHYN_GPIO_PIN2|ATIM_TIMX_CPLM_CHYN_GPIO_PIN3,GPIO_PIN_RESET);
    
//    Tim1Handle.Instance = ATIM_TIMX_CPLM;                       /* 定时器x */
//    Tim1Handle.Init.Prescaler = 0;                              /* 预分频系数 */
//    Tim1Handle.Init.CounterMode = TIM_CR1_CMS_0;                /* Up-Down模式，中间对称 */
//    Tim1Handle.Init.Period = TIMER_PWM_CLK_FREQ/2/freq - 1;                              /* 自动重装载值 */
//    Tim1Handle.Init.ClockDivision = 0;
//    Tim1Handle.Init.RepetitionCounter = 0;
//    Tim1Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  /* 使能影子寄存器TIMx_ARR */
//  
//    HAL_TIM_PWM_Init(&Tim1Handle) ;

//    sConfig.OCMode = TIM_OCMODE_PWM1;                               /* PWM模式1 */
//    sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;                       /* OCy 高电平有效 */
//    sConfig.OCNPolarity = TIM_OCPOLARITY_LOW;                      /* OCyN 高电平有效 */
//    sConfig.OCIdleState = TIM_OCIDLESTATE_SET;                      /* 当MOE=0，OCx=0 */
//    sConfig.OCNIdleState = TIM_OCNIDLESTATE_SET;                    /* 当MOE=0，OCxN=0 */
////    sConfig.Pulse = 4199;                                           /*   设置占空比50%*/
//    HAL_TIM_PWM_ConfigChannel(&Tim1Handle, &sConfig, ATIM_TIMX_CPLM_CHY1);
//    HAL_TIM_PWM_ConfigChannel(&Tim1Handle, &sConfig, ATIM_TIMX_CPLM_CHY2);
//    HAL_TIM_PWM_ConfigChannel(&Tim1Handle, &sConfig, ATIM_TIMX_CPLM_CHY3);
//    sConfig.OCPolarity = TIM_OCPOLARITY_LOW;                       /* OCy 高电平有效 */
//    sConfig.OCNPolarity = TIM_OCPOLARITY_LOW;                      /* OCyN 高电平有效 */
//    sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;                      /* 当MOE=0，OCx=0 */
//    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;                    /* 当MOE=0，OCxN=0 */
//    HAL_TIM_PWM_ConfigChannel(&Tim1Handle, &sConfig, ATIM_TIMX_CPLM_CHY4);

//    /* 设置死区参数，开启死区中断 */
//    sConfigBK.OffStateRunMode = TIM_OSSR_ENABLE;           /* 运行模式的关闭输出状态 */
//    sConfigBK.OffStateIDLEMode = TIM_OSSI_ENABLE;          /* 空闲模式的关闭输出状态 */
//    sConfigBK.LockLevel = TIM_LOCKLEVEL_OFF;                /* 不用寄存器锁功能 */
//    sConfigBK.BreakState = TIM_BREAK_ENABLE;                /* 使能刹车输入 */
//    sConfigBK.BreakPolarity = TIM_BREAKPOLARITY_HIGH;       /* 刹车输入有效信号极性为高 */
//    sConfigBK.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE; /* 使能AOE位，允许刹车结束后自动恢复输出 */
//    sConfigBK.DeadTime = GetDtgByDeadTimeNS(PWM_DEAD_TIME_NS);
//    HAL_TIMEx_ConfigBreakDeadTime(&Tim1Handle, &sConfigBK);

//    HAL_TIM_PWM_Start(&Tim1Handle, ATIM_TIMX_CPLM_CHY1);         /* OCy 输出使能 */
//    HAL_TIMEx_PWMN_Start(&Tim1Handle, ATIM_TIMX_CPLM_CHY1);      /* OCyN 输出使能 */
//    HAL_TIM_PWM_Start(&Tim1Handle, ATIM_TIMX_CPLM_CHY2);         /* OCy 输出使能 */
//    HAL_TIMEx_PWMN_Start(&Tim1Handle, ATIM_TIMX_CPLM_CHY2);      /* OCyN 输出使能 */
//    HAL_TIM_PWM_Start(&Tim1Handle, ATIM_TIMX_CPLM_CHY3);         /* OCy 输出使能 */
//    HAL_TIMEx_PWMN_Start(&Tim1Handle, ATIM_TIMX_CPLM_CHY3);      /* OCyN 输出使能 */
//    

//    ATIM_TIMX_CPLM_CHY_CCRY1 = 0;                             /* 设置比较寄存器 */
//    ATIM_TIMX_CPLM_CHY_CCRY2 = 0;                             /* 设置比较寄存器 */
//    ATIM_TIMX_CPLM_CHY_CCRY3 = 0;                             /* 设置比较寄存器 */
//    ATIM_TIMX_CPLM_CHY_CCRY4 = Tim1Handle.Init.Period;        /* 设置比较寄存器 */

    ATIM_TIMX_CPLM->CR1 = (0x0 << TIM_CR1_CKD_Pos)    |     //死区和输入采样时钟不分频，与计数器频率一致
                          (0x1 << TIM_CR1_ARPE_Pos)   |	   //预装载缓冲使能
                          (0x1 << TIM_CR1_CMS_Pos)    |     //计数模式，中心对齐模式1
                          (0x0 << TIM_CR1_DIR_Pos)    |     //计数方向，向上
                          (0x0 << TIM_CR1_OPM_Pos)	   |	   //单脉冲模式禁用，发生更新事件时不停止计数
                          (0x1 << TIM_CR1_URS_Pos)    |     //只有计数器上溢/下溢会生成更新事件(UEV)
                          (0x0 << TIM_CR1_UDIS_Pos)   |     //使能更新事件
                          (0x0 << TIM_CR1_CEN_Pos)    ;     //禁止计数器
                          
    ATIM_TIMX_CPLM->CR2 = 0x0 << TIM_CR2_CCPC_Pos   |      //各通道输出使能和模式选择位预装载关闭
                          0x0 << TIM_CR2_CCUS_Pos   |      //如果CCPC=1，只能通过COM位更新上述预装载更新
                          0x0 << TIM_CR2_CCDS_Pos   |      //通道的DMA触发，未使用
                          0x0 << TIM_CR2_MMS_Pos    |      //主模式输出信号选择，未使用
                          0x0 << TIM_CR2_TI1S_Pos   |      //输入输出异或功能关闭
                          0x0 << TIM_CR2_OIS1_Pos   |      //空闲状态(MOE=0)下,通道输出使能(CC1E=1)时,OC1=0
                          0x0 << TIM_CR2_OIS1N_Pos  |      //空闲状态(MOE=0)下,通道输出使能(CC1NE=1)时,OC1N=1
                          0x0 << TIM_CR2_OIS2_Pos   |      //空闲状态(MOE=0)下,通道输出使能(CC2E=1)时,OC2=0
                          0x0 << TIM_CR2_OIS2N_Pos  |      //空闲状态(MOE=0)下,通道输出使能(CC2NE=1)时,OC2N=1
                          0x0 << TIM_CR2_OIS3_Pos   |      //空闲状态(MOE=0)下,通道输出使能(CC3E=1)时,OC3=0
                          0x0 << TIM_CR2_OIS3N_Pos  ;      //空闲状态(MOE=0)下,通道输出使能(CC3NE=1)时,OC3N=1
                          
    ATIM_TIMX_CPLM->SMCR  = 0x0000;						    //从模式控制寄存器，禁止从模式

    ATIM_TIMX_CPLM->DIER  = 0x0000;						    //DMA和中断控制，禁止所有中断

    ATIM_TIMX_CPLM->PSC   = 0;                     //计数器预分频1，168M/1

    ATIM_TIMX_CPLM->ARR   = TIMER_PWM_CLK_FREQ/2/freq; //自动装载值

    ATIM_TIMX_CPLM->RCR   = 1; //重复计数器，2次溢出触发一次更新事件，中心对齐模式下则为一个PWM周期(下溢时)触发一次更新事件


    ATIM_TIMX_CPLM->CCER = 0x1 << TIM_CCER_CC1E_Pos    |    //使能CC1输出
                           0x0 << TIM_CCER_CC1P_Pos    |    //CC1输出极性，高电平有效
                           0x1 << TIM_CCER_CC1NE_Pos   |    //使能CCN1输出
                           0x1 << TIM_CCER_CC1NP_Pos   |    //CCN1输出极性，高电平有效
                           0x1 << TIM_CCER_CC2E_Pos    |    //使能CC2输出
                           0x0 << TIM_CCER_CC2P_Pos    |    //CC2输出极性，高电平有效
                           0x1 << TIM_CCER_CC2NE_Pos   |    //使能CCN2输出
                           0x1 << TIM_CCER_CC2NP_Pos   |    //CCN2输出极性，高电平有效
                           0x1 << TIM_CCER_CC3E_Pos    |    //使能CC3输出
                           0x0 << TIM_CCER_CC3P_Pos    |    //CC3输出极性，高电平有效
                           0x1 << TIM_CCER_CC3NE_Pos   |    //使能CCN3输出
                           0x1 << TIM_CCER_CC3NP_Pos   |    //CCN4输出极性，高电平有效
                           0x1 << TIM_CCER_CC4E_Pos    |    //CC4输出，使能
                           0x0 << TIM_CCER_CC4P_Pos    ;    //CC4输出极性，高电平有效

    ATIM_TIMX_CPLM->BDTR = GetDtgByDeadTimeNS(PWM_DEAD_TIME_NS)      |    //死区时间
                           0x0 << TIM_BDTR_LOCK_Pos    |    //配置锁定关闭
                           0x1 << TIM_BDTR_OSSI_Pos    |    //空闲状态(MOE=0)下,CC1E=0时有效;1:输出无效电平(CC1P的反状态),0:不由TIM控制;
                           0x1 << TIM_BDTR_OSSR_Pos    |    //输出状态(MOE=1)下,CC1E=0时有效;1:输出无效电平(CC1P的反状态),0:不由TIM控制;
                           0x0 << TIM_BDTR_BKE_Pos     |    //刹车功能禁止
                           0x0 << TIM_BDTR_BKP_Pos     |    //刹车输入极性
                           0x0 << TIM_BDTR_AOE_Pos     |    //自动输出禁止,MOE只能由软件置1,否则可在发生下一更新事件时自动置1
                           0x1 << TIM_BDTR_MOE_Pos     ;    //主输出使能

    ATIM_TIMX_CPLM->CCMR1 = 0x0 << TIM_CCMR1_CC1S_Pos   |    //CC1通道配置为输出
                       0x0 << TIM_CCMR1_OC1FE_Pos  |    //CC1快速输出比较禁止
                       0x1 << TIM_CCMR1_OC1PE_Pos  |    //CC1比较寄存器缓存使能，当发生更新事件时装载
                       0x6 << TIM_CCMR1_OC1M_Pos   |    //CC1输出模式PWM1
                       0x0 << TIM_CCMR1_OC1CE_Pos  |    //输出比较清零禁止,OC1Ref不受ETRF(从模式下有效)输入影响
                       0x0 << TIM_CCMR1_CC2S_Pos   |    //CC2通道配置为输出
                       0x0 << TIM_CCMR1_OC2FE_Pos  |    //CC2快速输出比较禁止
                       0x1 << TIM_CCMR1_OC2PE_Pos  |    //CC2比较寄存器缓存使能，当发生更新事件时装载
                       0x6 << TIM_CCMR1_OC2M_Pos   |    ///CC2输出模式PWM1
                       0x0 << TIM_CCMR1_OC2CE_Pos  ;    //输出比较清零禁止,OC2Ref不受ETRF(从模式下有效)输入影响

    ATIM_TIMX_CPLM->CCMR2 = 0x0 << TIM_CCMR2_CC3S_Pos   |    //CC3通道配置为输出
                       0x0 << TIM_CCMR2_OC3FE_Pos  |    //CC3快速输出比较禁止
                       0x1 << TIM_CCMR2_OC3PE_Pos  |    //CC3比较寄存器缓存使能，当发生更新事件时装载
                       0x6 << TIM_CCMR2_OC3M_Pos   |    //CC3输出模式PWM1
                       0x0 << TIM_CCMR2_OC3CE_Pos  |    //输出比较清零禁止,OC3Ref不受ETRF(从模式下有效)输入影响
                       0x0 << TIM_CCMR2_CC4S_Pos   |    //CC4通道配置为输出
                       0x0 << TIM_CCMR2_OC4FE_Pos  |    //CC4快速输出比较禁止
                       0x1 << TIM_CCMR2_OC4PE_Pos  |    //CC4比较寄存器缓存使能，当发生更新事件时装载
                       0x7 << TIM_CCMR2_OC4M_Pos   |    //CC4输出模式PWM1
                       0x0 << TIM_CCMR2_OC4CE_Pos  ;    //输出比较清零禁止,OC4Ref不受ETRF(从模式下有效)输入影响

    ATIM_TIMX_CPLM->CCR1 = 0;
    ATIM_TIMX_CPLM->CCR2 = 0;
    ATIM_TIMX_CPLM->CCR3 = 0;
    ATIM_TIMX_CPLM->CCR4 = ATIM_TIMX_CPLM->ARR - 1;

    ATIM_TIMX_CPLM->CR1 |= 0x01;  //使能计数器
}
void TimerPWM_DutySet( _iq dutyA, _iq dutyB, _iq dutyC )
{
    dutyA += _IQ(0.5);
    dutyB += _IQ(0.5);
    dutyC += _IQ(0.5);
    
//    PWM_DT_Compensation(dutyA);
//    PWM_DT_Compensation(dutyB);
//    PWM_DT_Compensation(dutyC);

    dutyA = _IQsat(dutyA,_IQ(1),_IQ(0));
    dutyB = _IQsat(dutyB,_IQ(1),_IQ(0));
    dutyC = _IQsat(dutyC,_IQ(1),_IQ(0));
    
//    if ((dutyA>=dutyB)&&(dutyA>=dutyC))
//    {
//        eCurrentMode = CURRENT_MODE_BC;
//    }
//    else if ((dutyB>=dutyA)&&(dutyB>=dutyC))
//    {
//        eCurrentMode = CURRENT_MODE_AC;
//    }
//    else if ((dutyC>=dutyA)&&(dutyC>=dutyB))
//    {
//        eCurrentMode = CURRENT_MODE_AB;
//    }

    ATIM_TIMX_CPLM_CHY_CCRY3 = _IQsat ( _IQmpy ( dutyA, TIM1->ARR ), ATIM_TIMX_CPLM->ARR, 0 );
    ATIM_TIMX_CPLM_CHY_CCRY2 = _IQsat ( _IQmpy ( dutyB, TIM1->ARR ), ATIM_TIMX_CPLM->ARR, 0 );
    ATIM_TIMX_CPLM_CHY_CCRY1 = _IQsat ( _IQmpy ( dutyC, TIM1->ARR ), ATIM_TIMX_CPLM->ARR, 0 );
}
/*! \fn				void TimerPWM_ModeSet(E_PWM_MODE mode)
 *  \brief 		set pwm mode
 *  \param 		mode
 *  \return 	none
 */
void TimerPWM_ModeSet(E_PWM_MODE mode)
{
    switch(mode)
    {
        case PWM_MODE_OFF:
        {
            SetGpioOutMode(GPIO_OUT_MODE_PP);  //GPIO切换为推挽输出

            HAL_GPIO_WritePin(ATIM_TIMX_CPLM_CHY_GPIO_PORT,ATIM_TIMX_CPLM_CHY_GPIO_PIN1|ATIM_TIMX_CPLM_CHY_GPIO_PIN2|ATIM_TIMX_CPLM_CHY_GPIO_PIN3,GPIO_PIN_RESET);//GPIO输出全部拉低
            HAL_GPIO_WritePin(ATIM_TIMX_CPLM_CHYN_GPIO_PORT,ATIM_TIMX_CPLM_CHYN_GPIO_PIN1|ATIM_TIMX_CPLM_CHYN_GPIO_PIN2|ATIM_TIMX_CPLM_CHYN_GPIO_PIN3,GPIO_PIN_SET);
        }
        break;

        case PWM_MODE_STOP:
        {
            SetGpioOutMode(GPIO_OUT_MODE_PP);  //GPIO切换为推挽输出

            HAL_GPIO_WritePin(ATIM_TIMX_CPLM_CHY_GPIO_PORT,ATIM_TIMX_CPLM_CHY_GPIO_PIN1|ATIM_TIMX_CPLM_CHY_GPIO_PIN2|ATIM_TIMX_CPLM_CHY_GPIO_PIN3,GPIO_PIN_RESET);//上桥臂全部拉低
            HAL_GPIO_WritePin(ATIM_TIMX_CPLM_CHYN_GPIO_PORT,ATIM_TIMX_CPLM_CHYN_GPIO_PIN1|ATIM_TIMX_CPLM_CHYN_GPIO_PIN2|ATIM_TIMX_CPLM_CHYN_GPIO_PIN3,GPIO_PIN_RESET);//下桥臂全部拉高
        }
        break;

        case PWM_MODE_BRAKE:
        {
            SetGpioOutMode(GPIO_OUT_MODE_H_PP_L_AF);  //GPIO切换为上桥臂推挽下桥臂复用（PWM）

            HAL_GPIO_WritePin(ATIM_TIMX_CPLM_CHY_GPIO_PORT,ATIM_TIMX_CPLM_CHY_GPIO_PIN1|ATIM_TIMX_CPLM_CHY_GPIO_PIN2|ATIM_TIMX_CPLM_CHY_GPIO_PIN3,GPIO_PIN_RESET);//上桥臂全部拉低
        }
        break;

        case PWM_MODE_RUN:
        {
            SetGpioOutMode(GPIO_OUT_MODE_AF);  //GPIO切换为复用输出
        }
        break;

        default:
        {

        } break;
    }
}
//-------------------- private functions ------------------------------------
static void SetGpioOutMode(E_GPIO_OUT_MODE mode)
{
    if(GPIO_OUT_MODE_PP == mode)
    {
        ATIM_TIMX_CPLM_CHY_GPIO_PORT->MODER &= ~0x003F0000; //GPIO切换为推挽输出
        ATIM_TIMX_CPLM_CHYN_GPIO_PORT->MODER &= ~0xFC000000;
        ATIM_TIMX_CPLM_CHY_GPIO_PORT->MODER |= 0x00150000;
        ATIM_TIMX_CPLM_CHYN_GPIO_PORT->MODER |= 0x54000000;
    }
    else if(GPIO_OUT_MODE_AF == mode)
    {
        ATIM_TIMX_CPLM_CHY_GPIO_PORT->MODER &= ~0x003F0000; //GPIO切换为复用输出
        ATIM_TIMX_CPLM_CHYN_GPIO_PORT->MODER &= ~0xFC000000;
        ATIM_TIMX_CPLM_CHY_GPIO_PORT->MODER |= 0x002A0000;
        ATIM_TIMX_CPLM_CHYN_GPIO_PORT->MODER |= 0xA8000000;
    } else if(GPIO_OUT_MODE_H_PP_L_AF == mode)     //上桥臂推挽下桥臂PWM
    {
        ATIM_TIMX_CPLM_CHY_GPIO_PORT->MODER &= ~0x003F0000; //GPIO切换为复用输出
        ATIM_TIMX_CPLM_CHYN_GPIO_PORT->MODER &= ~0xFC000000;
        ATIM_TIMX_CPLM_CHY_GPIO_PORT->MODER |= 0x00150000;
        ATIM_TIMX_CPLM_CHYN_GPIO_PORT->MODER |= 0xA8000000;
    }
}
/**
  * @brief  Configure the DeadTime
  * @param  deadtime_ns
  * @retval u16DTG
  * @notes  None
  */
static uint16_t GetDtgByDeadTimeNS(uint16_t deadtime_ns)
{
    uint16_t temp;
    uint16_t u16DTG;

    temp = (uint16_t)((uint64_t)deadtime_ns * (uint64_t)TIMER_PWM_CLK_FREQ / 1000000000);

    if(temp < 0x80)
    {
        u16DTG = temp;
    }
    else if(temp < 0x80*2)
    {
        u16DTG = 0x80 | ((temp-0x80)/2);
    }
    else if(temp < 0x80*4)
    {
        u16DTG = 0xC0 | ((temp-0x100)/8);
    }
    else if(temp < 0x80*8)
    {
        u16DTG = 0xE0 | ((temp-0x200)/16);
    }
    else
    {
        u16DTG = 0xFF;
    }

    return u16DTG;
}

//-----------------------End of file------------------------------------------
/** @} */ /* End of group */
