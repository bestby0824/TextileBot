/**
* ��Ȩ����(C)
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
* <����> | <�汾> | <����> | <����>
*
* xxxx/xx/xx | 1.0.0 | MuNiu | �����ļ�
*
*/
//------------------------------------------------------------------------------

//-------------------- pragmas ----------------------------------------------

//-------------------- include files ----------------------------------------
#include "TimerPWM.h"
#include "MotorInfo.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
TIM_HandleTypeDef Tim1Handle;                             /* ��ʱ��x��� */
TIM_BreakDeadTimeConfigTypeDef sConfigBK;                 /* ����ʱ������ */

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

    ATIM_TIMX_CPLM_CLK_ENABLE();            /* TIMx ʱ��ʹ�� */
    ATIM_TIMX_CPLM_CHY_GPIO_CLK_ENABLE();   /* ͨ��X��ӦIO��ʱ��ʹ�� */
    ATIM_TIMX_CPLM_CHYN_GPIO_CLK_ENABLE();  /* ͨ��X����ͨ����ӦIO��ʱ��ʹ�� */

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;  //���ó��������
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = ATIM_TIMX_CPLM_CHY_GPIO_AF;

    GPIO_InitStruct.Pin = ATIM_TIMX_CPLM_CHY_GPIO_PIN1|ATIM_TIMX_CPLM_CHY_GPIO_PIN2|ATIM_TIMX_CPLM_CHY_GPIO_PIN3|ATIM_TIMX_CPLM_CHY_GPIO_PIN4;
    HAL_GPIO_Init(ATIM_TIMX_CPLM_CHY_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ATIM_TIMX_CPLM_CHYN_GPIO_PIN1|ATIM_TIMX_CPLM_CHYN_GPIO_PIN2|ATIM_TIMX_CPLM_CHYN_GPIO_PIN3;
    HAL_GPIO_Init(ATIM_TIMX_CPLM_CHYN_GPIO_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(ATIM_TIMX_CPLM_CHY_GPIO_PORT,ATIM_TIMX_CPLM_CHY_GPIO_PIN1|ATIM_TIMX_CPLM_CHY_GPIO_PIN2|ATIM_TIMX_CPLM_CHY_GPIO_PIN3|ATIM_TIMX_CPLM_CHY_GPIO_PIN4,GPIO_PIN_RESET);//ȫ������
    HAL_GPIO_WritePin(ATIM_TIMX_CPLM_CHYN_GPIO_PORT,ATIM_TIMX_CPLM_CHYN_GPIO_PIN1|ATIM_TIMX_CPLM_CHYN_GPIO_PIN2|ATIM_TIMX_CPLM_CHYN_GPIO_PIN3,GPIO_PIN_RESET);
    
//    Tim1Handle.Instance = ATIM_TIMX_CPLM;                       /* ��ʱ��x */
//    Tim1Handle.Init.Prescaler = 0;                              /* Ԥ��Ƶϵ�� */
//    Tim1Handle.Init.CounterMode = TIM_CR1_CMS_0;                /* Up-Downģʽ���м�Գ� */
//    Tim1Handle.Init.Period = TIMER_PWM_CLK_FREQ/2/freq - 1;                              /* �Զ���װ��ֵ */
//    Tim1Handle.Init.ClockDivision = 0;
//    Tim1Handle.Init.RepetitionCounter = 0;
//    Tim1Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  /* ʹ��Ӱ�ӼĴ���TIMx_ARR */
//  
//    HAL_TIM_PWM_Init(&Tim1Handle) ;

//    sConfig.OCMode = TIM_OCMODE_PWM1;                               /* PWMģʽ1 */
//    sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;                       /* OCy �ߵ�ƽ��Ч */
//    sConfig.OCNPolarity = TIM_OCPOLARITY_LOW;                      /* OCyN �ߵ�ƽ��Ч */
//    sConfig.OCIdleState = TIM_OCIDLESTATE_SET;                      /* ��MOE=0��OCx=0 */
//    sConfig.OCNIdleState = TIM_OCNIDLESTATE_SET;                    /* ��MOE=0��OCxN=0 */
////    sConfig.Pulse = 4199;                                           /*   ����ռ�ձ�50%*/
//    HAL_TIM_PWM_ConfigChannel(&Tim1Handle, &sConfig, ATIM_TIMX_CPLM_CHY1);
//    HAL_TIM_PWM_ConfigChannel(&Tim1Handle, &sConfig, ATIM_TIMX_CPLM_CHY2);
//    HAL_TIM_PWM_ConfigChannel(&Tim1Handle, &sConfig, ATIM_TIMX_CPLM_CHY3);
//    sConfig.OCPolarity = TIM_OCPOLARITY_LOW;                       /* OCy �ߵ�ƽ��Ч */
//    sConfig.OCNPolarity = TIM_OCPOLARITY_LOW;                      /* OCyN �ߵ�ƽ��Ч */
//    sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;                      /* ��MOE=0��OCx=0 */
//    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;                    /* ��MOE=0��OCxN=0 */
//    HAL_TIM_PWM_ConfigChannel(&Tim1Handle, &sConfig, ATIM_TIMX_CPLM_CHY4);

//    /* �����������������������ж� */
//    sConfigBK.OffStateRunMode = TIM_OSSR_ENABLE;           /* ����ģʽ�Ĺر����״̬ */
//    sConfigBK.OffStateIDLEMode = TIM_OSSI_ENABLE;          /* ����ģʽ�Ĺر����״̬ */
//    sConfigBK.LockLevel = TIM_LOCKLEVEL_OFF;                /* ���üĴ��������� */
//    sConfigBK.BreakState = TIM_BREAK_ENABLE;                /* ʹ��ɲ������ */
//    sConfigBK.BreakPolarity = TIM_BREAKPOLARITY_HIGH;       /* ɲ��������Ч�źż���Ϊ�� */
//    sConfigBK.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE; /* ʹ��AOEλ������ɲ���������Զ��ָ���� */
//    sConfigBK.DeadTime = GetDtgByDeadTimeNS(PWM_DEAD_TIME_NS);
//    HAL_TIMEx_ConfigBreakDeadTime(&Tim1Handle, &sConfigBK);

//    HAL_TIM_PWM_Start(&Tim1Handle, ATIM_TIMX_CPLM_CHY1);         /* OCy ���ʹ�� */
//    HAL_TIMEx_PWMN_Start(&Tim1Handle, ATIM_TIMX_CPLM_CHY1);      /* OCyN ���ʹ�� */
//    HAL_TIM_PWM_Start(&Tim1Handle, ATIM_TIMX_CPLM_CHY2);         /* OCy ���ʹ�� */
//    HAL_TIMEx_PWMN_Start(&Tim1Handle, ATIM_TIMX_CPLM_CHY2);      /* OCyN ���ʹ�� */
//    HAL_TIM_PWM_Start(&Tim1Handle, ATIM_TIMX_CPLM_CHY3);         /* OCy ���ʹ�� */
//    HAL_TIMEx_PWMN_Start(&Tim1Handle, ATIM_TIMX_CPLM_CHY3);      /* OCyN ���ʹ�� */
//    

//    ATIM_TIMX_CPLM_CHY_CCRY1 = 0;                             /* ���ñȽϼĴ��� */
//    ATIM_TIMX_CPLM_CHY_CCRY2 = 0;                             /* ���ñȽϼĴ��� */
//    ATIM_TIMX_CPLM_CHY_CCRY3 = 0;                             /* ���ñȽϼĴ��� */
//    ATIM_TIMX_CPLM_CHY_CCRY4 = Tim1Handle.Init.Period;        /* ���ñȽϼĴ��� */

    ATIM_TIMX_CPLM->CR1 = (0x0 << TIM_CR1_CKD_Pos)    |     //�������������ʱ�Ӳ���Ƶ���������Ƶ��һ��
                          (0x1 << TIM_CR1_ARPE_Pos)   |	   //Ԥװ�ػ���ʹ��
                          (0x1 << TIM_CR1_CMS_Pos)    |     //����ģʽ�����Ķ���ģʽ1
                          (0x0 << TIM_CR1_DIR_Pos)    |     //������������
                          (0x0 << TIM_CR1_OPM_Pos)	   |	   //������ģʽ���ã����������¼�ʱ��ֹͣ����
                          (0x1 << TIM_CR1_URS_Pos)    |     //ֻ�м���������/��������ɸ����¼�(UEV)
                          (0x0 << TIM_CR1_UDIS_Pos)   |     //ʹ�ܸ����¼�
                          (0x0 << TIM_CR1_CEN_Pos)    ;     //��ֹ������
                          
    ATIM_TIMX_CPLM->CR2 = 0x0 << TIM_CR2_CCPC_Pos   |      //��ͨ�����ʹ�ܺ�ģʽѡ��λԤװ�عر�
                          0x0 << TIM_CR2_CCUS_Pos   |      //���CCPC=1��ֻ��ͨ��COMλ��������Ԥװ�ظ���
                          0x0 << TIM_CR2_CCDS_Pos   |      //ͨ����DMA������δʹ��
                          0x0 << TIM_CR2_MMS_Pos    |      //��ģʽ����ź�ѡ��δʹ��
                          0x0 << TIM_CR2_TI1S_Pos   |      //�����������ܹر�
                          0x0 << TIM_CR2_OIS1_Pos   |      //����״̬(MOE=0)��,ͨ�����ʹ��(CC1E=1)ʱ,OC1=0
                          0x0 << TIM_CR2_OIS1N_Pos  |      //����״̬(MOE=0)��,ͨ�����ʹ��(CC1NE=1)ʱ,OC1N=1
                          0x0 << TIM_CR2_OIS2_Pos   |      //����״̬(MOE=0)��,ͨ�����ʹ��(CC2E=1)ʱ,OC2=0
                          0x0 << TIM_CR2_OIS2N_Pos  |      //����״̬(MOE=0)��,ͨ�����ʹ��(CC2NE=1)ʱ,OC2N=1
                          0x0 << TIM_CR2_OIS3_Pos   |      //����״̬(MOE=0)��,ͨ�����ʹ��(CC3E=1)ʱ,OC3=0
                          0x0 << TIM_CR2_OIS3N_Pos  ;      //����״̬(MOE=0)��,ͨ�����ʹ��(CC3NE=1)ʱ,OC3N=1
                          
    ATIM_TIMX_CPLM->SMCR  = 0x0000;						    //��ģʽ���ƼĴ�������ֹ��ģʽ

    ATIM_TIMX_CPLM->DIER  = 0x0000;						    //DMA���жϿ��ƣ���ֹ�����ж�

    ATIM_TIMX_CPLM->PSC   = 0;                     //������Ԥ��Ƶ1��168M/1

    ATIM_TIMX_CPLM->ARR   = TIMER_PWM_CLK_FREQ/2/freq; //�Զ�װ��ֵ

    ATIM_TIMX_CPLM->RCR   = 1; //�ظ���������2���������һ�θ����¼������Ķ���ģʽ����Ϊһ��PWM����(����ʱ)����һ�θ����¼�


    ATIM_TIMX_CPLM->CCER = 0x1 << TIM_CCER_CC1E_Pos    |    //ʹ��CC1���
                           0x0 << TIM_CCER_CC1P_Pos    |    //CC1������ԣ��ߵ�ƽ��Ч
                           0x1 << TIM_CCER_CC1NE_Pos   |    //ʹ��CCN1���
                           0x1 << TIM_CCER_CC1NP_Pos   |    //CCN1������ԣ��ߵ�ƽ��Ч
                           0x1 << TIM_CCER_CC2E_Pos    |    //ʹ��CC2���
                           0x0 << TIM_CCER_CC2P_Pos    |    //CC2������ԣ��ߵ�ƽ��Ч
                           0x1 << TIM_CCER_CC2NE_Pos   |    //ʹ��CCN2���
                           0x1 << TIM_CCER_CC2NP_Pos   |    //CCN2������ԣ��ߵ�ƽ��Ч
                           0x1 << TIM_CCER_CC3E_Pos    |    //ʹ��CC3���
                           0x0 << TIM_CCER_CC3P_Pos    |    //CC3������ԣ��ߵ�ƽ��Ч
                           0x1 << TIM_CCER_CC3NE_Pos   |    //ʹ��CCN3���
                           0x1 << TIM_CCER_CC3NP_Pos   |    //CCN4������ԣ��ߵ�ƽ��Ч
                           0x1 << TIM_CCER_CC4E_Pos    |    //CC4�����ʹ��
                           0x0 << TIM_CCER_CC4P_Pos    ;    //CC4������ԣ��ߵ�ƽ��Ч

    ATIM_TIMX_CPLM->BDTR = GetDtgByDeadTimeNS(PWM_DEAD_TIME_NS)      |    //����ʱ��
                           0x0 << TIM_BDTR_LOCK_Pos    |    //���������ر�
                           0x1 << TIM_BDTR_OSSI_Pos    |    //����״̬(MOE=0)��,CC1E=0ʱ��Ч;1:�����Ч��ƽ(CC1P�ķ�״̬),0:����TIM����;
                           0x1 << TIM_BDTR_OSSR_Pos    |    //���״̬(MOE=1)��,CC1E=0ʱ��Ч;1:�����Ч��ƽ(CC1P�ķ�״̬),0:����TIM����;
                           0x0 << TIM_BDTR_BKE_Pos     |    //ɲ�����ܽ�ֹ
                           0x0 << TIM_BDTR_BKP_Pos     |    //ɲ�����뼫��
                           0x0 << TIM_BDTR_AOE_Pos     |    //�Զ������ֹ,MOEֻ���������1,������ڷ�����һ�����¼�ʱ�Զ���1
                           0x1 << TIM_BDTR_MOE_Pos     ;    //�����ʹ��

    ATIM_TIMX_CPLM->CCMR1 = 0x0 << TIM_CCMR1_CC1S_Pos   |    //CC1ͨ������Ϊ���
                       0x0 << TIM_CCMR1_OC1FE_Pos  |    //CC1��������ȽϽ�ֹ
                       0x1 << TIM_CCMR1_OC1PE_Pos  |    //CC1�ȽϼĴ�������ʹ�ܣ������������¼�ʱװ��
                       0x6 << TIM_CCMR1_OC1M_Pos   |    //CC1���ģʽPWM1
                       0x0 << TIM_CCMR1_OC1CE_Pos  |    //����Ƚ������ֹ,OC1Ref����ETRF(��ģʽ����Ч)����Ӱ��
                       0x0 << TIM_CCMR1_CC2S_Pos   |    //CC2ͨ������Ϊ���
                       0x0 << TIM_CCMR1_OC2FE_Pos  |    //CC2��������ȽϽ�ֹ
                       0x1 << TIM_CCMR1_OC2PE_Pos  |    //CC2�ȽϼĴ�������ʹ�ܣ������������¼�ʱװ��
                       0x6 << TIM_CCMR1_OC2M_Pos   |    ///CC2���ģʽPWM1
                       0x0 << TIM_CCMR1_OC2CE_Pos  ;    //����Ƚ������ֹ,OC2Ref����ETRF(��ģʽ����Ч)����Ӱ��

    ATIM_TIMX_CPLM->CCMR2 = 0x0 << TIM_CCMR2_CC3S_Pos   |    //CC3ͨ������Ϊ���
                       0x0 << TIM_CCMR2_OC3FE_Pos  |    //CC3��������ȽϽ�ֹ
                       0x1 << TIM_CCMR2_OC3PE_Pos  |    //CC3�ȽϼĴ�������ʹ�ܣ������������¼�ʱװ��
                       0x6 << TIM_CCMR2_OC3M_Pos   |    //CC3���ģʽPWM1
                       0x0 << TIM_CCMR2_OC3CE_Pos  |    //����Ƚ������ֹ,OC3Ref����ETRF(��ģʽ����Ч)����Ӱ��
                       0x0 << TIM_CCMR2_CC4S_Pos   |    //CC4ͨ������Ϊ���
                       0x0 << TIM_CCMR2_OC4FE_Pos  |    //CC4��������ȽϽ�ֹ
                       0x1 << TIM_CCMR2_OC4PE_Pos  |    //CC4�ȽϼĴ�������ʹ�ܣ������������¼�ʱװ��
                       0x7 << TIM_CCMR2_OC4M_Pos   |    //CC4���ģʽPWM1
                       0x0 << TIM_CCMR2_OC4CE_Pos  ;    //����Ƚ������ֹ,OC4Ref����ETRF(��ģʽ����Ч)����Ӱ��

    ATIM_TIMX_CPLM->CCR1 = 0;
    ATIM_TIMX_CPLM->CCR2 = 0;
    ATIM_TIMX_CPLM->CCR3 = 0;
    ATIM_TIMX_CPLM->CCR4 = ATIM_TIMX_CPLM->ARR - 1;

    ATIM_TIMX_CPLM->CR1 |= 0x01;  //ʹ�ܼ�����
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
            SetGpioOutMode(GPIO_OUT_MODE_PP);  //GPIO�л�Ϊ�������

            HAL_GPIO_WritePin(ATIM_TIMX_CPLM_CHY_GPIO_PORT,ATIM_TIMX_CPLM_CHY_GPIO_PIN1|ATIM_TIMX_CPLM_CHY_GPIO_PIN2|ATIM_TIMX_CPLM_CHY_GPIO_PIN3,GPIO_PIN_RESET);//GPIO���ȫ������
            HAL_GPIO_WritePin(ATIM_TIMX_CPLM_CHYN_GPIO_PORT,ATIM_TIMX_CPLM_CHYN_GPIO_PIN1|ATIM_TIMX_CPLM_CHYN_GPIO_PIN2|ATIM_TIMX_CPLM_CHYN_GPIO_PIN3,GPIO_PIN_SET);
        }
        break;

        case PWM_MODE_STOP:
        {
            SetGpioOutMode(GPIO_OUT_MODE_PP);  //GPIO�л�Ϊ�������

            HAL_GPIO_WritePin(ATIM_TIMX_CPLM_CHY_GPIO_PORT,ATIM_TIMX_CPLM_CHY_GPIO_PIN1|ATIM_TIMX_CPLM_CHY_GPIO_PIN2|ATIM_TIMX_CPLM_CHY_GPIO_PIN3,GPIO_PIN_RESET);//���ű�ȫ������
            HAL_GPIO_WritePin(ATIM_TIMX_CPLM_CHYN_GPIO_PORT,ATIM_TIMX_CPLM_CHYN_GPIO_PIN1|ATIM_TIMX_CPLM_CHYN_GPIO_PIN2|ATIM_TIMX_CPLM_CHYN_GPIO_PIN3,GPIO_PIN_RESET);//���ű�ȫ������
        }
        break;

        case PWM_MODE_BRAKE:
        {
            SetGpioOutMode(GPIO_OUT_MODE_H_PP_L_AF);  //GPIO�л�Ϊ���ű��������ű۸��ã�PWM��

            HAL_GPIO_WritePin(ATIM_TIMX_CPLM_CHY_GPIO_PORT,ATIM_TIMX_CPLM_CHY_GPIO_PIN1|ATIM_TIMX_CPLM_CHY_GPIO_PIN2|ATIM_TIMX_CPLM_CHY_GPIO_PIN3,GPIO_PIN_RESET);//���ű�ȫ������
        }
        break;

        case PWM_MODE_RUN:
        {
            SetGpioOutMode(GPIO_OUT_MODE_AF);  //GPIO�л�Ϊ�������
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
        ATIM_TIMX_CPLM_CHY_GPIO_PORT->MODER &= ~0x003F0000; //GPIO�л�Ϊ�������
        ATIM_TIMX_CPLM_CHYN_GPIO_PORT->MODER &= ~0xFC000000;
        ATIM_TIMX_CPLM_CHY_GPIO_PORT->MODER |= 0x00150000;
        ATIM_TIMX_CPLM_CHYN_GPIO_PORT->MODER |= 0x54000000;
    }
    else if(GPIO_OUT_MODE_AF == mode)
    {
        ATIM_TIMX_CPLM_CHY_GPIO_PORT->MODER &= ~0x003F0000; //GPIO�л�Ϊ�������
        ATIM_TIMX_CPLM_CHYN_GPIO_PORT->MODER &= ~0xFC000000;
        ATIM_TIMX_CPLM_CHY_GPIO_PORT->MODER |= 0x002A0000;
        ATIM_TIMX_CPLM_CHYN_GPIO_PORT->MODER |= 0xA8000000;
    } else if(GPIO_OUT_MODE_H_PP_L_AF == mode)     //���ű��������ű�PWM
    {
        ATIM_TIMX_CPLM_CHY_GPIO_PORT->MODER &= ~0x003F0000; //GPIO�л�Ϊ�������
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
