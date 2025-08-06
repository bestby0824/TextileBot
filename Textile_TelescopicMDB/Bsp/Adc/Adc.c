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
//-------------------- include files ----------------------------------------
#include "ADC.h"
#include "Int_Ctrl.h"
#include <string.h>
#include "VVVF.h"
#include "SVPWM.h"
#include "TimerPWM.h"
#include "Foc.h"

//-------------------- local definitions ------------------------------------

const uint8_t  ADC_RG_SQUE[ADC_CH_RQ_NUM] = ADC_RG_CH_LIST;                //规则组转换序列
const GPIO_TypeDef* ADC_Port_List[ADC_CH_ALL_NUM] = ADC_GPIO_PORT_LIST;    //所有ADC引脚
const uint16_t ADC_Pin_List[ADC_CH_ALL_NUM] = ADC_GPIO_PIN_LIST;

//-------------------- private data -----------------------------------------
uint16_t g_u16AdcData[ADC_CH_RQ_NUM] = {0};
static uint16_t g_u16AdcDataFilted[ADC_CH_RQ_NUM] = {0};

ADC_HandleTypeDef  hadc1, hadc2;
extern TIM_HandleTypeDef Tim1Handle;                             /* 定时器x句柄 */
uint16_t uhADCxConvertedRegValue = 0; // 存储规则通道转换值
uint16_t uhADCxConvertedInjValue = 0; // 存储注入通道转换值
_iq CurrentA = 0, CurrentB = 0, CurrentC = 0;

static void BubbleSort ( uint16_t* pbuf, uint16_t lenth );
static void ADC_BSP_Init ( void );
static void DMA_Init ( void );
uint16_t ADC_GetValue ( E_ADC_CH ch );
/*! \fn			  void Adc_Init(E_DEVICE_TYPE  dev)
 *  \brief 		ADC initialization function
 *  \param 		psAdcCurrentDef
 *  \return 	0:success other:false
 */
void ADC_Init()
{
    uint8_t i = 0;

    ADC_BSP_Init ( );

    /*Periph clock enable*/
    ADC_DMA_ClK_ENABLE();

    ADC1->CR2 &= ~ ( 1 << ADC_CR2_ADON_Pos ); //ADC1关闭
//    ADC2->CR2 &= ~(1 << ADC_CR2_ADON_Pos); //ADC2关闭
    
    /*ADC configuration */
    ADC1->CR1  =      0 << ADC_CR1_OVRIE_Pos   | //溢出中断,禁止
                      0 << ADC_CR1_RES_Pos     | //分辨率,12位
                      0 << ADC_CR1_AWDCH_Pos   | //规则组模拟看门狗,禁止
                      0 << ADC_CR1_JAWDEN_Pos  | //注入组模拟看门狗,禁止
                      0 << ADC_CR1_JDISCEN_Pos | //注入通道间断模式,禁止
                      0 << ADC_CR1_DISCEN_Pos  | //规则通道间断模式,禁止
                      0 << ADC_CR1_JAUTO_Pos   | //注入组自动转换,禁止
                      1 << ADC_CR1_SCAN_Pos    | //扫描模式,使能
                      1 << ADC_CR1_JEOCIE_Pos  | //注入组转换完成中断,使能
                      0 << ADC_CR1_AWDIE_Pos   | //模拟看门狗中断,禁止
                      0 << ADC_CR1_EOCIE_Pos   ; //规则组转换完成中断,禁止

    ADC1->CR2 =       1 << ADC_CR2_SWSTART_Pos | //规则组转换使能,使能
                      2 << ADC_CR2_EXTEN_Pos   | //规则组外部触发,使能,下降沿
                      8 << ADC_CR2_EXTSEL_Pos  | //规则组外部触发事件,定时器3事件
                      0 << ADC_CR2_JSWSTART_Pos | //注入组转换使能,禁止
                      1 << ADC_CR2_JEXTEN_Pos  | //注入组外部触发,使能且上升沿有效
                      0 << ADC_CR2_JEXTSEL_Pos | //注入组外部触发事件,定时器1 TRGO
                      0 << ADC_CR2_ALIGN_Pos   | //数据对齐,右对齐
                      1 << ADC_CR2_DDS_Pos     | //连续DMA请求(对于单个ADC模式),使能
                      1 << ADC_CR2_DMA_Pos     | //使用DMA传输,使能
                      0 << ADC_CR2_CONT_Pos    | //连续转换模式(对于单个ADC模式),禁止
                      0 << ADC_CR2_ADON_Pos    ; //ADC开启,关闭
                      
    ADC123_COMMON ->CCR   = 0 << ADC_CCR_ADCPRE_Pos | //ADC预分频,2分频(Tadcclk频率 = APB2/2=84M/2=42M)
                            0 << ADC_CCR_DMA_Pos    | //DMA模式(对于多重ADC模式),使能
                            0 << ADC_CCR_DDS_Pos    | //DMA连续请求(对于多重ADC模式),使能
                            0 << ADC_CCR_DELAY_Pos  | //2个采样阶段间延迟(对于多重ADC模式),5*Tadcclk
                            5 << ADC_CCR_MULTI_Pos  ; //多重ADC模式选择,双重仅注入同时模式

    ADC1->JOFR1 = 0; //注入组数据偏移

    //规则通道配置
    ADC1->SQR1      = ( ADC_CH_RQ_NUM - 1 ) << 20;      //规则通道序列数目
    ADC1->SQR2      = 0;                                //序列清零
    ADC1->SQR3      = 0;

    for ( i = 0; i < ADC_CH_RQ_NUM; i++ )               //规则通道序列
    {
        if ( i < 6 )
        {
            ADC1->SQR3 |= ADC_RG_SQUE[i] << ( i * 5 );
        }
        else if ( i < 12 )
        {
            ADC1->SQR2 |= ADC_RG_SQUE[i] << ( ( i - 6 ) * 5 );
        }
        else if ( i < 16 )
        {
            ADC1->SQR1 |= ADC_RG_SQUE[i] << ( ( i - 12 ) * 5 );
        }
    }

    ADC1->JSQR = 2 << ADC_JSQR_JL_Pos                            | //注如组序列长度  2+1=3
                 ADC_I_1_ChSQR << ADC_JSQR_JSQ2_Pos | //注入组第一次转换
                 ADC_I_2_ChSQR << ADC_JSQR_JSQ3_Pos | //注入组第二次转换
                 ADC_I_3_ChSQR << ADC_JSQR_JSQ4_Pos ; //注入组第三次转换
    
    DMA_Init();

    //中断配置
    HAL_NVIC_SetPriority ( ADC_IRQn, IRQ_Priority_TIM_PWM, 0 );
    HAL_NVIC_EnableIRQ ( ADC_IRQn );

    ADC1->CR2 |= 1 << ADC_CR2_ADON_Pos; //ADC开启
}
//-------------------- private functions ------------------------------------

// ADC中断服务函数(注入组)
void ADC_IRQHandler()
{
    ADC1->SR &= ~ADC_SR_JEOC_Msk;
    
    ADC_CurrentSampleCpl_Callback ( );
}
/*! \fn        void ADC_VoltageSampleStart(void)
 *  \brief     adc convert start
 *  \param     none
 *  \return    0:success other:false
 */
void ADC_VoltageSampleStart(void)
{
    ADC1->CR2 |= 1<<ADC_CR2_SWSTART_Pos;
}

static void ADC_BSP_Init ( void )
{
    GPIO_InitTypeDef   GPIO_InitStruct;
    uint16_t i;

    //规则组 ADC GPIO 配置
    GPIO_InitStruct.Mode      = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

    for ( i = 0; i < ADC_CH_ALL_NUM; i++ )
    {
        GPIO_InitStruct.Pin = ADC_Pin_List[i];
        HAL_GPIO_Init ( ( GPIO_TypeDef* ) ADC_Port_List[i], &GPIO_InitStruct );
    }

    //ADC时钟使能
    __HAL_RCC_ADC1_CLK_ENABLE();

    //ADC1   通道采样时间
    ADC1->SMPR1	=           ADC_SampleTime	  		|
                            ADC_SampleTime << 3		|
                            ADC_SampleTime << 6		|
                            ADC_SampleTime << 9		|
                            ADC_SampleTime << 12	|
                            ADC_SampleTime << 15	|
                            ADC_SampleTime << 18	|
                            ADC_SampleTime << 21	|
                            ADC_SampleTime << 24	;

    ADC1->SMPR2 =           ADC_SampleTime	  		|
                            ADC_SampleTime << 3		|
                            ADC_SampleTime << 6		|
                            ADC_SampleTime << 9		|
                            ADC_SampleTime << 12	|
                            ADC_SampleTime << 15	|
                            ADC_SampleTime << 18	|
                            ADC_SampleTime << 21	|
                            ADC_SampleTime << 24	|
                            ADC_SampleTime << 27	;


}


/**
  * @brief  This function handles DMA interrupt request.
  * @param  None
  * @retval None
  * @note   timer3下降沿触发采样（20KHZ）
  */
void ADC_DMA_IRQ_Handler ( void )
{
    ADC_DMA_IRQ_CLEAR();
}


/*! \fn        uint8_t ADC_GetSrcValue(uint16_t* pu16Buffer)
 *  \brief     get converted value buffer address of adc
 *  \param     pu16Buffer: point to couvert value buffer
 *  \return    0:success other:false
 */
uint16_t ADC_GetValue ( E_ADC_CH ch )
{
    uint16_t u16Data = 0;

    if ( ch < ADC_CH_RQ_NUM )
    {
        u16Data = g_u16AdcData[ch];
    }
    else if( ch == ADC_CH_I_1 )
    {
        u16Data = ADC1->JDR1;
    }
    else if( ch == ADC_CH_I_2 )
    {
        u16Data = ADC1->JDR2;
    }
    else if( ch == ADC_CH_I_3 )
    {
        u16Data = ADC1->JDR3;
    }
    else
    {
        ;
    }
    
    return u16Data;
}
void ADC_RQ_Filter ( void )
{

#define FILT_FIFO_DEPTH   6
#define FILT_EXTREME_NUM   2

    static uint16_t g_pu16AdcFilterFifo[ADC_CH_RQ_NUM][FILT_FIFO_DEPTH] = {0};
    static uint8_t u8AdcFilterPos = 0;
    uint8_t ch, i;
    uint16_t pu16TempBuf[FILT_FIFO_DEPTH];

    for ( ch = 0; ch < ADC_CH_RQ_NUM; ch++ )
    {
        uint32_t u32Temp = 0;

        g_pu16AdcFilterFifo[ch][u8AdcFilterPos] = g_u16AdcData[ch];
        memcpy ( pu16TempBuf, g_pu16AdcFilterFifo[ch], FILT_FIFO_DEPTH * 2 );
        BubbleSort ( pu16TempBuf, FILT_FIFO_DEPTH );

        for ( i = FILT_EXTREME_NUM; i < FILT_FIFO_DEPTH - FILT_EXTREME_NUM; i++ )
        {
            u32Temp += pu16TempBuf[i];
        }

        g_u16AdcDataFilted[ch] = u32Temp / ( FILT_FIFO_DEPTH - FILT_EXTREME_NUM - FILT_EXTREME_NUM );
    }

    u8AdcFilterPos++;
    if ( u8AdcFilterPos == FILT_FIFO_DEPTH )
    {
        u8AdcFilterPos = 0;
    }

}
static void DMA_Init ( void )
{
    ADC_DMA_ClK_ENABLE();

    ADC_DMA_STREAM->CR = 0;
    ADC_DMA_STREAM->CR =           ADC_DMA_CHANNEL            //通道
                                   | 0x00 << DMA_SxCR_MBURST_Pos
                                   | 0x00 << DMA_SxCR_PBURST_Pos
                                   | 0x00 << DMA_SxCR_CT_Pos
                                   | 0x00 << DMA_SxCR_DBM_Pos
                                   | 0x01 << DMA_SxCR_PL_Pos         //通道优先级 中
                                   | 0x00 << DMA_SxCR_PINCOS_Pos
                                   | 0x01 << DMA_SxCR_MSIZE_Pos      //存储器数据宽度 16位
                                   | 0x01 << DMA_SxCR_PSIZE_Pos      //外设数据宽度 16位
                                   | 0x01 << DMA_SxCR_MINC_Pos       //存储器地址递增
                                   | 0x00 << DMA_SxCR_PINC_Pos       //外设地址不递增
                                   | 0x01 << DMA_SxCR_CIRC_Pos       //循环模式,使能
                                   | 0x00 << DMA_SxCR_DIR_Pos        //外设到存储器
                                   | 0x00 << DMA_SxCR_PFCTRL_Pos
                                   | 0x01 << DMA_SxCR_TCIE_Pos       //传输完成中断,使能
                                   | 0x00 << DMA_SxCR_HTIE_Pos
                                   | 0x00 << DMA_SxCR_TEIE_Pos       //传输错误中断,禁止
                                   | 0x00 << DMA_SxCR_DMEIE_Pos
                                   | 0x00 << DMA_SxCR_EN_Pos;        //通道不使能

    ADC_DMA_STREAM->M0AR = ( uint32_t ) g_u16AdcData;//存储器地址
    ADC_DMA_STREAM->PAR = ( uint32_t ) &ADC1->DR; //外设地址
    ADC_DMA_STREAM->NDTR = ADC_CH_RQ_NUM;        //数据传输数量
    ADC_DMA_STREAM->FCR =   0x00 << DMA_SxFCR_FEIE_Pos    //FIFO错误中断,禁止
                            | 0x00 << DMA_SxFCR_DMDIS_Pos; //直接模式,使能


    HAL_NVIC_SetPriority ( ADC_DMA_IRQ, IRQ_Priority_ADC_DMA, 0 );
    HAL_NVIC_EnableIRQ ( ADC_DMA_IRQ );

    ADC_DMA_STREAM->CR |=  0x01; //使能通道

}
static void BubbleSort ( uint16_t* pbuf, uint16_t lenth )
{
    uint16_t i, j, u16Data;

    for ( j = 0; j < lenth - 1; j++ )
    {
        for ( i = 0; i < lenth - 1 - j; i++ )
        {
            if ( pbuf[i] > pbuf[i + 1] )
            {
                u16Data = pbuf[i];
                pbuf[i] = pbuf[i + 1];
                pbuf[i + 1] = u16Data;
            }
        }
    }
}
