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
#include "Adc.h"
#include <stdlib.h>
#include <string.h>
#include "Dido.h"
#include "Xint.h"
//-------------------- local definitions ------------------------------------
const uint8_t  ADC_RG_SQUE[ADC_RQ_CH_NUM] = ADC_RG_CH_LIST;             //规则组转换序列
const GPIO_TypeDef* ADC_Port_List[ADC_RQ_CH_NUM] = ADC_GPIO_PORT_LIST;
const uint16_t ADC_Pin_List[ADC_RQ_CH_NUM] = ADC_GPIO_PIN_LIST;
//-------------------- private data -----------------------------------------
DAC_HandleTypeDef hdac_Handle1;

static uint16_t g_u16AdcData[ADC_RQ_CH_NUM] = {0};
static uint16_t g_u16AdcDataFilted[ADC_RQ_CH_NUM] = {0};

//-------------------- private functions declare ----------------------------
static void ADC_BSP_Init ( void );
static void DMA_Init ( void );
//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------


void Adc_Init()
{
    uint8_t i = 0;

    ADC_BSP_Init ( );

    /*Periph clock enable*/
    ADC_DMA_ClK_ENABLE();

    ADC3->CR2 &= ~ ( 1 << ADC_CR2_ADON_Pos ); //ADC3关闭
    ADC1->CR2 &= ~ ( 1 << ADC_CR2_ADON_Pos ); //ADC1关闭

    /*ADC configuration */
    ADC3->CR1  =      0 << ADC_CR1_OVRIE_Pos   | //溢出中断,禁止
                      0 << ADC_CR1_RES_Pos     | //分辨率,12位
                      0 << ADC_CR1_AWDCH_Pos   | //规则组模拟看门狗,禁止
                      0 << ADC_CR1_JAWDEN_Pos  | //注入组模拟看门狗,禁止
                      0 << ADC_CR1_JDISCEN_Pos | //注入通道间断模式,禁止
                      0 << ADC_CR1_DISCEN_Pos  | //规则通道间断模式,禁止
                      0 << ADC_CR1_JAUTO_Pos   | //注入组自动转换,禁止
                      1 << ADC_CR1_SCAN_Pos    | //扫描模式,使能
                      0 << ADC_CR1_JEOCIE_Pos  | //注入组转换完成中断,使能
                      0 << ADC_CR1_AWDIE_Pos   | //模拟看门狗中断,禁止
                      0 << ADC_CR1_EOCIE_Pos   ; //规则组转换完成中断,禁止

    ADC3->CR2 =       1 << ADC_CR2_SWSTART_Pos | //规则组转换使能,使能
                      10 << ADC_CR2_EXTEN_Pos   | //规则组外部触发,使能,下降沿
                      8 << ADC_CR2_EXTSEL_Pos  | //规则组外部触发事件,定时器3事件
                      0 << ADC_CR2_JSWSTART_Pos | //注入组转换使能,禁止
                      0 << ADC_CR2_JEXTEN_Pos  | //注入组外部触发,使能且上升沿有效
                      0 << ADC_CR2_JEXTSEL_Pos | //注入组外部触发事件,定时器1 TRGO
                      0 << ADC_CR2_ALIGN_Pos   | //数据对齐,右对齐
                      1 << ADC_CR2_DDS_Pos     | //连续DMA请求(对于单个ADC模式),使能
                      1 << ADC_CR2_DMA_Pos     | //使用DMA传输,使能
                      0 << ADC_CR2_CONT_Pos    | //连续转换模式(对于单个ADC模式),禁止
                      0 << ADC_CR2_ADON_Pos    ; //ADC开启,关闭

    ADC123_COMMON ->CCR   = 0 << ADC_CCR_ADCPRE_Pos | //ADC预分频,2分频(Tadcclk频率 = APB2/2=84M/2=42M)
                            1 << ADC_CCR_DMA_Pos    | //DMA模式(对于多重ADC模式),使能
                            0 << ADC_CCR_DDS_Pos    | //DMA连续请求(对于多重ADC模式),使能
                            0 << ADC_CCR_DELAY_Pos  | //2个采样阶段间延迟(对于多重ADC模式),5*Tadcclk
                            0 << ADC_CCR_MULTI_Pos  ; //多重ADC模式选择,双重仅注入同时模式
    //规则通道配置
    ADC3->SQR1      = ( ADC_RQ_CH_NUM - ADC_CH_0_Stop_V - 1 ) << 20;     //规则通道序列数目 0表示1，7表示8   13-9-1
    ADC3->SQR2      = 0;                                //序列清零
    ADC3->SQR3      = 0;

    for ( i = 0; i < ( ADC_RQ_CH_NUM - ADC_CH_0_Stop_V ); i++ )           //规则通道序列---4
    {
        if ( i < 6 )
        {
            ADC3->SQR3 |= ADC_RG_SQUE[i + ADC_CH_0_Stop_V] << ( ( i ) * 5 );//5是SQ1,6是SQ2,7是SQ3,8是SQ4
        }
        else if ( i < 12 )
        {
            ADC3->SQR2 |= ADC_RG_SQUE[i + ADC_CH_0_Stop_V] << ( ( i - 6 ) * 5 );
        }
        else if ( i < 16 )
        {
            ADC3->SQR1 |= ADC_RG_SQUE[i + ADC_CH_0_Stop_V] << ( ( i - 12 ) * 5 );
        }
    }

    /*ADC configuration */
    ADC1->CR1  =      0 << ADC_CR1_OVRIE_Pos   | //溢出中断,禁止
                      0 << ADC_CR1_RES_Pos     | //分辨率,12位
                      0 << ADC_CR1_AWDCH_Pos   | //规则组模拟看门狗,禁止
                      0 << ADC_CR1_JAWDEN_Pos  | //注入组模拟看门狗,禁止
                      0 << ADC_CR1_JDISCEN_Pos | //注入通道间断模式,禁止
                      0 << ADC_CR1_DISCEN_Pos  | //规则通道间断模式,禁止
                      0 << ADC_CR1_JAUTO_Pos   | //注入组自动转换,禁止
                      1 << ADC_CR1_SCAN_Pos    | //扫描模式,使能
                      0 << ADC_CR1_JEOCIE_Pos  | //注入组转换完成中断,使能
                      0 << ADC_CR1_AWDIE_Pos   | //模拟看门狗中断,禁止
                      0 << ADC_CR1_EOCIE_Pos   ; //规则组转换完成中断,禁止

    ADC1->CR2 =       1 << ADC_CR2_SWSTART_Pos | //规则组转换使能,使能
                      10 << ADC_CR2_EXTEN_Pos   | //规则组外部触发,使能,下降沿
                      8 << ADC_CR2_EXTSEL_Pos  | //规则组外部触发事件,定时器3事件
                      0 << ADC_CR2_JSWSTART_Pos | //注入组转换使能,禁止
                      0 << ADC_CR2_JEXTEN_Pos  | //注入组外部触发,使能且上升沿有效
                      0 << ADC_CR2_JEXTSEL_Pos | //注入组外部触发事件,定时器1 TRGO
                      0 << ADC_CR2_ALIGN_Pos   | //数据对齐,右对齐
                      1 << ADC_CR2_DDS_Pos     | //连续DMA请求(对于单个ADC模式),使能
                      1 << ADC_CR2_DMA_Pos     | //使用DMA传输,使能
                      0 << ADC_CR2_CONT_Pos    | //连续转换模式(对于单个ADC模式),禁止
                      0 << ADC_CR2_ADON_Pos    ; //ADC开启,关闭

    ADC123_COMMON ->CCR   = 0 << ADC_CCR_ADCPRE_Pos | //ADC预分频,2分频(Tadcclk频率 = APB2/2=84M/2=42M)
                            1 << ADC_CCR_DMA_Pos    | //DMA模式(对于多重ADC模式),使能
                            0 << ADC_CCR_DDS_Pos    | //DMA连续请求(对于多重ADC模式),使能
                            0 << ADC_CCR_DELAY_Pos  | //2个采样阶段间延迟(对于多重ADC模式),5*Tadcclk
                            0 << ADC_CCR_MULTI_Pos  ; //多重ADC模式选择,双重仅注入同时模式
    //规则通道配置
    ADC1->SQR1      = ( ADC_CH_0_Stop_V - 1 ) << 20;       //规则通道序列数目 0表示1，7表示8 --9-1
    ADC1->SQR2      = 0;                                //序列清零
    ADC1->SQR3      = 0;

    for ( i = 0; i < ADC_CH_0_Stop_V; i++ )               //规则通道序列--9
    {
        if ( i < 6 )
        {
            ADC1->SQR3 |= ADC_RG_SQUE[i] << ( i * 5 );//    10是SQ1,12是SQ2,13是SQ3,14是SQ4,15是SQ5,8是SQ6,
        }
        else if ( i < 12 )
        {
            ADC1->SQR2 |= ADC_RG_SQUE[i] << ( ( i - 6 ) * 5 );//9是SQ7,4是SQ8,5是SQ9
        }
        else if ( i < 16 )
        {
            ADC1->SQR1 |= ADC_RG_SQUE[i] << ( ( i - 12 ) * 5 );
        }
    }

    DMA_Init();

    //中断配置
    HAL_NVIC_SetPriority ( ADC_IRQn, IRQ_Priority_ADC_Irq, 0 );
    HAL_NVIC_EnableIRQ ( ADC_IRQn );

    ADC3->CR2 |= 1 << ADC_CR2_ADON_Pos; //ADC开启
    ADC1->CR2 |= 1 << ADC_CR2_ADON_Pos; //ADC开启
}
void ADC_IRQHandler ( void )
{

}


/**
  * @brief  This function handles DMA interrupt request.
  * @param  None
  * @retval None
  */
void ADC_DMA_IRQ_Handler ( void )
{
    //通道下降沿后约5uS执行到此处
    ADC_DMA_IRQ_CLEAR();
//    ADC3 ->SR &= ~ADC_SR_OVR_Msk;
//    ADC1 ->SR &= ~ADC_SR_OVR_Msk;

}
void ADC_DMA_IRQ_Handler_2 ( void )
{
    //通道下降沿后约5uS执行到此处
    ADC_DMA_IRQ_CLEAR_2();
//    ADC3 ->SR &= ~ADC_SR_OVR_Msk;
//    ADC1 ->SR &= ~ADC_SR_OVR_Msk;

}

void ADC_SWStart ( void )
{
    ADC3->CR2 |= ADC_CR2_SWSTART_Msk;//软件触发（仅有Timer7不支持做触发源）
    ADC1->CR2 |= ADC_CR2_SWSTART_Msk;//软件触发（仅有Timer7不支持做触发源）
}


/*! \fn        uint8_t ADC_GetSrcValue(uint16_t* pu16Buffer)
 *  \brief     get converted value buffer address of adc
 *  \param     pu16Buffer: point to couvert value buffer
 *  \return    0:success other:false
 */
uint16_t ADC_GetValue ( E_ADC_CH ch )
{
    uint16_t u16Data;

    if ( ch < ADC_RQ_CH_NUM )
    {
        u16Data = g_u16AdcData[ch];
    }
    return u16Data;
}

//-------------------- private functions ------------------------------------

static void ADC_BSP_Init ( void )
{
    GPIO_InitTypeDef   GPIO_InitStruct;
    uint16_t i;

    //规则组 ADC GPIO 配置
    GPIO_InitStruct.Mode      = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

    for ( i = 0; i < ADC_RQ_CH_NUM; i++ )
    {
        GPIO_InitStruct.Pin = ADC_Pin_List[i];
        HAL_GPIO_Init ( ( GPIO_TypeDef* ) ADC_Port_List[i], &GPIO_InitStruct );
    }

    //ADC时钟使能
    __HAL_RCC_ADC3_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    //ADC通道采样时间
    ADC1->SMPR1 =        ADC_SampleTime          |
                         ADC_SampleTime << 3     |
                         ADC_SampleTime << 6     |
                         ADC_SampleTime << 9     |
                         ADC_SampleTime << 12    |
                         ADC_SampleTime << 15    |
                         ADC_SampleTime << 18    |
                         ADC_SampleTime << 21    |
                         ADC_SampleTime << 24    ;

    ADC1->SMPR2 =           ADC_SampleTime          |
                            ADC_SampleTime << 3     |
                            ADC_SampleTime << 6     |
                            ADC_SampleTime << 9     |
                            ADC_SampleTime << 12    |
                            ADC_SampleTime << 15    |
                            ADC_SampleTime << 18    |
                            ADC_SampleTime << 21    |
                            ADC_SampleTime << 24    |
                            ADC_SampleTime << 27    ;

    ADC3->SMPR1 =        ADC_SampleTime          |
                         ADC_SampleTime << 3     |
                         ADC_SampleTime << 6     |
                         ADC_SampleTime << 9     |
                         ADC_SampleTime << 12    |
                         ADC_SampleTime << 15    |
                         ADC_SampleTime << 18    |
                         ADC_SampleTime << 21    |
                         ADC_SampleTime << 24    ;

    ADC3->SMPR2 =           ADC_SampleTime          |
                            ADC_SampleTime << 3     |
                            ADC_SampleTime << 6     |
                            ADC_SampleTime << 9     |
                            ADC_SampleTime << 12    |
                            ADC_SampleTime << 15    |
                            ADC_SampleTime << 18    |
                            ADC_SampleTime << 21    |
                            ADC_SampleTime << 24    |
                            ADC_SampleTime << 27    ;

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

    ADC_DMA_STREAM->M0AR = ( uint32_t ) ( g_u16AdcData + ADC_CH_MOTOR2_24V_V + 1 ); //存储器地址
    ADC_DMA_STREAM->PAR = ( uint32_t ) &ADC3->DR; //外设地址
    ADC_DMA_STREAM->NDTR = ADC_CH_MOTOR2_24V_I - ADC_CH_MOTOR2_24V_V;        //数据传输数量
    ADC_DMA_STREAM->FCR =   0x00 << DMA_SxFCR_FEIE_Pos    //FIFO错误中断,禁止
                            | 0x00 << DMA_SxFCR_DMDIS_Pos; //直接模式,使能


    HAL_NVIC_SetPriority ( ADC_DMA_IRQ, IRQ_Priority_ADC_DMA, 0 );
    HAL_NVIC_EnableIRQ ( ADC_DMA_IRQ );

    ADC_DMA_STREAM->CR |=  0x01; //使能通道

    ADC_DMA_STREAM_2->CR = 0;
    ADC_DMA_STREAM_2->CR =           ADC_DMA_CHANNEL_2           //通道
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

    ADC_DMA_STREAM_2->M0AR = ( uint32_t ) ( g_u16AdcData ); //存储器地址
    ADC_DMA_STREAM_2->PAR = ( uint32_t ) &ADC1->DR; //外设地址
    ADC_DMA_STREAM_2->NDTR = ADC_CH_MOTOR2_24V_V + 1;      //数据传输数量
    ADC_DMA_STREAM_2->FCR =   0x00 << DMA_SxFCR_FEIE_Pos    //FIFO错误中断,禁止
                              | 0x00 << DMA_SxFCR_DMDIS_Pos; //直接模式,使能
    HAL_NVIC_SetPriority ( ADC_DMA_IRQ_2, IRQ_Priority_ADC_DMA, 0 );
    HAL_NVIC_EnableIRQ ( ADC_DMA_IRQ_2 );
    ADC_DMA_STREAM_2->CR |=  0x01; //使能通道
}
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


