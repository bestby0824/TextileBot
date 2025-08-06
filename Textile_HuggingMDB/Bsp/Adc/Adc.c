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
//-------------------- include files ----------------------------------------
#include "ADC.h"
#include "Int_Ctrl.h"
#include <string.h>
#include "VVVF.h"
#include "SVPWM.h"
#include "TimerPWM.h"
#include "Foc.h"

//-------------------- local definitions ------------------------------------

const uint8_t  ADC_RG_SQUE[ADC_CH_RQ_NUM] = ADC_RG_CH_LIST;                //������ת������
const GPIO_TypeDef* ADC_Port_List[ADC_CH_ALL_NUM] = ADC_GPIO_PORT_LIST;    //����ADC����
const uint16_t ADC_Pin_List[ADC_CH_ALL_NUM] = ADC_GPIO_PIN_LIST;

//-------------------- private data -----------------------------------------
uint16_t g_u16AdcData[ADC_CH_RQ_NUM] = {0};
static uint16_t g_u16AdcDataFilted[ADC_CH_RQ_NUM] = {0};

ADC_HandleTypeDef  hadc1, hadc2;
extern TIM_HandleTypeDef Tim1Handle;                             /* ��ʱ��x��� */
uint16_t uhADCxConvertedRegValue = 0; // �洢����ͨ��ת��ֵ
uint16_t uhADCxConvertedInjValue = 0; // �洢ע��ͨ��ת��ֵ
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

    ADC1->CR2 &= ~ ( 1 << ADC_CR2_ADON_Pos ); //ADC1�ر�
//    ADC2->CR2 &= ~(1 << ADC_CR2_ADON_Pos); //ADC2�ر�
    
    /*ADC configuration */
    ADC1->CR1  =      0 << ADC_CR1_OVRIE_Pos   | //����ж�,��ֹ
                      0 << ADC_CR1_RES_Pos     | //�ֱ���,12λ
                      0 << ADC_CR1_AWDCH_Pos   | //������ģ�⿴�Ź�,��ֹ
                      0 << ADC_CR1_JAWDEN_Pos  | //ע����ģ�⿴�Ź�,��ֹ
                      0 << ADC_CR1_JDISCEN_Pos | //ע��ͨ�����ģʽ,��ֹ
                      0 << ADC_CR1_DISCEN_Pos  | //����ͨ�����ģʽ,��ֹ
                      0 << ADC_CR1_JAUTO_Pos   | //ע�����Զ�ת��,��ֹ
                      1 << ADC_CR1_SCAN_Pos    | //ɨ��ģʽ,ʹ��
                      1 << ADC_CR1_JEOCIE_Pos  | //ע����ת������ж�,ʹ��
                      0 << ADC_CR1_AWDIE_Pos   | //ģ�⿴�Ź��ж�,��ֹ
                      0 << ADC_CR1_EOCIE_Pos   ; //������ת������ж�,��ֹ

    ADC1->CR2 =       1 << ADC_CR2_SWSTART_Pos | //������ת��ʹ��,ʹ��
                      2 << ADC_CR2_EXTEN_Pos   | //�������ⲿ����,ʹ��,�½���
                      8 << ADC_CR2_EXTSEL_Pos  | //�������ⲿ�����¼�,��ʱ��3�¼�
                      0 << ADC_CR2_JSWSTART_Pos | //ע����ת��ʹ��,��ֹ
                      1 << ADC_CR2_JEXTEN_Pos  | //ע�����ⲿ����,ʹ������������Ч
                      0 << ADC_CR2_JEXTSEL_Pos | //ע�����ⲿ�����¼�,��ʱ��1 TRGO
                      0 << ADC_CR2_ALIGN_Pos   | //���ݶ���,�Ҷ���
                      1 << ADC_CR2_DDS_Pos     | //����DMA����(���ڵ���ADCģʽ),ʹ��
                      1 << ADC_CR2_DMA_Pos     | //ʹ��DMA����,ʹ��
                      0 << ADC_CR2_CONT_Pos    | //����ת��ģʽ(���ڵ���ADCģʽ),��ֹ
                      0 << ADC_CR2_ADON_Pos    ; //ADC����,�ر�
                      
    ADC123_COMMON ->CCR   = 0 << ADC_CCR_ADCPRE_Pos | //ADCԤ��Ƶ,2��Ƶ(TadcclkƵ�� = APB2/2=84M/2=42M)
                            0 << ADC_CCR_DMA_Pos    | //DMAģʽ(���ڶ���ADCģʽ),ʹ��
                            0 << ADC_CCR_DDS_Pos    | //DMA��������(���ڶ���ADCģʽ),ʹ��
                            0 << ADC_CCR_DELAY_Pos  | //2�������׶μ��ӳ�(���ڶ���ADCģʽ),5*Tadcclk
                            5 << ADC_CCR_MULTI_Pos  ; //����ADCģʽѡ��,˫�ؽ�ע��ͬʱģʽ

    ADC1->JOFR1 = 0; //ע��������ƫ��

    //����ͨ������
    ADC1->SQR1      = ( ADC_CH_RQ_NUM - 1 ) << 20;      //����ͨ��������Ŀ
    ADC1->SQR2      = 0;                                //��������
    ADC1->SQR3      = 0;

    for ( i = 0; i < ADC_CH_RQ_NUM; i++ )               //����ͨ������
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

    ADC1->JSQR = 2 << ADC_JSQR_JL_Pos                            | //ע�������г���  2+1=3
                 ADC_I_1_ChSQR << ADC_JSQR_JSQ2_Pos | //ע�����һ��ת��
                 ADC_I_2_ChSQR << ADC_JSQR_JSQ3_Pos | //ע����ڶ���ת��
                 ADC_I_3_ChSQR << ADC_JSQR_JSQ4_Pos ; //ע���������ת��
    
    DMA_Init();

    //�ж�����
    HAL_NVIC_SetPriority ( ADC_IRQn, IRQ_Priority_TIM_PWM, 0 );
    HAL_NVIC_EnableIRQ ( ADC_IRQn );

    ADC1->CR2 |= 1 << ADC_CR2_ADON_Pos; //ADC����
}
//-------------------- private functions ------------------------------------

// ADC�жϷ�����(ע����)
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

    //������ ADC GPIO ����
    GPIO_InitStruct.Mode      = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

    for ( i = 0; i < ADC_CH_ALL_NUM; i++ )
    {
        GPIO_InitStruct.Pin = ADC_Pin_List[i];
        HAL_GPIO_Init ( ( GPIO_TypeDef* ) ADC_Port_List[i], &GPIO_InitStruct );
    }

    //ADCʱ��ʹ��
    __HAL_RCC_ADC1_CLK_ENABLE();

    //ADC1   ͨ������ʱ��
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
  * @note   timer3�½��ش���������20KHZ��
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
    ADC_DMA_STREAM->CR =           ADC_DMA_CHANNEL            //ͨ��
                                   | 0x00 << DMA_SxCR_MBURST_Pos
                                   | 0x00 << DMA_SxCR_PBURST_Pos
                                   | 0x00 << DMA_SxCR_CT_Pos
                                   | 0x00 << DMA_SxCR_DBM_Pos
                                   | 0x01 << DMA_SxCR_PL_Pos         //ͨ�����ȼ� ��
                                   | 0x00 << DMA_SxCR_PINCOS_Pos
                                   | 0x01 << DMA_SxCR_MSIZE_Pos      //�洢�����ݿ�� 16λ
                                   | 0x01 << DMA_SxCR_PSIZE_Pos      //�������ݿ�� 16λ
                                   | 0x01 << DMA_SxCR_MINC_Pos       //�洢����ַ����
                                   | 0x00 << DMA_SxCR_PINC_Pos       //�����ַ������
                                   | 0x01 << DMA_SxCR_CIRC_Pos       //ѭ��ģʽ,ʹ��
                                   | 0x00 << DMA_SxCR_DIR_Pos        //���赽�洢��
                                   | 0x00 << DMA_SxCR_PFCTRL_Pos
                                   | 0x01 << DMA_SxCR_TCIE_Pos       //��������ж�,ʹ��
                                   | 0x00 << DMA_SxCR_HTIE_Pos
                                   | 0x00 << DMA_SxCR_TEIE_Pos       //��������ж�,��ֹ
                                   | 0x00 << DMA_SxCR_DMEIE_Pos
                                   | 0x00 << DMA_SxCR_EN_Pos;        //ͨ����ʹ��

    ADC_DMA_STREAM->M0AR = ( uint32_t ) g_u16AdcData;//�洢����ַ
    ADC_DMA_STREAM->PAR = ( uint32_t ) &ADC1->DR; //�����ַ
    ADC_DMA_STREAM->NDTR = ADC_CH_RQ_NUM;        //���ݴ�������
    ADC_DMA_STREAM->FCR =   0x00 << DMA_SxFCR_FEIE_Pos    //FIFO�����ж�,��ֹ
                            | 0x00 << DMA_SxFCR_DMDIS_Pos; //ֱ��ģʽ,ʹ��


    HAL_NVIC_SetPriority ( ADC_DMA_IRQ, IRQ_Priority_ADC_DMA, 0 );
    HAL_NVIC_EnableIRQ ( ADC_DMA_IRQ );

    ADC_DMA_STREAM->CR |=  0x01; //ʹ��ͨ��

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
