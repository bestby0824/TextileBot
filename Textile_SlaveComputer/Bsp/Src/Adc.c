/**
* ��Ȩ����(C)
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
* <����> | <�汾> | <����> | <����>
*
* xxxx/xx/xx | 1.0.0 | HWW | �����ļ�
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
const uint8_t  ADC_RG_SQUE[ADC_RQ_CH_NUM] = ADC_RG_CH_LIST;             //������ת������
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

    ADC3->CR2 &= ~ ( 1 << ADC_CR2_ADON_Pos ); //ADC3�ر�
    ADC1->CR2 &= ~ ( 1 << ADC_CR2_ADON_Pos ); //ADC1�ر�

    /*ADC configuration */
    ADC3->CR1  =      0 << ADC_CR1_OVRIE_Pos   | //����ж�,��ֹ
                      0 << ADC_CR1_RES_Pos     | //�ֱ���,12λ
                      0 << ADC_CR1_AWDCH_Pos   | //������ģ�⿴�Ź�,��ֹ
                      0 << ADC_CR1_JAWDEN_Pos  | //ע����ģ�⿴�Ź�,��ֹ
                      0 << ADC_CR1_JDISCEN_Pos | //ע��ͨ�����ģʽ,��ֹ
                      0 << ADC_CR1_DISCEN_Pos  | //����ͨ�����ģʽ,��ֹ
                      0 << ADC_CR1_JAUTO_Pos   | //ע�����Զ�ת��,��ֹ
                      1 << ADC_CR1_SCAN_Pos    | //ɨ��ģʽ,ʹ��
                      0 << ADC_CR1_JEOCIE_Pos  | //ע����ת������ж�,ʹ��
                      0 << ADC_CR1_AWDIE_Pos   | //ģ�⿴�Ź��ж�,��ֹ
                      0 << ADC_CR1_EOCIE_Pos   ; //������ת������ж�,��ֹ

    ADC3->CR2 =       1 << ADC_CR2_SWSTART_Pos | //������ת��ʹ��,ʹ��
                      10 << ADC_CR2_EXTEN_Pos   | //�������ⲿ����,ʹ��,�½���
                      8 << ADC_CR2_EXTSEL_Pos  | //�������ⲿ�����¼�,��ʱ��3�¼�
                      0 << ADC_CR2_JSWSTART_Pos | //ע����ת��ʹ��,��ֹ
                      0 << ADC_CR2_JEXTEN_Pos  | //ע�����ⲿ����,ʹ������������Ч
                      0 << ADC_CR2_JEXTSEL_Pos | //ע�����ⲿ�����¼�,��ʱ��1 TRGO
                      0 << ADC_CR2_ALIGN_Pos   | //���ݶ���,�Ҷ���
                      1 << ADC_CR2_DDS_Pos     | //����DMA����(���ڵ���ADCģʽ),ʹ��
                      1 << ADC_CR2_DMA_Pos     | //ʹ��DMA����,ʹ��
                      0 << ADC_CR2_CONT_Pos    | //����ת��ģʽ(���ڵ���ADCģʽ),��ֹ
                      0 << ADC_CR2_ADON_Pos    ; //ADC����,�ر�

    ADC123_COMMON ->CCR   = 0 << ADC_CCR_ADCPRE_Pos | //ADCԤ��Ƶ,2��Ƶ(TadcclkƵ�� = APB2/2=84M/2=42M)
                            1 << ADC_CCR_DMA_Pos    | //DMAģʽ(���ڶ���ADCģʽ),ʹ��
                            0 << ADC_CCR_DDS_Pos    | //DMA��������(���ڶ���ADCģʽ),ʹ��
                            0 << ADC_CCR_DELAY_Pos  | //2�������׶μ��ӳ�(���ڶ���ADCģʽ),5*Tadcclk
                            0 << ADC_CCR_MULTI_Pos  ; //����ADCģʽѡ��,˫�ؽ�ע��ͬʱģʽ
    //����ͨ������
    ADC3->SQR1      = ( ADC_RQ_CH_NUM - ADC_CH_0_Stop_V - 1 ) << 20;     //����ͨ��������Ŀ 0��ʾ1��7��ʾ8   13-9-1
    ADC3->SQR2      = 0;                                //��������
    ADC3->SQR3      = 0;

    for ( i = 0; i < ( ADC_RQ_CH_NUM - ADC_CH_0_Stop_V ); i++ )           //����ͨ������---4
    {
        if ( i < 6 )
        {
            ADC3->SQR3 |= ADC_RG_SQUE[i + ADC_CH_0_Stop_V] << ( ( i ) * 5 );//5��SQ1,6��SQ2,7��SQ3,8��SQ4
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
    ADC1->CR1  =      0 << ADC_CR1_OVRIE_Pos   | //����ж�,��ֹ
                      0 << ADC_CR1_RES_Pos     | //�ֱ���,12λ
                      0 << ADC_CR1_AWDCH_Pos   | //������ģ�⿴�Ź�,��ֹ
                      0 << ADC_CR1_JAWDEN_Pos  | //ע����ģ�⿴�Ź�,��ֹ
                      0 << ADC_CR1_JDISCEN_Pos | //ע��ͨ�����ģʽ,��ֹ
                      0 << ADC_CR1_DISCEN_Pos  | //����ͨ�����ģʽ,��ֹ
                      0 << ADC_CR1_JAUTO_Pos   | //ע�����Զ�ת��,��ֹ
                      1 << ADC_CR1_SCAN_Pos    | //ɨ��ģʽ,ʹ��
                      0 << ADC_CR1_JEOCIE_Pos  | //ע����ת������ж�,ʹ��
                      0 << ADC_CR1_AWDIE_Pos   | //ģ�⿴�Ź��ж�,��ֹ
                      0 << ADC_CR1_EOCIE_Pos   ; //������ת������ж�,��ֹ

    ADC1->CR2 =       1 << ADC_CR2_SWSTART_Pos | //������ת��ʹ��,ʹ��
                      10 << ADC_CR2_EXTEN_Pos   | //�������ⲿ����,ʹ��,�½���
                      8 << ADC_CR2_EXTSEL_Pos  | //�������ⲿ�����¼�,��ʱ��3�¼�
                      0 << ADC_CR2_JSWSTART_Pos | //ע����ת��ʹ��,��ֹ
                      0 << ADC_CR2_JEXTEN_Pos  | //ע�����ⲿ����,ʹ������������Ч
                      0 << ADC_CR2_JEXTSEL_Pos | //ע�����ⲿ�����¼�,��ʱ��1 TRGO
                      0 << ADC_CR2_ALIGN_Pos   | //���ݶ���,�Ҷ���
                      1 << ADC_CR2_DDS_Pos     | //����DMA����(���ڵ���ADCģʽ),ʹ��
                      1 << ADC_CR2_DMA_Pos     | //ʹ��DMA����,ʹ��
                      0 << ADC_CR2_CONT_Pos    | //����ת��ģʽ(���ڵ���ADCģʽ),��ֹ
                      0 << ADC_CR2_ADON_Pos    ; //ADC����,�ر�

    ADC123_COMMON ->CCR   = 0 << ADC_CCR_ADCPRE_Pos | //ADCԤ��Ƶ,2��Ƶ(TadcclkƵ�� = APB2/2=84M/2=42M)
                            1 << ADC_CCR_DMA_Pos    | //DMAģʽ(���ڶ���ADCģʽ),ʹ��
                            0 << ADC_CCR_DDS_Pos    | //DMA��������(���ڶ���ADCģʽ),ʹ��
                            0 << ADC_CCR_DELAY_Pos  | //2�������׶μ��ӳ�(���ڶ���ADCģʽ),5*Tadcclk
                            0 << ADC_CCR_MULTI_Pos  ; //����ADCģʽѡ��,˫�ؽ�ע��ͬʱģʽ
    //����ͨ������
    ADC1->SQR1      = ( ADC_CH_0_Stop_V - 1 ) << 20;       //����ͨ��������Ŀ 0��ʾ1��7��ʾ8 --9-1
    ADC1->SQR2      = 0;                                //��������
    ADC1->SQR3      = 0;

    for ( i = 0; i < ADC_CH_0_Stop_V; i++ )               //����ͨ������--9
    {
        if ( i < 6 )
        {
            ADC1->SQR3 |= ADC_RG_SQUE[i] << ( i * 5 );//    10��SQ1,12��SQ2,13��SQ3,14��SQ4,15��SQ5,8��SQ6,
        }
        else if ( i < 12 )
        {
            ADC1->SQR2 |= ADC_RG_SQUE[i] << ( ( i - 6 ) * 5 );//9��SQ7,4��SQ8,5��SQ9
        }
        else if ( i < 16 )
        {
            ADC1->SQR1 |= ADC_RG_SQUE[i] << ( ( i - 12 ) * 5 );
        }
    }

    DMA_Init();

    //�ж�����
    HAL_NVIC_SetPriority ( ADC_IRQn, IRQ_Priority_ADC_Irq, 0 );
    HAL_NVIC_EnableIRQ ( ADC_IRQn );

    ADC3->CR2 |= 1 << ADC_CR2_ADON_Pos; //ADC����
    ADC1->CR2 |= 1 << ADC_CR2_ADON_Pos; //ADC����
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
    //ͨ���½��غ�Լ5uSִ�е��˴�
    ADC_DMA_IRQ_CLEAR();
//    ADC3 ->SR &= ~ADC_SR_OVR_Msk;
//    ADC1 ->SR &= ~ADC_SR_OVR_Msk;

}
void ADC_DMA_IRQ_Handler_2 ( void )
{
    //ͨ���½��غ�Լ5uSִ�е��˴�
    ADC_DMA_IRQ_CLEAR_2();
//    ADC3 ->SR &= ~ADC_SR_OVR_Msk;
//    ADC1 ->SR &= ~ADC_SR_OVR_Msk;

}

void ADC_SWStart ( void )
{
    ADC3->CR2 |= ADC_CR2_SWSTART_Msk;//�������������Timer7��֧��������Դ��
    ADC1->CR2 |= ADC_CR2_SWSTART_Msk;//�������������Timer7��֧��������Դ��
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

    //������ ADC GPIO ����
    GPIO_InitStruct.Mode      = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

    for ( i = 0; i < ADC_RQ_CH_NUM; i++ )
    {
        GPIO_InitStruct.Pin = ADC_Pin_List[i];
        HAL_GPIO_Init ( ( GPIO_TypeDef* ) ADC_Port_List[i], &GPIO_InitStruct );
    }

    //ADCʱ��ʹ��
    __HAL_RCC_ADC3_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    //ADCͨ������ʱ��
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

    ADC_DMA_STREAM->M0AR = ( uint32_t ) ( g_u16AdcData + ADC_CH_MOTOR2_24V_V + 1 ); //�洢����ַ
    ADC_DMA_STREAM->PAR = ( uint32_t ) &ADC3->DR; //�����ַ
    ADC_DMA_STREAM->NDTR = ADC_CH_MOTOR2_24V_I - ADC_CH_MOTOR2_24V_V;        //���ݴ�������
    ADC_DMA_STREAM->FCR =   0x00 << DMA_SxFCR_FEIE_Pos    //FIFO�����ж�,��ֹ
                            | 0x00 << DMA_SxFCR_DMDIS_Pos; //ֱ��ģʽ,ʹ��


    HAL_NVIC_SetPriority ( ADC_DMA_IRQ, IRQ_Priority_ADC_DMA, 0 );
    HAL_NVIC_EnableIRQ ( ADC_DMA_IRQ );

    ADC_DMA_STREAM->CR |=  0x01; //ʹ��ͨ��

    ADC_DMA_STREAM_2->CR = 0;
    ADC_DMA_STREAM_2->CR =           ADC_DMA_CHANNEL_2           //ͨ��
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

    ADC_DMA_STREAM_2->M0AR = ( uint32_t ) ( g_u16AdcData ); //�洢����ַ
    ADC_DMA_STREAM_2->PAR = ( uint32_t ) &ADC1->DR; //�����ַ
    ADC_DMA_STREAM_2->NDTR = ADC_CH_MOTOR2_24V_V + 1;      //���ݴ�������
    ADC_DMA_STREAM_2->FCR =   0x00 << DMA_SxFCR_FEIE_Pos    //FIFO�����ж�,��ֹ
                              | 0x00 << DMA_SxFCR_DMDIS_Pos; //ֱ��ģʽ,ʹ��
    HAL_NVIC_SetPriority ( ADC_DMA_IRQ_2, IRQ_Priority_ADC_DMA, 0 );
    HAL_NVIC_EnableIRQ ( ADC_DMA_IRQ_2 );
    ADC_DMA_STREAM_2->CR |=  0x01; //ʹ��ͨ��
}
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


