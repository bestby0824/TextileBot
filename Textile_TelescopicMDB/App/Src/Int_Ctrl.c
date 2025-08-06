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
#include "Int_Ctrl.h"
#include "TimerBase.h"
#include "ADC.h"
#include "main.h"
#include "Can_Host.h"
#include "HostProcess.h"
#include "Uart.h"
#include "HostState_Ctrl.h"
#include "VVVF.h"
#include "SensorCali_Ctrl.h"
#include "Foc.h"
#include "MotorInfo.h"
#include "MotorCmd_Ctrl.h"
#include "Monitor.h"
#include "DataBaseProcess.h"
#include "RelayCmd_Ctrl.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
uint8_t DebugTXBuf[8];

//-------------------- private functions declare ----------------------------

//-------------------- public data ------------------------------------------
uint32_t TimeCnt = 0;
//-------------------- public functions -------------------------------------
/**
  * @brief  void TimerBase1_Update_Callback(void)
  * @param  none
  * @retval None
  * @notes  None
  */
void TimerBase1_Update_Callback ( void ) //10KHZ
{
    static uint32_t ticks = 0;

    ticks = TimerBase1_GetTicks();
    switch ( ticks % 10 ) //1kHz
    {
    case 0:  //can����״̬���л�
    {
        HostState_Ctrl ( m_eHostComState );
    }
    break;

    case 1:  //��ͬ����ģʽ��ת��ָ������߼�
    {
        MotorCmd_Ctrl ( &RegReal_PosCtrlCmd, &g_sMotorCmd );
    }
    break;

    case 2:  //�������궨����ʱ��������֤��λ�궨ִ����ɣ�
    {
        if ( 1 == u8Standby_Flag ) SensorCali_Ctrl ( );
    }
    break;

    case 3:
    {
        MotorInfo_Pressure ( &g_sMotor_Info );
        MotorInfo_Switch ( &g_sMotor_Info );
    }
    break;

    case 4:
    {
        FaultClearProcess ( );
    }
    break;

    case 5:
    {
        ParamRead_Write();
    }
    break;

    case 6:
    {
        RelayCmd_Ctrl ( &RegReal_RelayCmd );
    }
    break;

    case 7:
    {

    }
    break;

    case 8:
    {

    }
    break;

    case 9:
    {
        MonitorProcess ( );
    }
    break;

    default:
    {

    }
    break;
    }
}
/**
 * @brief       adc�жϻص�������ע�뷽ʽ����20KHZ����
 * @param       ��
 * @retval      ��
 */
void ADC_CurrentSampleCpl_Callback ( void )       //20KHZ
{
    static uint32_t g_u32CurrentTick = 0;

    g_u32CurrentTick++;

    if ( g_u32CurrentTick % ( 20000 / Freq_4K ) == 0 )  //4KHZ
    {
        MotorInfo_EAngleCalc ( &g_sMotor_Info, Freq_4K );  //��Ƕȼ���
    }
    if ( g_u32CurrentTick % ( 20000 / Freq_4K ) == 1 )  //4KHZ
    {
        MotorInfo_OutSpdCalc ( &g_sMotor_Info, Freq_4K );               //��������ٶ�
    }
    if ( g_u32CurrentTick % ( 20000 / Freq_500 ) == 3 )  //500HZ
    {
        MotorInfo_PosAbsCalc ( &g_sMotor_Info, Freq_500 );              //�����������λ��
//        Encoder_PosClc( &Encoder_Handle, sTeles_Ctrl.PosCali_OK );
        TrigFunc_Clc ( &TrigFunc_Handle, sTeles_Ctrl.PosCali_OK );
    }
    MotorInfo_GetCurrent ( &g_sMotor_Info, Freq_20K );  //�������ȡ

    if ( ( Foc_EN_Flag == 1 ) && ( u8Standby_Flag == 1 ) ) FOC_Core ( &g_sMotor_Info, &g_sMotorCmd );
}
//-------------------- private functions ----------------------------

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
