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
    case 0:  //can报文状态机切换
    {
        HostState_Ctrl ( m_eHostComState );
    }
    break;

    case 1:  //不同运行模式下转向指令控制逻辑
    {
        MotorCmd_Ctrl ( &RegReal_PosCtrlCmd, &g_sMotorCmd );
    }
    break;

    case 2:  //传感器标定（延时启动，保证复位标定执行完成）
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
 * @brief       adc中断回调函数（注入方式），20KHZ处理
 * @param       无
 * @retval      无
 */
void ADC_CurrentSampleCpl_Callback ( void )       //20KHZ
{
    static uint32_t g_u32CurrentTick = 0;

    g_u32CurrentTick++;

    if ( g_u32CurrentTick % ( 20000 / Freq_4K ) == 0 )  //4KHZ
    {
        MotorInfo_EAngleCalc ( &g_sMotor_Info, Freq_4K );  //电角度计算
    }
    if ( g_u32CurrentTick % ( 20000 / Freq_4K ) == 1 )  //4KHZ
    {
        MotorInfo_OutSpdCalc ( &g_sMotor_Info, Freq_4K );               //计算出轴速度
    }
    if ( g_u32CurrentTick % ( 20000 / Freq_500 ) == 3 )  //500HZ
    {
        MotorInfo_PosAbsCalc ( &g_sMotor_Info, Freq_500 );              //计算后轮码盘位置
//        Encoder_PosClc( &Encoder_Handle, sTeles_Ctrl.PosCali_OK );
        TrigFunc_Clc ( &TrigFunc_Handle, sTeles_Ctrl.PosCali_OK );
    }
    MotorInfo_GetCurrent ( &g_sMotor_Info, Freq_20K );  //相电流获取

    if ( ( Foc_EN_Flag == 1 ) && ( u8Standby_Flag == 1 ) ) FOC_Core ( &g_sMotor_Info, &g_sMotorCmd );
}
//-------------------- private functions ----------------------------

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
