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
#include "DidoProcess.h"
#include "ErrorCode.h"
#include "StFlash.h"
#include "DataBaseProcess.h"
#include "AdcProcess.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
ShiftSta g_sCurShiftSta;
CtrlMode m_eCtrlMode = Idle_MODE;
//-------------------- private functions declare ----------------------------

//-------------------- public data ------------------------------------------
uint16_t u16NRSTimes = 0;
uint16_t LastMode = Idle_MODE;

//-------------------- public functions -------------------------------------
RegDef_InputIO InputIOInfor;
LightSta Reg_LightSta;
TriColorSta Reg_TriColorSta;
E_DO_CH Reg_E_DO_CH;
SWSta Reg_SWSta;
/*textile*/
/*
* @brief       IO控制函数
* @param       无
* @retval      无
*
*/

void UpdateIOSta ( void )
{
    DI_ReadByIndex ( &InputIOInfor.PC_5VSta, DI_CH_PC_5V );
    DI_ReadByIndex ( &InputIOInfor.STOP_0Sta, DI_CH_0_STOP );
    DI_ReadByIndex ( &InputIOInfor.PowerSWSta, DI_CH_POWERSWSTA );
    DI_ReadByIndex ( &InputIOInfor.RST_MCUSta, DI_CH_RST_MCU );
    DI_ReadByIndex ( &InputIOInfor.CtrlModeOpt, DI_CH_PC_MODE );
    DI_ReadByIndex ( &InputIOInfor.TouchEdgeSta, DI_CH_TouchEdge );
}
/*
* @brief       电源控制函数
* @param       无
* @retval      无
*
*/

void PowerCtrl_SW ( E_DO_CH PowerOpt, SWSta Sta )
{
    switch ( Sta )
    {
    case OFF:
    {
        DO_WriteByIndex ( GPIO_PIN_RESET, PowerOpt );
    }
    break;
    case ON:
    {
        DO_WriteByIndex ( GPIO_PIN_SET, PowerOpt );
    }
    break;
    }
}

/*
* @brief       无线串口模块配置函数
* @param       无
* @retval      无
*
*/
void WLCfg ( SWSta Sta )
{
    switch ( Sta )
    {
    case OFF:
    {
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_WL_RST );
    }
    break;
    case ON:
    {
        DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_WL_RST );
    }
    break;
    }
}
/*
* @brief      转向灯控制函数
* @param       无
* @retval      无
*
*/
void TurnSignalCtrl ( LightSta Sta )
{
    switch ( Sta )
    {
    case Sta_N:
    {
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_LEFT );
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_RIGHT );
    }
    break;
    case Sta_L:
    {
        DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_LED_LEFT );
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_RIGHT );
        //DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_LEFT );
    }
    break;
    case Sta_R:
    {
        DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_LED_RIGHT );
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_LEFT );
        //DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_RIGHT );
    }
    break;
    case Sta_A:
    {
        DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_LED_RIGHT );
        DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_LED_LEFT );
        //DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_RIGHT );
    }
    break;
    default:
    {
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_LEFT );
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_RIGHT );
    }
    break;
    }
}

/*
* @brief      beep控制函数
* @param       无
* @retval      无
*
*/
void BeepCtrl ( SWSta Sta )
{
    switch ( Sta )
    {
    case OFF:
    {
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_BEEP_SE );
    }
    break;
    case ON:
    {
        DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_BEEP_SE );
    }
    break;
    }
}
/*
* @brief      beep闪烁控制函数
* @param       无
* @retval      无
*
*/
void Buzzer_Beep ( uint32_t on_time, uint32_t off_time )
{
    // 打开蜂鸣器
    BeepCtrl ( ON );
    Delay_mS ( on_time );

    // 关闭蜂鸣器
    BeepCtrl ( OFF );
    Delay_mS ( off_time );
}
/*
* @brief      三色灯控制函数
* @param       无
* @retval      无
*
*/
void TriColorLedCtrl ( TriColorSta Sta )
{
    switch ( Sta )
    {
    case Led_OFF:
    {
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_R );
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_G );
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_Y );
    }
    break;
    case Led_R:
    {
        DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_LED_R );
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_G );
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_Y );
    }
    break;
    case Led_G:
    {
        DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_LED_G );
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_R );
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_Y );
    }
    break;
    case Led_Y:
    {
        DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_LED_Y );
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_R );
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_G );
    }
    break;
    default:
    {
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_R );
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_G );
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED_Y );
    }
    break;
    }
}
/*
* @brief       0类急停
* @param       无
* @retval      无
*
*/
void STOP_0_Ctrl ( SWSta Sta )  //ON代表打开急停，急停生效
{
    switch ( Sta )
    {
    case OFF:
    {
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_STOP0 );
    }
    break;
    case ON:
    {
        DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_STOP0 );
    }
    break;

    }
}
/*
* @brief       上位机电源控制函数
* @param       无
* @retval      无
*
*/
void DoSetPCPower ( SWSta Sta )
{
    PowerCtrl_SW ( DO_CH_POWER_SW4, Sta );
}
/*
* @brief       Motor2电源控制函数
* @param       无
* @retval      无
*
*/
void DoSetMotor2Power ( SWSta Sta )
{
    PowerCtrl_SW ( DO_CH_POWER_SW3, Sta );
}
/*
* @brief       Motor1电源控制函数
* @param       无
* @retval      无
*
*/
void DoSetMotor1Power ( SWSta Sta )
{
    PowerCtrl_SW ( DO_CH_POWER_SW2, Sta );
}
/*
* @brief       传感器（雷达）电源控制函数
* @param       无
* @retval      无
*
*/
void DoSetSensorPower ( SWSta Sta )
{
    PowerCtrl_SW ( DO_CH_POWER_SW1, Sta );
}
/*
* @brief       电磁铁电源控制函数
* @param       //电磁铁开关，ON关闭,OFF开
* @retval      无
*
*/
void DoSetMagnetPower ( SWSta Sta )
{
    switch ( Sta )
    {
    {
    case OFF:
        PowerCtrl_SW ( DO_CH_MAGNET_ON, ON );
    }
    break;
    case ON:
    {
        PowerCtrl_SW ( DO_CH_MAGNET_ON, OFF );
    }
    break;
    }
}
/*
* @brief       电源控制函数
* @param       无
* @retval      无
*
*/
void PowerCtrl( PowerOptDef PowerOpt, SWSta PowerSta)
{

    switch(PowerOpt)
    {
        case PCPower:
        {
             DoSetPCPower ( PowerSta ); //上位机电源
        }
        break;
        case Motor1Power:
        {
             DoSetMotor1Power ( PowerSta ); //Motor1电源，低关闭
        }
        break;
        case Motor2Power:
        {
            DoSetMotor2Power ( PowerSta ); //Motor2电源，低关闭
        }
        break;
        case SensorPower:
        {
            DoSetSensorPower ( PowerSta ); //传感器电源，低关闭
        }
        break;
        case AllPower:
        {
            //DoSetPCPower ( PowerSta ); //上位机电源
            DoSetMotor1Power ( PowerSta ); //Motor1电源，低关闭
            DoSetMotor2Power ( PowerSta ); //Motor1电源，低关闭
            DoSetSensorPower ( PowerSta ); //传感器电源，低关闭
        }
        break;
    }
   
    
   
    
}
/*
* @brief       IO控制初始化函数
* @param       无
* @retval      无
*
*/
void IOCtrlInit ( void )
{
    PowerCtrl( AllPower, OFF);
    DoSetMagnetPower ( OFF );//电磁铁
    TurnSignalCtrl ( Sta_N ); //转向灯，低关闭
    BeepCtrl ( OFF ); //蜂鸣器，低关闭
    TriColorLedCtrl ( Led_OFF ); //三色灯，低关闭
    STOP_0_Ctrl ( OFF );//
    WLCfg ( ON );//无线模块，配置需拉低
}

/*textile*/

void DidoProcessInit ( void )
{
		Dido_Init();
		PowerCtrl(PCPower,ON);
//    DoSetPower_SW1 ( ON );
//    DoSetPower_SW2 ( ON );
//    DoSetPower_SW3 ( ON );//
//    DoSetTAG_LED1_ON ( OFF );//车顶二维码电源
//    DoSetTAG_LED2_ON ( OFF );//车顶二维码电源
    IOCtrlInit ();
}
ShiftSta DoGetFBSW()
{
    return g_sCurShiftSta;
}


int8_t DoSetFBSW ( ShiftSta Sta )
{
    switch ( Sta )
    {
    case Shift_N:
    {
    }
    break;
    case Shift_D:
    {

    }
    break;
    case Shift_R:
    {


    }
    break;
    }
    g_sCurShiftSta = Sta;
    return Err_None;
}
/*
* @brief      转向灯控制函数
* @param       无
* @retval      无
*
*/
void DoSetSteeringLampSW ( LightSta Sta, uint32_t on_time, uint32_t off_time )    //转向灯
{
    TurnSignalCtrl ( Sta );
    Delay_mS ( on_time );
    TurnSignalCtrl ( Sta_N );
    Delay_mS ( off_time );
}
#if 0
/*
* @brief       液压电源函数
* @param       无
* @retval      无
*
*/
int8_t DoSetPower_SW1 ( SWSta Sta )
{
    switch ( Sta )
    {
    case OFF:
    {
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_Power1 );
    }
    break;
    case ON:
    {
        DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_Power1 );
    }
    break;

    }
    return Err_None;
}
/*
* @brief       转向电源函数
* @param       无
* @retval      无
*
*/
int8_t DoSetPower_SW2 ( SWSta Sta )
{
    switch ( Sta )
    {
    case OFF:
    {
//            DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_Power2 );
    }
    break;
    case ON:
    {
//            DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_Power2 );
    }
    break;

    }
    return Err_None;
}

/*
* @brief       上位机电源函数
* @param       无
* @retval      无
*
*/
int8_t DoSetPower_SW3 ( SWSta Sta )
{
    switch ( Sta )
    {
    case OFF:
    {
        PowerCtrl_SW ( DO_CH_POWER_SW4, OFF );
    }
    break;
    case ON:
    {
        DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_POWER_SW3 );
    }
    break;

    }
    return Err_None;
}

/*
* @brief       车顶二维码电源
* @param       无
* @retval      无
*
*/
int8_t DoSetTAG_LED1_ON ( SWSta Sta )
{
    switch ( Sta )
    {
    case OFF:
    {
        //DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_TAG1_LED );
    }
    break;
    case ON:
    {
        // DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_TAG1_LED );
    }
    break;

    }
    return Err_None;
}
/*
* @brief       备用电源函数
* @param       无
* @retval      无
*
*/
int8_t DoSetTAG_LED2_ON ( SWSta Sta )
{
    switch ( Sta )
    {
    case OFF:
    {
        //DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_TAG2_LED );
    }
    break;
    case ON:
    {
        //DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_TAG2_LED );
    }
    break;

    }
    return Err_None;
}
/*
* @brief       0类急停
* @param       无
* @retval      无
*
*/
int8_t DoSet0_STOP_CTRL_ON ( SWSta Sta )  //ON代表打开急停，急停生效
{
    switch ( Sta )
    {
    case OFF:
    {
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_STOP0 );
    }
    break;
    case ON:
    {
        DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_STOP0 );
    }
    break;

    }
    return Err_None;
}
/*
* @brief       2类急停（没有调用）
* @param       无
* @retval      无
*
*/
int8_t DoSet2_STOP_CTRL_ON ( SWSta Sta )    //ON代表打开急停，急停生效
{
    switch ( Sta )
    {
    case OFF:
    {
        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_STOP2 );
    }
    break;
    case ON:
    {
        DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_STOP2 );
    }
    break;

    }
    return Err_None;
}
int8_t DoSeWhistle ( SWSta Sta )
{
    switch ( Sta )
    {
    case OFF:
    {

    }
    break;
    case ON:
    {

    }
    break;

    }
    return Err_None;
}


int8_t DoSetLED5 ( SWSta Sta )
{
    switch ( Sta )
    {
    case OFF:
    {
        //        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED5 );
    }
    break;
    case ON:
    {
        //        DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_LED5 );
    }
    break;

    }
    return Err_None;
}
int8_t DoSetLED6 ( SWSta Sta )
{
    switch ( Sta )
    {
    case OFF:
    {
        //        DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED6 );
    }
    break;
    case ON:
    {
        //        DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_LED6 );
    }
    break;

    }
    return Err_None;
}
int8_t DoSetRadar ( uint16_t *StaL, uint16_t *StaR )
{
    return Err_None;
}

/**
 * @brief       轮询检测IO口，判定叉车控制模式
 * @param       无
 * @retval      无
 */
void CtrlModeFeedback ( void )
{
    GPIO_PinState Flg_PC_Ctrl, Flg_RC_Ctrl, Flg_HAND_Ctrl;

//    DI_ReadByIndex ( &Flg_PC_Ctrl, DI_CH_PC_MODE );
//    DI_ReadByIndex ( &Flg_RC_Ctrl, DI_CH_RC_MODE );
//    DI_ReadByIndex ( &Flg_HAND_Ctrl, DI_CH_HAND_MODE );//

//    if ( Flg_PC_Ctrl == GPIO_PIN_SET )
//    {
//        m_eCtrlMode = PC_MODE;//上位机
//    }
//    else if ( Flg_RC_Ctrl == GPIO_PIN_RESET )
//    {
//        m_eCtrlMode = RC_MODE;//遥控
//    } else if ( Flg_HAND_Ctrl == GPIO_PIN_SET )
//    {
//        m_eCtrlMode = HAND_MODE;//手动
//    }

//    else
//    {
//        NULL;
//    }
}
/**
 * @brief       获取叉车当前控制模式（遥控、自动、遥控）
 * @param       无
 * @retval      当前控制模式
 */
CtrlMode GetCtrlMode ( void )
{
    return m_eCtrlMode;
}
/**
 * @brief       将本次上电复位次数记录在对应flash中
 * @param       无
 * @retval      无
 */
void Set_NRSTIMES ( void )
{
    static uint8_t u8WriteCnt = 0, u8flaerr = 0;
    static uint16_t u16Addr = 0;

    for ( u16Addr = 0; u16Addr < 1024; u16Addr++ )
    {
        if ( ( * ( uint16_t* ) ( ADDRESS_NRSTIMES + u16Addr * 2 ) ) == 0xFFFF )
        {
            u16NRSTimes = ( * ( uint16_t* ) ( ADDRESS_NRSTIMES + ( ( u16Addr > 0 ) ? ( u16Addr - 1 ) : u16Addr ) * 2 ) );
            break;
        }
    }

    u8WriteCnt = 3;

    if ( u16NRSTimes == 0xFFFF ) u16NRSTimes = 0;
    else u16NRSTimes += 1;

    u8flaerr = Flash_HalfWordWrite ( ( ADDRESS_NRSTIMES + u16NRSTimes * 2 ), &u16NRSTimes, 1 );
    while ( u8flaerr && u8WriteCnt )
    {
        u8WriteCnt--;
        u8flaerr = Flash_HalfWordWrite ( ( ADDRESS_NRSTIMES + u16NRSTimes * 2 ), &u16NRSTimes, 1 );
    }
}
/**
 * @brief       下电擦除flash中相关上电次数记录数据
 * @param       无
 * @retval      无
 */
void NRST_Erase ( void )
{
    static uint8_t u8EraseCnt = 0, u8EraseRes = 1;
    static uint16_t u16CurVol = 4096;
    static uint16_t u16ForkVolMaxLim = 1221;  //0.6倍的Fork_24V
    static uint16_t u16ForkVolMinLim = 407;  //0.2倍的Fork_24V

    u16CurVol = Get_FORK_24V ( );
    if ( ( u16CurVol < u16ForkVolMaxLim ) && ( u16CurVol > u16ForkVolMinLim ) && u8EraseRes )
    {
        u8EraseCnt = 5;
        if ( 0 == ( Flash_Erase ( ADDRESS_NRSTIMES, 1 ) && u8EraseCnt-- ) )
        {
            u8EraseCnt --;
            u8EraseRes = 0;
        }
    }
}
#endif
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


