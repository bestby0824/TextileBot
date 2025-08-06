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
* @brief       IO���ƺ���
* @param       ��
* @retval      ��
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
* @brief       ��Դ���ƺ���
* @param       ��
* @retval      ��
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
* @brief       ���ߴ���ģ�����ú���
* @param       ��
* @retval      ��
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
* @brief      ת��ƿ��ƺ���
* @param       ��
* @retval      ��
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
* @brief      beep���ƺ���
* @param       ��
* @retval      ��
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
* @brief      beep��˸���ƺ���
* @param       ��
* @retval      ��
*
*/
void Buzzer_Beep ( uint32_t on_time, uint32_t off_time )
{
    // �򿪷�����
    BeepCtrl ( ON );
    Delay_mS ( on_time );

    // �رշ�����
    BeepCtrl ( OFF );
    Delay_mS ( off_time );
}
/*
* @brief      ��ɫ�ƿ��ƺ���
* @param       ��
* @retval      ��
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
* @brief       0�༱ͣ
* @param       ��
* @retval      ��
*
*/
void STOP_0_Ctrl ( SWSta Sta )  //ON����򿪼�ͣ����ͣ��Ч
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
* @brief       ��λ����Դ���ƺ���
* @param       ��
* @retval      ��
*
*/
void DoSetPCPower ( SWSta Sta )
{
    PowerCtrl_SW ( DO_CH_POWER_SW4, Sta );
}
/*
* @brief       Motor2��Դ���ƺ���
* @param       ��
* @retval      ��
*
*/
void DoSetMotor2Power ( SWSta Sta )
{
    PowerCtrl_SW ( DO_CH_POWER_SW3, Sta );
}
/*
* @brief       Motor1��Դ���ƺ���
* @param       ��
* @retval      ��
*
*/
void DoSetMotor1Power ( SWSta Sta )
{
    PowerCtrl_SW ( DO_CH_POWER_SW2, Sta );
}
/*
* @brief       ���������״��Դ���ƺ���
* @param       ��
* @retval      ��
*
*/
void DoSetSensorPower ( SWSta Sta )
{
    PowerCtrl_SW ( DO_CH_POWER_SW1, Sta );
}
/*
* @brief       �������Դ���ƺ���
* @param       //��������أ�ON�ر�,OFF��
* @retval      ��
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
* @brief       ��Դ���ƺ���
* @param       ��
* @retval      ��
*
*/
void PowerCtrl( PowerOptDef PowerOpt, SWSta PowerSta)
{

    switch(PowerOpt)
    {
        case PCPower:
        {
             DoSetPCPower ( PowerSta ); //��λ����Դ
        }
        break;
        case Motor1Power:
        {
             DoSetMotor1Power ( PowerSta ); //Motor1��Դ���͹ر�
        }
        break;
        case Motor2Power:
        {
            DoSetMotor2Power ( PowerSta ); //Motor2��Դ���͹ر�
        }
        break;
        case SensorPower:
        {
            DoSetSensorPower ( PowerSta ); //��������Դ���͹ر�
        }
        break;
        case AllPower:
        {
            //DoSetPCPower ( PowerSta ); //��λ����Դ
            DoSetMotor1Power ( PowerSta ); //Motor1��Դ���͹ر�
            DoSetMotor2Power ( PowerSta ); //Motor1��Դ���͹ر�
            DoSetSensorPower ( PowerSta ); //��������Դ���͹ر�
        }
        break;
    }
   
    
   
    
}
/*
* @brief       IO���Ƴ�ʼ������
* @param       ��
* @retval      ��
*
*/
void IOCtrlInit ( void )
{
    PowerCtrl( AllPower, OFF);
    DoSetMagnetPower ( OFF );//�����
    TurnSignalCtrl ( Sta_N ); //ת��ƣ��͹ر�
    BeepCtrl ( OFF ); //���������͹ر�
    TriColorLedCtrl ( Led_OFF ); //��ɫ�ƣ��͹ر�
    STOP_0_Ctrl ( OFF );//
    WLCfg ( ON );//����ģ�飬����������
}

/*textile*/

void DidoProcessInit ( void )
{
		Dido_Init();
		PowerCtrl(PCPower,ON);
//    DoSetPower_SW1 ( ON );
//    DoSetPower_SW2 ( ON );
//    DoSetPower_SW3 ( ON );//
//    DoSetTAG_LED1_ON ( OFF );//������ά���Դ
//    DoSetTAG_LED2_ON ( OFF );//������ά���Դ
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
* @brief      ת��ƿ��ƺ���
* @param       ��
* @retval      ��
*
*/
void DoSetSteeringLampSW ( LightSta Sta, uint32_t on_time, uint32_t off_time )    //ת���
{
    TurnSignalCtrl ( Sta );
    Delay_mS ( on_time );
    TurnSignalCtrl ( Sta_N );
    Delay_mS ( off_time );
}
#if 0
/*
* @brief       Һѹ��Դ����
* @param       ��
* @retval      ��
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
* @brief       ת���Դ����
* @param       ��
* @retval      ��
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
* @brief       ��λ����Դ����
* @param       ��
* @retval      ��
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
* @brief       ������ά���Դ
* @param       ��
* @retval      ��
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
* @brief       ���õ�Դ����
* @param       ��
* @retval      ��
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
* @brief       0�༱ͣ
* @param       ��
* @retval      ��
*
*/
int8_t DoSet0_STOP_CTRL_ON ( SWSta Sta )  //ON����򿪼�ͣ����ͣ��Ч
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
* @brief       2�༱ͣ��û�е��ã�
* @param       ��
* @retval      ��
*
*/
int8_t DoSet2_STOP_CTRL_ON ( SWSta Sta )    //ON����򿪼�ͣ����ͣ��Ч
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
 * @brief       ��ѯ���IO�ڣ��ж��泵����ģʽ
 * @param       ��
 * @retval      ��
 */
void CtrlModeFeedback ( void )
{
    GPIO_PinState Flg_PC_Ctrl, Flg_RC_Ctrl, Flg_HAND_Ctrl;

//    DI_ReadByIndex ( &Flg_PC_Ctrl, DI_CH_PC_MODE );
//    DI_ReadByIndex ( &Flg_RC_Ctrl, DI_CH_RC_MODE );
//    DI_ReadByIndex ( &Flg_HAND_Ctrl, DI_CH_HAND_MODE );//

//    if ( Flg_PC_Ctrl == GPIO_PIN_SET )
//    {
//        m_eCtrlMode = PC_MODE;//��λ��
//    }
//    else if ( Flg_RC_Ctrl == GPIO_PIN_RESET )
//    {
//        m_eCtrlMode = RC_MODE;//ң��
//    } else if ( Flg_HAND_Ctrl == GPIO_PIN_SET )
//    {
//        m_eCtrlMode = HAND_MODE;//�ֶ�
//    }

//    else
//    {
//        NULL;
//    }
}
/**
 * @brief       ��ȡ�泵��ǰ����ģʽ��ң�ء��Զ���ң�أ�
 * @param       ��
 * @retval      ��ǰ����ģʽ
 */
CtrlMode GetCtrlMode ( void )
{
    return m_eCtrlMode;
}
/**
 * @brief       �������ϵ縴λ������¼�ڶ�Ӧflash��
 * @param       ��
 * @retval      ��
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
 * @brief       �µ����flash������ϵ������¼����
 * @param       ��
 * @retval      ��
 */
void NRST_Erase ( void )
{
    static uint8_t u8EraseCnt = 0, u8EraseRes = 1;
    static uint16_t u16CurVol = 4096;
    static uint16_t u16ForkVolMaxLim = 1221;  //0.6����Fork_24V
    static uint16_t u16ForkVolMinLim = 407;  //0.2����Fork_24V

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


