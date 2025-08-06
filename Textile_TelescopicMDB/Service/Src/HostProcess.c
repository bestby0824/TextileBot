/**
* ��Ȩ����(C)
*
* ********
*
* @file
* @brief
* @details
* @author MUNIU
* @version 1.0.0
* @date xxxx-xx-xx
* @par Description:
* @par Change History:
*
* <����> | <�汾> | <����> | <����>
*
* xxxx/xx/xx | 1.0.0 | MUNIU | �����ļ�
*
*/
//------------------------------------------------------------------------------

//-------------------- include files ----------------------------------------
#include "HostProcess.h"
#include "Can_Host.h"
#include "string.h"
#include "DataBaseProcess.h"
#include "Monitor.h"
#include "HostState_Ctrl.h"
#include "MotorCmd_Ctrl.h"
#include "MotorInfo.h"
#include "ADC.h"
#include "CoderTama.h"
#include "SensorCali_Ctrl.h"
#include "RelayCmd_Ctrl.h"

//-------------------- private functions declare ----------------------------
#define DELAYCNT                   10

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------

//-------------------- private functions declare ----------------------------
static int Can_Host_Send ( uint32_t Cmd_ID );
static void Set_BootFlg ( void );

//-------------------- public data ------------------------------------------
UserFrame_t Tx_frame;
UserFrame_t Rx_frame;
BootRxStruct BootRxHandle;
RegDef_PosCtrlCmd RegReal_PosCtrlCmd = CanRxDefault;  //�Ĵ���0x110
RegDef_SensorCaliCmd RegReal_SensorCaliCmd;           //�Ĵ���0x111
RegDef_FaultClearCmd RegReal_FaultClearCmd;           //�Ĵ���0x112
RegDef_ParamManageCmd RegReal_ParamManageCmd;         //�Ĵ���0x113
RegDef_HeartCmd RegReal_HeartCmd;                     //�Ĵ���0x114
RegDef_ResetCmd RegReal_ResetCmd;                     //�Ĵ���0x115
RegDef_RelayCmd RegReal_RelayCmd;                     //�Ĵ���0x116
RegDef_PosCtrlFB RegReal_PosCtrlFB;                   //�Ĵ���0x200
RegDef_SensorCaliFB RegReal_SensorCaliFB;             //�Ĵ���0x201
RegDef_FaultClearFB RegReal_FaultClearFB;             //�Ĵ���0x202
RegDef_ParamManageFB RegReal_ParamManageFB;           //�Ĵ���0x203
RegDef_HeartFB RegReal_HeartFB;                       //�Ĵ���0x204
RegDef_ResetFB RegReal_ResetFB;                       //�Ĵ���0x205
RegDef_RelayFB RegReal_RelayFB;                       //�Ĵ���0x206
RegDef_FaultCodeFB RegReal_FaultCodeFB;               //�Ĵ���0x211
RegDef_SensorDataFB RegReal_SensorDataFB;             //�Ĵ���0x212
RegDef_BoardState1 RegReal_BoardState1;               //�Ĵ���0x213
RegDef_BoardState2 RegReal_BoardState2;               //�Ĵ���0x214
RegDef_BoardState3 RegReal_BoardState3;               //�Ĵ���0x215

uint16_t g_u16SensorCaliStep = 0, g_u16FultClearStep = 0, g_u16ParamStep = 0, g_u16HeartStep = 0, g_u16ResetStep = 0;
uint16_t u16PosCtrlFB_LastSN = 0, u16PosCtrlFB_DelayCnt = 0, u16RelayFB_LastSN = 0, u16RelayFB_DelayCnt = 0;
//-------------------- public functions -------------------------------------
void Can_Host_Receive ( void );
void CanTaskProcess ( void );
void BootProcess ( void );

/**
 * @brief       Host��Ӧ�Ĵ���״̬���ͣ�20HZ����
 * @param       ��
 * @retval      ��
 */
void CanTaskProcess ( void )
{
    static uint32_t u32CanTaskCnt = 0;
    static uint16_t u16CanTaskStep = 0;

    u32CanTaskCnt++;
    if ( ( u32CanTaskCnt % 10 ) == 0 )  //10msһ�Σ�����ת��ָ��
    {
        u16CanTaskStep = Step_PosCtrlFB;
    }
    else if ( ( u32CanTaskCnt % 10 ) == 2 )  //10msһ�Σ������̵�������ָ��
    {
        u16CanTaskStep = Step_RelayFB;
    }
    else if ( ( u32CanTaskCnt % 100 ) == 1 )  //100msһ�Σ�����������
    {
        u16CanTaskStep = Step_FaultCodeFB;
    }
    else if ( ( u32CanTaskCnt % 100 ) == 3 )  //100msһ�Σ���������������
    {
        u16CanTaskStep = Step_SensorDataFB;
    }
    else if ( ( u32CanTaskCnt % 100 ) == 5 )  //100msһ�Σ�����״̬1
    {
        u16CanTaskStep = Step_State1FB;
    }
    else if ( ( u32CanTaskCnt % 100 ) == 7 )  //100msһ�Σ�����״̬2
    {
        u16CanTaskStep = Step_State2FB;
    }
    else if ( ( u32CanTaskCnt % 100 ) == 9 )  //100msһ�Σ�����״̬3
    {
        u16CanTaskStep = Step_State3FB;
    }
    else if ( g_u16SensorCaliStep == 2 )
    {
        u16CanTaskStep = Step_SensorCaliFB;
        g_u16SensorCaliStep = 0;
    }
    else if ( g_u16FultClearStep == 2 )
    {
        u16CanTaskStep = Step_FaultClearFB;
        g_u16FultClearStep = 0; 
    }
    else if ( g_u16ParamStep == 2 )  //����������ϣ����Իظ�
    {
        u16CanTaskStep = Step_ParamManageFB;
        g_u16ParamStep = 0;
    }
    else if ( g_u16HeartStep == 1 )
    {
        u16CanTaskStep = Step_HeartFB;
        g_u16HeartStep = 0;
    }
    else if ( g_u16ResetStep == 1 )  //��ʱ�ظ�����λ����
    {
        if ( RegReal_ResetCmd.u16Delay < 10 )
        {
            RegReal_ResetCmd.u16Delay = 10;
        }
        u16CanTaskStep = Step_ResetFB;
        g_u16ResetStep = 0;
    }
    else
    {
        u16CanTaskStep = StepS_None;
        return;
    }

    switch ( u16CanTaskStep )
    {
        case Step_PosCtrlFB:
        {
            if ( u16PosCtrlFB_LastSN != RegReal_PosCtrlFB.u16SN )
            {
                u16PosCtrlFB_DelayCnt = DELAYCNT;
                u16PosCtrlFB_LastSN = RegReal_PosCtrlFB.u16SN;
            }
            if ( u16PosCtrlFB_DelayCnt > 0 )
            {
                RegReal_PosCtrlFB.u16EndofCtrl = CmdRuning;
                u16PosCtrlFB_DelayCnt--;
            }
            Can_Host_Send ( Reg_PosCtrlFB );
        }
        break;
        case Step_RelayFB:
        {
            if ( u16RelayFB_LastSN != RegReal_RelayFB.u16SN )
            {
                u16RelayFB_DelayCnt = DELAYCNT;
                u16RelayFB_LastSN = RegReal_RelayFB.u16SN;
            }
            if ( u16RelayFB_DelayCnt > 0 )
            {
                RegReal_RelayFB.u16EndofCtrl = CmdRuning;
                u16RelayFB_DelayCnt--;
            }
            Can_Host_Send ( Reg_RelayFB );
        }
        break;
        case Step_SensorCaliFB:
        {
            Can_Host_Send ( Reg_SensorCaliFB );
        }
        break;
        case Step_FaultClearFB:
        {
            Can_Host_Send ( Reg_FaultClearFB );
        }
        break;
        case Step_ParamManageFB:
        {
            Can_Host_Send ( Reg_ParamManageFB );
        }
        break;
        case Step_HeartFB:
        {
            Can_Host_Send ( Reg_HeartFB );
        }
        break;
        case Step_ResetFB:
        {
            Can_Host_Send ( Reg_ResetFB );
        }
        break;
        case Step_FaultCodeFB:
        {
            Can_Host_Send ( Reg_FaultCodeFB );
        }
        break;
        case Step_SensorDataFB:
        {
            Can_Host_Send ( Reg_SensorDataFB );
        }
        break;
        case Step_State1FB:
        {
            Can_Host_Send ( Reg_BoardState1 );
        }
        break;
        case Step_State2FB:
        {
            Can_Host_Send ( Reg_BoardState2 );
        }
        break;
        case Step_State3FB:
        {
            Can_Host_Send ( Reg_BoardState3 );
        }
        break;
        default:
            break;
    }
}
/**
 * @brief       Host��Ӧ�Ĵ���״̬����
 * @param       �Ĵ���ID
 * @retval      ��
 */
static int Can_Host_Send ( uint32_t Cmd_ID )
{
    static uint8_t res = 0;
    Tx_frame.ID = Cmd_ID;
    Tx_frame.Len = 8;

    switch ( Cmd_ID )
    {
        case Reg_PosCtrlFB:
        {
            memcpy ( ( uint8_t * ) &Tx_frame.Buff, ( uint8_t * ) &RegReal_PosCtrlFB, Tx_frame.Len );
        }
        break;
        case Reg_RelayFB:
        {
            memcpy ( ( uint8_t * ) &Tx_frame.Buff, ( uint8_t * ) &RegReal_RelayFB, Tx_frame.Len );
        }
        break;
        case Reg_SensorCaliFB:
        {
            memcpy ( &RegReal_SensorCaliFB, &SensorCaliFB, sizeof ( SensorCaliFB ) );
            memcpy ( ( uint8_t * ) &Tx_frame.Buff, ( uint8_t * ) &RegReal_SensorCaliFB, Tx_frame.Len );
        }
        break;
        case Reg_FaultClearFB:
        {
            memcpy ( &RegReal_FaultClearFB, &RegReal_FaultClearCmd, sizeof ( RegReal_FaultClearCmd ) );
            memcpy ( ( uint8_t * ) &Tx_frame.Buff, ( uint8_t * ) &RegReal_FaultClearFB, Tx_frame.Len );
        }
        break;
        case Reg_ParamManageFB:
        {
            memcpy ( ( uint8_t * ) &Tx_frame.Buff, ( uint8_t * ) &RegReal_ParamManageFB, Tx_frame.Len );
        }
        break;
        case Reg_HeartFB:
        {
            memcpy ( &RegReal_HeartFB, &RegReal_HeartCmd, sizeof ( RegReal_HeartCmd ) );
            memcpy ( ( uint8_t * ) &Tx_frame.Buff, ( uint8_t * ) &RegReal_HeartFB, Tx_frame.Len );
        }
        break;
        case Reg_ResetFB:
        {
            memcpy ( &RegReal_ResetFB, &RegReal_ResetCmd, sizeof ( RegReal_ResetCmd ) );
            memcpy ( ( uint8_t * ) &Tx_frame.Buff, ( uint8_t * ) &RegReal_ResetFB, Tx_frame.Len );
        }
        break;
        case Reg_FaultCodeFB:
        {
            memcpy ( ( uint8_t * ) &Tx_frame.Buff, ( uint8_t * ) &RegReal_FaultCodeFB, Tx_frame.Len );
        }
        break;
        case Reg_SensorDataFB:
        {
            memcpy ( ( uint8_t * ) &Tx_frame.Buff, ( uint8_t * ) &RegReal_SensorDataFB, Tx_frame.Len );
        }
        break;
        case Reg_BoardState1:
        {
            memcpy ( ( uint8_t * ) &Tx_frame.Buff, ( uint8_t * ) &RegReal_BoardState1, Tx_frame.Len );
        }
        break;
        case Reg_BoardState2:
        {
            memcpy ( ( uint8_t * ) &Tx_frame.Buff, ( uint8_t * ) &RegReal_BoardState2, Tx_frame.Len );
        }
        break;
        case Reg_BoardState3:
        {
            memcpy ( ( uint8_t * ) &Tx_frame.Buff, ( uint8_t * ) &RegReal_BoardState3, Tx_frame.Len );
        }
        break;
        default:
            break;
    }
    res = Can_Host_send_msg ( Tx_frame.ID, ( uint8_t * ) &Tx_frame.Buff, Tx_frame.Len );

    return res;
}
/**
 * @brief       ��䷢�͵����ݽṹ��
 * @param       ��
 * @retval      ��
 */
void CanFBDataObtain ( void )
{
    RegReal_PosCtrlFB.u16Mode = g_sMotorCmd.eCmdMode;  //��������λ��
    RegReal_PosCtrlFB.u16SN = RegReal_PosCtrlCmd.u16SN;
    RegReal_PosCtrlFB.u16PosFbk = g_sMotor_Info.iqPosAbs;
    RegReal_PosCtrlFB.u16EndofCtrl = g_sMotorCmd.eRunState;
    
    RegReal_RelayFB.u16SN = RegReal_RelayCmd.u16SN;
    RegReal_RelayFB.u16EndofCtrl = gRelayCmd.eRunState;
    RegReal_RelayFB.u16CurSta = ( gRelayCmd.eFbMode == CLOSE ) ? CLOSE : OPEN;
    RegReal_RelayFB.u16SwichSta = g_sMotor_Info.u16Switch_State;

    RegReal_FaultCodeFB.u32Fault = g_u32FaultNum.all;
    RegReal_FaultCodeFB.u32Warning = g_u32WarningNum.all;
    
    RegReal_BoardState1.u16Ref1 = g_sMotor_Info.iqPosAbs;
    RegReal_BoardState1.u16Ref2 = g_sMotor_Info.iqSpd_PU;
    RegReal_BoardState1.u16Ref3 = _IQdiv16(g_sMotor_Info.iqEffectCurrent);
    RegReal_BoardState1.s16Ref4 = g_sMotor_Info.s16Temp;
}
/**
 * @brief       ����CAN1�����ݲ�������䵽Host�ṹ��
 * @param       ��
 * @retval      ��
 */
void Can_Host_Receive ( void )
{
    Rx_frame.ID = Can_Host_receive_msg ( Rx_frame.Buff );
    Rx_frame.Len = 8;
    if ( Rx_frame.ID == 0 ) return;

    switch ( Rx_frame.ID )
    {
        case Reg_UpDataCmd:
        {
            __disable_irq();         //�ر����ж�
            memcpy ( ( uint8_t * ) &BootRxHandle, ( uint8_t * ) &Rx_frame.Buff, Rx_frame.Len );
            __enable_irq();          // �������ж�
            if ( BOOT_CMD_LINK == BootRxHandle.u16Cmd )
            {
                m_eHostComState = HostCom_ToIAP;
            }
        }
        break;
        case Reg_PosCtrlCmd:
        {
            __disable_irq();         //�ر����ж�
            memcpy ( ( uint8_t * ) &RegReal_PosCtrlCmd, ( uint8_t * ) &Rx_frame.Buff, Rx_frame.Len );
            __enable_irq();          // �������ж�
        }
        break;
        case Reg_SensorCaliCmd:
        {
            __disable_irq();         //�ر����ж�
            memcpy ( ( uint8_t * ) &RegReal_SensorCaliCmd, ( uint8_t * ) &Rx_frame.Buff, Rx_frame.Len );
            __enable_irq();          // �������ж�
            g_u16SensorCaliStep = 1;
        }
        break;
        case Reg_FaultClearCmd:
        {
            __disable_irq();         //�ر����ж�
            memcpy ( ( uint8_t * ) &RegReal_FaultClearCmd, ( uint8_t * ) &Rx_frame.Buff, Rx_frame.Len );
            __enable_irq();          // �������ж�
            g_u16FultClearStep = 1;
        }
        break;
        case Reg_ParamManageCmd:
        {
            __disable_irq();         //�ر����ж�
            memcpy ( ( uint8_t * ) &RegReal_ParamManageCmd, ( uint8_t * ) &Rx_frame.Buff, Rx_frame.Len );
            __enable_irq();          // �������ж�
            g_u16ParamStep = 1;
        }
        break;
        case Reg_HeartCmd:
        {
            __disable_irq();         //�ر����ж�
            memcpy ( ( uint8_t * ) &RegReal_HeartCmd, ( uint8_t * ) &Rx_frame.Buff, Rx_frame.Len );
            __enable_irq();          // �������ж�
            s16CtrlLife = FullLife_500ms;
            g_u16HeartStep = 1;
        }
        break;
        case Reg_ResetCmd:
        {
            __disable_irq();         //�ر����ж�
            memcpy ( ( uint8_t * ) &RegReal_ResetCmd, ( uint8_t * ) &Rx_frame.Buff, Rx_frame.Len );
            __enable_irq();          // �������ж�
            g_u16ResetStep = 1;
        }
        break;
        case Reg_RelayCmd:
        {
            __disable_irq();         //�ر����ж�
            memcpy ( ( uint8_t * ) &RegReal_RelayCmd, ( uint8_t * ) &Rx_frame.Buff, Rx_frame.Len );
            __enable_irq();          // �������ж�
        }
        break;
        default:
            break;
    }
}
/**
 * @brief       �����λ
 * @param       ��
 * @retval      ��
 */
void SoftReset ( void )
{
    __disable_irq();
    __NVIC_SystemReset();
}
/**
 * @brief       Bootģʽ��1000HZ����
 * @param       ��
 * @retval      ��
 */
void BootProcess ( void )
{
    Set_BootFlg();
    __disable_irq();
    __NVIC_SystemReset();
}
IAPAPP_FLAG IapAppFlag;
uint8_t flaerr, WriteCnt;
static void Set_BootFlg ( void )
{
    WriteCnt = 3;
    IapAppFlag.all = ( * ( uint16_t* ) ( ADDRESS_FLAG ) );
    IapAppFlag.bit.AppWriteFlg = IAP_FLAG_Marked;   //���ñ��
    flaerr = Flash_HalfWordWrite ( ADDRESS_FLAG, & ( IapAppFlag.all ), 1 );
    while ( flaerr && WriteCnt )
    {
        WriteCnt--;
        flaerr = Flash_HalfWordWrite ( ADDRESS_FLAG, & ( IapAppFlag.all ), 1 );
    }
}
void Set_BootSuccessFlg ( void )
{
    WriteCnt = 3;
    IapAppFlag.all = ( * ( uint16_t* ) ( ADDRESS_FLAG ) );
    if ( IapAppFlag.bit.JmpNewOK != IAP_FLAG_Marked )
    {
        IapAppFlag.bit.JmpNewOK = IAP_FLAG_Marked;   //���ñ��
        flaerr = Flash_HalfWordWrite ( ADDRESS_FLAG, & ( IapAppFlag.all ), 1 );
        while ( flaerr && WriteCnt )
        {
            WriteCnt--;
            flaerr = Flash_HalfWordWrite ( ADDRESS_FLAG, & ( IapAppFlag.all ), 1 );
        }
        __disable_irq();
        __NVIC_SystemReset();//�ٴ��������DMA�쳣״̬
    }
}
void StoreVersion_Factory ( void )
{
    WriteCnt = 3;
    ParamHandle_Steer.Reg_SW_Factory = ( * ( uint32_t* ) ( ADDRESS_VerFactory ) );
    if ( ParamHandle_Steer.Reg_SW_Factory != ParamHandle_Steer.Reg_SW_Ver )
    {
        ParamHandle_Steer.Reg_SW_Factory = ParamHandle_Steer.Reg_SW_Ver;
        flaerr = Flash_WordWrite ( ADDRESS_VerFactory, & ParamHandle_Steer.Reg_SW_Factory, 1 );
        while ( flaerr && WriteCnt )
        {
            WriteCnt--;
            flaerr = Flash_WordWrite ( ADDRESS_VerFactory, & ParamHandle_Steer.Reg_SW_Factory, 1 );
        }
    }
}
void StoreVersion_NewApp ( void )
{
    WriteCnt = 3;
    ParamHandle_Steer.Reg_SW_NewApp = ( * ( uint32_t* ) ( ADDRESS_VerNewApp ) );
    if ( ParamHandle_Steer.Reg_SW_NewApp != ParamHandle_Steer.Reg_SW_Ver )
    {
        ParamHandle_Steer.Reg_SW_NewApp = ParamHandle_Steer.Reg_SW_Ver;
        flaerr = Flash_WordWrite ( ADDRESS_VerNewApp, & ParamHandle_Steer.Reg_SW_NewApp, 1 );
        while ( flaerr && WriteCnt )
        {
            WriteCnt--;
            flaerr = Flash_WordWrite ( ADDRESS_VerNewApp, & ParamHandle_Steer.Reg_SW_NewApp, 1 );
        }
    }
}
/**
 * @brief       д������E2Prom
 * @param       ��
 * @retval      ����1K
 */

#define RegWritePassword 0x01020304
#define MaximumTimeout 600000
U16ToU32Type U32DataBuff;
uint32_t TimeoutCnt = 0;
void ParamRead_Write ( void )
{
    if ( g_u16ParamStep == 1 )
    {
        RegReal_ParamManageFB.u16Addr = RegReal_ParamManageCmd.u16Addr;
        RegReal_ParamManageFB.u16Mode = RegReal_ParamManageCmd.u16Mode;
        if ( RegReal_ParamManageCmd.u16SN != RegReal_ParamManageFB.u16SN )  //��SNָ��
        {
            RegReal_ParamManageFB.u16SN = RegReal_ParamManageCmd.u16SN;
            if ( RegReal_ParamManageCmd.u16Mode == Para_READ ) //0
            {

                if ( RegReal_ParamManageCmd.u16Addr <= ( uint16_t* ) & ( ParamHandle_Steer.Reg_CRC ) - ( uint16_t* ) & ( ParamHandle_Steer. Reg_Spd_Slow ) )
                {
                    RegReal_ParamManageFB.u16Value = * ( ( uint16_t* ) & ( ParamHandle_Steer.Reg_Spd_Slow ) + RegReal_ParamManageCmd.u16Addr );
                }
                else if ( RegReal_ParamManageCmd.u16Addr >= ADDRESS_FMWParaREAD && \
                          RegReal_ParamManageCmd.u16Addr < ADDRESS_FMWParaREAD + \
                          ( uint16_t* ) & ( ParamHandle_Steer.Reg_Spd_Slow ) - ( uint16_t* ) & ( ParamHandle_Steer. Reg_Tab_Ver ) )
                {
                    RegReal_ParamManageFB.u16Value = * ( ( uint16_t* ) & ( ParamHandle_Steer.Reg_Tab_Ver ) + RegReal_ParamManageCmd.u16Addr - ADDRESS_FMWParaREAD );
                }
                else
                {
                    RegReal_ParamManageFB.u16Value = 0xFFFF;
                }
                //��ȡ����������Ϸ�
            }
            else if ( RegReal_ParamManageCmd.u16Mode == Para_WRITE )
            {
                SetBoardUpdata_Fault ( );
                if ( ( g_sMotorCmd.eCmdMode == CmdMode_FreeStop ) || ( g_sMotorCmd.eCmdMode == CmdMode_Fault ) )
                {
                    switch ( RegReal_ParamManageCmd.u16Addr )    //д���������Ϸ���
                    {
                        case 1028:
                        {
                            ParamHandle_Steer.Reg_PI_POS_KP = _IQmpy64 ( RegReal_ParamManageCmd.u16Value );
                        }
                        break;
                        case 1030:
                        {
                            ParamHandle_Steer.Reg_PI_POS_KI = _IQmpy64 ( RegReal_ParamManageCmd.u16Value );
                        }
                        break;
                        case 1032:
                        {
                            ParamHandle_Steer.Reg_PI_SPD_KP = _IQmpy64 ( RegReal_ParamManageCmd.u16Value );
                        }
                        break;
                        case 1034:
                        {
                            ParamHandle_Steer.Reg_PI_SPD_KI = _IQmpy64 ( RegReal_ParamManageCmd.u16Value );
                        }
                        break;
                        case 1036:
                        {
                            ParamHandle_Steer.Reg_PI_ID_KP = RegReal_ParamManageCmd.u16Value;
                        }
                        break;
                        case 1037:
                        {
                            ParamHandle_Steer.Reg_PI_ID_KI = RegReal_ParamManageCmd.u16Value;
                        }
                        break;
                        case 1038:
                        {
                            ParamHandle_Steer.Reg_PI_IQ_KP = RegReal_ParamManageCmd.u16Value;
                        }
                        break;
                        case 1039:
                        {
                            ParamHandle_Steer.Reg_PI_IQ_KI = RegReal_ParamManageCmd.u16Value;
                        }
                        break;
                        case 1041:
                        {
                            ParamHandle_Steer.Reg_SpeedMax = _IQsat ( RegReal_ParamManageCmd.u16Value, 3000, 0 );
                        }
                        break;
                        case 1042:
                        {
                            ParamHandle_Steer.Reg_CurrentMax = _IQsat ( RegReal_ParamManageCmd.u16Value, 1000, 0 );
                        }
                        break;
                        case 1046:
                        {
                            ParamHandle_Steer.Reg_CtrlDeadZone = _IQsat ( RegReal_ParamManageCmd.u16Value, _IQ ( 0.02 ), 0 );
                        }
                        break;
                        case 1047:
                        {
                            ParamHandle_Steer.Reg_PosDeadZone = _IQsat ( RegReal_ParamManageCmd.u16Value, _IQ ( 0.02 ), 0 );
                        }
                        break;
                        case 1048:
                        {
                            ParamHandle_Steer.Reg_PosMax = _IQsat ( RegReal_ParamManageCmd.u16Value, _IQ ( 0.75 ), 0 );
                        }
                        break;
                        case 1049:
                        {
                            ParamHandle_Steer.Reg_PosMin = _IQsat ( RegReal_ParamManageCmd.u16Value, _IQ ( 0.75 ), 0 );
                        }
                        break;
                        case 1052:
                        {
                            ParamHandle_Steer.Reg_CaliTimeOut = _IQsat ( RegReal_ParamManageCmd.u16Value, 20000, 0 );
                        }
                        break;
                        case 1053:
                        {
                            ParamHandle_Steer.Reg_RunTimeOut = _IQsat ( RegReal_ParamManageCmd.u16Value, 20000, 0 );
                        }
                        break;
                        case 1054:
                        {
                            ParamHandle_Steer.Reg_RC_Spdmax = RegReal_ParamManageCmd.u16Value;
                        }
                        break;
                        case 76:
                        {
                            ParamHandle_Steer.Reg_AccSpd = RegReal_ParamManageCmd.u16Value;
                        }
                        break;
                        case 77:
                        {
                            ParamHandle_Steer.Reg_DecSpd = RegReal_ParamManageCmd.u16Value;
                        }
                        break;                        
                        default:
                            break;
                    }

                    Param2Eeprom ( ParamHandle_Steer );
                    if ( RegReal_ParamManageCmd.u16Addr >= ADDRESS_FMWParaREAD )
                    {
                        RegReal_ParamManageFB.u16Value = * ( ( uint16_t* ) & ( ParamHandle_Steer.Reg_Tab_Ver ) + RegReal_ParamManageCmd.u16Addr - ADDRESS_FMWParaREAD );
                    }
                    else
                    {
                        RegReal_ParamManageFB.u16Value = * ( ( uint16_t* ) & ( ParamHandle_Steer.Reg_Spd_Slow ) + RegReal_ParamManageCmd.u16Addr );
                    }
                    //д���ض�
                }
            }
            else
            {
                RegReal_ParamManageFB.u16Addr = 0xFFFF;
            }
        }
        g_u16ParamStep = 2;
    }
    if ( ParamHandle_Steer.Reg_Password == RegWritePassword )
    {
        TimeoutCnt++;
        if ( TimeoutCnt >= MaximumTimeout )
        {
            ParamHandle_Steer.Reg_Password = 202401;
            Param2Eeprom ( ParamHandle_Steer );
            RegReal_ParamManageFB.u16Value = * ( ( uint16_t* ) & ( ParamHandle_Steer.Reg_Spd_Slow ) + RegReal_ParamManageCmd.u16Addr );
            TimeoutCnt = 0;
        }
    }
}
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */
