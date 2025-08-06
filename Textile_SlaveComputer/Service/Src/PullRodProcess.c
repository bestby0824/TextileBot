/**
* °??¨?ù??(C)
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
* <????> | <°?±?> | <×÷??> | <?è??>
* xxxx/xx/xx | 1.0.0 | HWW | ???¨????
*
*/
//------------------------------------------------------------------------------
//-------------------- include files ----------------------------------------
#include "pullrodProcess.h"
#include "string.h"
#include "Debug_Com.h"
#include "pullrod_Ctrl.h"
#include "DataBaseProcess.h"
#include "Monitor.h"

int32_t SolenoidCRC_1 = 0, SolenoidCRC_2 = 0;

SolenoidReport_Pack SolenoidReport;
Cmddown SolenoidCmddown;          //液压板运动EndOfCtrl反馈;

#define Solenoid_Pack_Hand1_Default {            \
 RodID_Num,Solenoid_Opt_Read,   /*u8ID, u8Fun*/  \
0,0,        /*u16Reg_Addr, u16CRC*/             \
0,          /*s16Reg_Value*/                    \
0,          /*s32Reg_Value*/                    \
Dev_Stop,   /*Mode_Lift*/                       \
Dev_Stop,   /*Mode_Claw*/                       \
Dev_Stop,   /*Mode_BowRise*/                    \
Dev_Stop,   /*Mode_Rotate*/                     \
Dev_Stop,   /*Mode_Brake*/                      \
0,          /*Pos_Lift*/                        \
0,          /*Pos_Claw*/                        \
0,          /*Pos_BowRise*/                     \
0,          /*Pos_Rotate*/                      \
0,          /*Pos_Brake8*/                      \
}
#define SolenoidTXArry_Len       5
//-------------------- private data -----------------------------------------
static uint8_t RS232_TxBuf_Solenoid[RS232_TxBufLength_Solenoid];
static uint8_t RS232_RxBuf_Solenoid[RS232_RxBufLength_Solenoid];
static uint16_t RS232_RxLen_Solenoid;
static uint8_t SolenoidInitFlg = 0;
uint8_t LiftToBottom = 0;
uint16_t ClawState = 0     ;
//int16_t  m_nSolenoidLife = -1;
SolenoidCommand Solenoid_Pack_Hand1 = Solenoid_Pack_Hand1_Default, Solenoid_Pack_Hand1Array[SolenoidTXArry_Len];
//-------------------- private functions declare ----------------------------
static void SetSolenoidPackTask ( SolenoidCommand  *Pack_Tx_Handle );
//-------------------- public data ------------------------------------------
Solenoid_Info g_sSolenoid_Info1[RodID_Num];
//-------------------- public functions -------------------------------------

/*! \fn
*  \brief
*  \param
*  \return
*/
void Solenoid_Init ( void )
{
    int i;
    if ( SolenoidInitFlg == 0 )
    {
        Uart_Solenoid_Init ( 256000 );
        for ( i = 0; i < SolenoidTXArry_Len; i++ )
        {
            memcpy ( &Solenoid_Pack_Hand1Array[i], &Solenoid_Pack_Hand1, sizeof ( Solenoid_Pack_Hand1 ) );
        }
        SolenoidInitFlg = 1;
    }
}
/*
* @brief       函数
* @param       无
* @retval      无
*
*/
void SetSolenoidPos ( void )
{
    Solenoid_Pack_Hand1.u8Fun = Solenoid_Opt_Cmd;
    SetSolenoidPackTask ( &Solenoid_Pack_Hand1 );
}
/*
* @brief       函数
* @param       无
* @retval      无
*
*/
#if 0
uint16_t u16RxCRC, u16RxCRC1;
void SolenoidReceivPack ( void )
{
    uint8_t ParamReceiveCnt1_Solenoid = 0, ParamReceiveCnt2_Solenoid = 0, ParamReceiveCnt3_Solenoid = 0;
    SolenoidReport_Pack  Pack_Rx_Handle;
    Uart_Solenoid_Receive ( RS232_RxBuf_Solenoid, &RS232_RxLen_Solenoid );

    u16RxCRC = Modbus_GetCRC16 ( RS232_RxBuf_Solenoid, RS232_RxLen_Solenoid - 2 );
    u16RxCRC1 = RS232_RxBuf_Solenoid[RS232_RxLen_Solenoid - 1];
    u16RxCRC1 = ( u16RxCRC1 << 8 ) + RS232_RxBuf_Solenoid[RS232_RxLen_Solenoid - 2];
    SolenoidCRC_1++;
    if ( u16RxCRC1 == u16RxCRC )
    {
        if ( u8BootFlg_Solenoid == 0 )
        {
            if ( PARABURNHEAD == * ( uint16_t * ) RS232_RxBuf_Solenoid )
            {
                //透传模式
                Uart_Dbug_Send ( RS232_RxBuf_Solenoid, RS232_RxLen_Solenoid );
            }
            else
            {
                Pack_Rx_Handle.u8ID = RS232_RxBuf_Solenoid[2];
                Pack_Rx_Handle.u8Fun = RS232_RxBuf_Solenoid[3];
                if ( Pack_Rx_Handle.u8Fun == Solenoid_Opt_Report )
                {
                    memcpy ( &SolenoidReport, &RS232_RxBuf_Solenoid, sizeof ( SolenoidReport ) );
                    g_sSolenoid_Info1[RodID_Lift].Pos        = SolenoidReport.unClampHight;
                    g_sSolenoid_Info1[RodID_Claw].Pos        =  SolenoidReport.unClampOpenDegree;
                    g_sSolenoid_Info1[RodID_BowRise].Pos     =  SolenoidReport.unBowRiseDegree;

                    g_sSolenoid_Info1[RodID_Rotate].Pos      =  SolenoidReport.unRotateDegree;
                    g_sSolenoid_Info1[RodID_Brake].Pos       =  SolenoidReport.unBrakePercentage;
                    g_sSolenoid_Info1[RodID_SideShift].Pos   =  SolenoidReport.unSideShiftDegree;
                    g_sSolenoid_Info1[RodID_Lift].Pressure   =  SolenoidReport.unLift_Pressure;
                    ClawState = SolenoidReport.ClawState;
                    Fault_Handle1.Fault_Solenoid.all = SolenoidReport.u32FaultNum;
                    Solenoid_Warning.all = SolenoidReport.g_u16WarningNum;
                }
                else if ( Pack_Rx_Handle.u8Fun == Solenoid_Opt_Param_Solenoid )
                {
                    memcpy ( &ParamHandle_Solenoid, &RS232_RxBuf_Solenoid[6], sizeof ( ParamHandle_Solenoid ) );
                } else if ( Pack_Rx_Handle.u8Fun == Solenoid_Opt_Param_Hub485Fork )
                {
                    memcpy ( &ParamHandle_Hub485_Fork, &RS232_RxBuf_Solenoid[6], sizeof ( ParamHandle_Hub485_Fork ) );
                } else if ( Pack_Rx_Handle.u8Fun == Solenoid_Opt_Param_Hub485Body )
                {
                    memcpy ( &ParamHandle_Hub485_Body, &RS232_RxBuf_Solenoid[6], sizeof ( ParamHandle_Hub485_Body ) );
                }
//                m_nSolenoidLife = SolenoidCom_FullLife;
            }
        }
        else
        {
            if ( BOOTHEAD == * ( uint16_t * ) RS232_RxBuf_Solenoid )
            {
                Uart_Dbug_Send ( RS232_RxBuf_Solenoid, RS232_RxLen_Solenoid );
            }
            else if ( PARAHEAD == * ( uint16_t * ) RS232_RxBuf_Solenoid )
            {
                switch ( RS232_RxBuf_Solenoid[3] )
                {
                case Solenoid_Opt_Param_Solenoid:
                {
                    ParamReceiveCnt1_Solenoid++;
                }
                break;
                case Solenoid_Opt_Param_Hub485Fork:
                {
                    ParamReceiveCnt2_Solenoid++;
                }
                break;
                case Solenoid_Opt_Param_Hub485Body:
                {
                    ParamReceiveCnt3_Solenoid++;
                }
                break;
                default:
                    break;
                }

                if ( ( 3 <= ParamReceiveCnt1_Solenoid ) && ( 3 <= ParamReceiveCnt2_Solenoid ) && ( 3 <= ParamReceiveCnt3_Solenoid ) )
                {
                    u8BootFlg_Solenoid = 0;
                    ParamReceiveCnt1_Solenoid = 0;
                    ParamReceiveCnt2_Solenoid = 0;
                    ParamReceiveCnt3_Solenoid = 0;
                }
            }
        }
    }
}
#endif
/*
* @brief       函数
* @param       无
* @retval      无
*
*/
static void SetSolenoidPackTask ( SolenoidCommand  *Pack_Tx_Handle )
{
    uint16_t i_TX_flg;

    if ( Solenoid_Pack_Hand1Array[0].u8ID != RodID_Num ) {
    }
    for ( i_TX_flg = 0; i_TX_flg < SolenoidTXArry_Len - 1; i_TX_flg++ )
    {
        memcpy ( &Solenoid_Pack_Hand1Array[i_TX_flg], &Solenoid_Pack_Hand1Array[i_TX_flg + 1], sizeof ( Solenoid_Pack_Hand1 ) );
    }
    memcpy ( &Solenoid_Pack_Hand1Array[SolenoidTXArry_Len - 1], &Solenoid_Pack_Hand1, sizeof ( Solenoid_Pack_Hand1 ) );
}
/*
* @brief       串口控制液压板函数(不用)
* @param       无
* @retval      无
*
*/
void SolenoidSendPack ( void )
{
    uint32_t len = 6;
    uint16_t i_TX_flg;
    SolenoidCommand  Pack_Tx_Handle;

    for ( i_TX_flg = 0; i_TX_flg < SolenoidTXArry_Len; i_TX_flg++ )
    {
        if ( RodID_Num != Solenoid_Pack_Hand1Array[i_TX_flg].u8ID )
        {
            memcpy ( &Pack_Tx_Handle, &Solenoid_Pack_Hand1Array[i_TX_flg], sizeof ( Solenoid_Pack_Hand1 ) );
            Solenoid_Pack_Hand1Array[i_TX_flg].u8ID = RodID_Num;
            break;
        }
        else if ( i_TX_flg == SolenoidTXArry_Len - 1 )
        {
            return ;
        }
    }
    SolenoidCtrlCmd.u8Fun = Solenoid_Opt_Cmd;
    SolenoidCtrlCmd.u16SyncWord = 0xAA55;

    if ( Pack_Tx_Handle.u8Fun == Solenoid_Opt_Cmd )
    {
        len = sizeof ( SolenoidCtrlCmd );

        SolenoidCtrlCmd.u16CRC = Modbus_GetCRC16 ( ( uint8_t * ) &SolenoidCtrlCmd, len - 2 );

        memcpy ( &RS232_TxBuf_Solenoid, &SolenoidCtrlCmd, sizeof ( SolenoidCtrlCmd ) );
    }
    Uart_Solenoid_Send ( RS232_TxBuf_Solenoid, len );
}
/*
* @brief       函数
* @param       无
* @retval      无
*
*/
// ============================================
// slew programmable ramper
// ============================================
_iq ramper ( _iq in, _iq out, _iq rampDelta )
{
    _iq err;

    if ( rampDelta == 0 )  rampDelta = 1;
    err = in - out;
    if ( ( err > rampDelta ) && ( _IQabs ( in ) > _IQabs ( out ) ) )        return ( out + rampDelta );
    else if ( ( err < -rampDelta ) && ( _IQabs ( in ) > _IQabs ( out ) ) )  return ( out - rampDelta );
    else                        return ( in );

}
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


