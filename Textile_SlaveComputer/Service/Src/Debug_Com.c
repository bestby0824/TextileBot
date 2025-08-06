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
#include "Debug_Com.h"
#include "string.h"
#include "MODBusCRC.h"
#include "StFlash.h"
#include "Uart_pullrod.h"
#include "SteeringWheelProcess.h"
#include "DataBaseProcess.h"
#include "Monitor.h"
#include "Wireless_Ctrl.h"
//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
uint8_t DbugCom_TXBuf[Uart_Dbug_MAX_COMM_LENTH];
uint8_t DbugCom_RXBuf[Uart_Dbug_MAX_COMM_LENTH];
uint16_t DbugCom_TXLen = 0, DbugCom_RXLen = 0;
//RXStruct RXHandle1;
//TXStruct TXHandle1;
uint8_t Param_RXBuf[256] = {0};
uint8_t u8BootFlg_Solenoid = 0, u8BootFlg_SteeringWheel = 0;
#define MaxRecvDataLength   255

S_WIRELESS_FRAME Tx_Wireless_Frame, Rx_Wireless_Frame;
S_WIRELESS_FRAMEx Rx_Wireless_Framex;

//-------------------- private functions declare ----------------------------
//static void ParaBurnAck ( void );
//static void ParaBurnNack ( void );
#define ProtocalHeader_Length 8
#define  ProtocalTail_Length 2
//-------------------- public data ------------------------------------------
DefRxReg_RCMODE RegReal_RCMODECmd;
DefTxReg_RCMODE TxRegReal_RCMODE;
uint32_t WirelessBDSN = 0;
uint8_t RCMODEComFlag = 0;
//-------------------- public functions -------------------------------------
/*
* @brief       CRC校验函数
* @param       无
* @retval      无
*
*/
static int8_t DbugCom_CheckCrc ( uint8_t* pbData, uint16_t unPackLen, uint16_t unDatakLen )
{
    if ( unDatakLen < ( unPackLen + ProtocalTail_Length ) )
        return 0;
    if ( * ( uint16_t* ) ( pbData + unPackLen ) == Modbus_GetCRC16 ( pbData, unPackLen ) )
        return 1;
    else
        return 0;
}

static void DbugCom_ResolveCommand ( uint8_t* pbData, uint16_t unDataLen )
{
uint16_t unCurPos = 0;
DComHeader* psHeader = NULL;
    while ( unCurPos < MaxRecvDataLength ) //255
    {
        if ( ( unCurPos + ProtocalHeader_Length + ProtocalTail_Length ) > unDataLen ) //剩余数据不足一包头，认为已处理完
            break;
        psHeader = ( DComHeader* ) ( pbData + unCurPos );//帧头位置
        if ( 0xAA55 != psHeader->u16SyncWord ) //查找协议头
        {
            unCurPos++;
            continue;
        }
        if ( ( unDataLen - unCurPos ) < psHeader->u16DataLen + ProtocalTail_Length ) //不完整包，跳过
            break;
        if ( 1 == DbugCom_CheckCrc ( pbData + unCurPos, psHeader->u16DataLen, unDataLen - unCurPos ) )
        {
            memcpy (&RegReal_RCMODECmd, pbData + unCurPos + sizeof(DComHeader), sizeof(RegReal_RCMODECmd));//接收解析
            unCurPos += psHeader->u16DataLen + ProtocalTail_Length;//
//            psHeader->u16SN[1] = ENDIAN_SWAP16(psHeader->u16SN[1]);
            WirelessBDSN = * ( uint32_t* )psHeader->u16SN;
            RCMODEComFlag = 1;
        }
        else
        {
            unCurPos++;
        }
    }
}

void DbugCom_ReceiveData ( void )
{
    uint16_t u16RxCrc1, u16RxCrc2;
    
    if ( 1 != Uart_Dbug_Receive ( DbugCom_RXBuf, &DbugCom_RXLen ) )
    {
        DbugCom_RXLen = 0;
    }
    if ( 0 != DbugCom_RXLen )
    {
        u16RxCrc1 = ( uint16_t ) ( DbugCom_RXBuf[DbugCom_RXLen-1] << 8 ) + ( uint16_t ) ( DbugCom_RXBuf[DbugCom_RXLen-2] & 0x00FF );
        u16RxCrc2 = Modbus_GetCRC16 ( ( uint8_t *) &DbugCom_RXBuf, DbugCom_RXLen - 2 );
        if ( u16RxCrc1 == u16RxCrc2 ) {
            if ( DbugCom_RXBuf[6] == 14) {
                memcpy ( &Rx_Wireless_Frame, &DbugCom_RXBuf, sizeof ( Rx_Wireless_Frame ) );
                if ( ( Rx_Wireless_Frame.u16FrameHeader == 0xAA55 ) && ( Rx_Wireless_Frame.u16Length == 0x000E ) ) {
                    RegReal_RCMODECmd.u16Cmd = Rx_Wireless_Frame.u16Cmd_ID;
                    RegReal_RCMODECmd.u16Data = Rx_Wireless_Frame.u16Data; 
                    WirelessBDSN = ( Rx_Wireless_Frame.u16SN_H << 16 )+ Rx_Wireless_Frame.u16SN_L;
                }
                RCMODEComFlag = 1;
            } else if ( DbugCom_RXBuf[6] == 20) {
                memcpy ( &Rx_Wireless_Framex, &DbugCom_RXBuf, sizeof ( Rx_Wireless_Framex ) );
                if ( ( Rx_Wireless_Framex.u16FrameHeader == 0xAA55 ) && ( Rx_Wireless_Framex.u16Length == 0x0014 ) && ( Rx_Wireless_Framex.u16Cmd_ID == RC_RunningCmd )) {
                    RegReal_RCMODECmd.u16Cmd = Rx_Wireless_Framex.u16Cmd_ID;
                    memcpy ( &RegReal_RCMODECmd.u8Data, &Rx_Wireless_Framex.u8Data, 8 ); 
                    WirelessBDSN = ( Rx_Wireless_Framex.u16SN_H << 16 )+ Rx_Wireless_Framex.u16SN_L;                    
                }
            }
        }
//        DbugCom_ResolveCommand ( DbugCom_RXBuf, DbugCom_RXLen );//上位机接收函数
        DbugCom_RXLen = 0;
        memset ( DbugCom_RXBuf, 0, MaxRecvDataLength );
    }
    
}

void RCMODE_Send(void)
{
    TxRegReal_RCMODE.TxHeader.u16SyncWord = 0xAA55;
    if (ParamHandle_Control.Reg_PairingFlag == 0){
        TxRegReal_RCMODE.TxHeader.u16SN[0] = 0xFFFF;
        TxRegReal_RCMODE.TxHeader.u16SN[1] = 0XFFFF;
    } else {
        TxRegReal_RCMODE.TxHeader.u16SN[0] = ParamHandle_Control.Reg_SN;
        TxRegReal_RCMODE.TxHeader.u16SN[1] = ENDIAN_SWAP16(ParamHandle_Control.Reg_SN >> 8);
    }
    TxRegReal_RCMODE.TxHeader.u16DataLen = sizeof(TxRegReal_RCMODE) - ProtocalTail_Length;
    TxRegReal_RCMODE.u16CmdFb = RegReal_RCMODECmd.u16Cmd;
//    TxRegReal_RCMODE.u16Data = RegReal_RCMODECmd.u16Data;
    TxRegReal_RCMODE.u16Crc = Modbus_GetCRC16 ( ( uint8_t* )&TxRegReal_RCMODE, sizeof ( TxRegReal_RCMODE ) - ProtocalTail_Length);
    Uart_Dbug_Send ( ( uint8_t* )&TxRegReal_RCMODE, sizeof ( TxRegReal_RCMODE ));
}
#if 0
void DbugCom_ReceiveData ( void )
{
    uint16_t CRC_Value;
    if ( 1 == Uart_Dbug_Receive ( DbugCom_RXBuf, &DbugCom_RXLen ) ) {
        if ( DbugCom_RXLen > 5 )
        {
            CRC_Value = * ( uint16_t * ) &DbugCom_RXBuf[DbugCom_RXLen - 2];
            if ( CRC_Value == Modbus_GetCRC16 ( DbugCom_RXBuf, DbugCom_RXLen - 2 ) )
            {
                if ( BOOTHEAD == * ( uint16_t * ) DbugCom_RXBuf )
                {
                    memcpy ( &RXHandle1, DbugCom_RXBuf, DbugCom_RXLen );
                }
                else if ( ( PARAHEAD == * ( uint16_t * ) DbugCom_RXBuf ) || ( PARABURNHEAD == * ( uint16_t * ) DbugCom_RXBuf ) )
                {
                    memcpy ( Param_RXBuf, DbugCom_RXBuf, DbugCom_RXLen );
                }
            }
        }
    }
}

void DbugCom_SendPkg ( uint8_t Num )
{
    uint16_t CRC_Value;
    uint16_t len;
    * ( uint16_t * ) DbugCom_TXBuf = PARAHEAD;
    switch ( Num )
    {
    case 0:
    {
        len = sizeof ( ParamHandle_Control ) + 4;
        ParamHandle_Control.Reg_HW_Ver = 1;
        memcpy ( &DbugCom_TXBuf[2], ( uint8_t * ) &ParamHandle_Control, len - 4 );
    }
    break;
    case 1:
    {
        len = sizeof ( ParamHandle_Solenoid ) + 4;
        ParamHandle_Solenoid.Reg_HW_Ver = 1;
        memcpy ( &DbugCom_TXBuf[2], ( uint8_t * ) &ParamHandle_Solenoid, len - 4 );
    }
    break;
    case 2:
    {
        len = sizeof ( ParamHandle_Steer ) + 4;
        ParamHandle_Steer.Reg_SW_Boot = ParamHandle_Control.Reg_SW_Boot;
        ParamHandle_Steer.Reg_SW_Factory = 1000000;
        if(ParamHandle_Steer.Reg_SW_Ver>1000000) ParamHandle_Steer.Reg_SW_NewApp = ParamHandle_Steer.Reg_SW_Ver;
        memcpy ( &DbugCom_TXBuf[2], ( uint8_t * ) &ParamHandle_Steer, len - 4 );
        DbugCom_TXBuf[2] = 1;
        DbugCom_TXBuf[4] = 2;
        DbugCom_TXBuf[6] = 3;
    }
    break;
    case 3:
    {
        len = sizeof ( ParamHandle_Hub485_Fork ) + 4;
        memcpy ( &DbugCom_TXBuf[2], ( uint8_t * ) &ParamHandle_Hub485_Fork, len - 4 );
    }
    break;
    case 4:
    {
        len = sizeof ( ParamHandle_Hub485_Body ) + 4;
        memcpy ( &DbugCom_TXBuf[2], ( uint8_t * ) &ParamHandle_Hub485_Body, len - 4 );
    }
    break;
    case 5:
    {
        * ( uint16_t * ) DbugCom_TXBuf = DIAGHEAD;
        len = sizeof ( Fault_Handle1 ) + 4;
        memcpy ( &DbugCom_TXBuf[2], ( uint8_t * ) &Fault_Handle1, len - 4 );
    }
    break;

    default:
        break;
    }
    if ( len > 5 ) {
        * ( uint16_t * ) &DbugCom_TXBuf[len - 2] = Modbus_GetCRC16 ( ( uint8_t * ) &DbugCom_TXBuf, len - 2 );
        Uart_Dbug_Send ( DbugCom_TXBuf, len );
    }

}
/**
 * @brief       boot升级
 * @param       寄存器ID
 * @retval      无
 */
void BootProcess ( void )
{
    if ( BOOTHEAD == RXHandle1.u16SyncWord )
    {
        if ( Dev_Router_ID == RXHandle1.u16Dev_ID ) //升级当前板子
        {
            Set_BootFlg();
            __disable_irq();
            __NVIC_SystemReset();
        } else       //按路由表透传
        {
            switch ( RXHandle1.u16Dev_ID & 0x0F00 )
            {
            case SolenoidCom_ID:
            {
                u8BootFlg_Solenoid = 1;
                Uart_Solenoid_Send ( ( uint8_t* ) ( &RXHandle1 ), RXHandle1.u16Length );
            }
            break;
            case SteeringWheelCom_ID:
            {
                u8BootFlg_SteeringWheel = 1;
                RS485_SteeringWheel_Send ( ( uint8_t* ) ( &RXHandle1 ), RXHandle1.u16Length );
            }
            break;
            }

        }
        RXHandle1.u16SyncWord = 0x55AA;
    }
}

void ParamReceiveProcess ( void )
{
    if ( PARAHEAD == * ( uint16_t * ) Param_RXBuf )
    {
        switch ( ( * ( ParamStruct_Hub485 * ) &Param_RXBuf[2] ).Reg_Dev_Ver )
        {
        case Dev_ForlLift_Control:
        {
            if ( 0 == memcmp ( &ParamHandle_Control, &Param_RXBuf[2], sizeof ( ParamHandle_Control ) ) ) return;
            memcpy ( ( uint8_t * ) &ParamHandle_Control, &Param_RXBuf[2], sizeof ( ParamHandle_Control ) );
            if ( 0 == Param2Eeprom ( ) )
            {
                __disable_irq();
                __NVIC_SystemReset();
            }
            memset ( Param_RXBuf, 0x00, 256 );
        }
        break;
        case Dev_ForkLift_Solenoid:
        {
            if ( 0 == memcmp ( &ParamHandle_Solenoid, &Param_RXBuf[2], sizeof ( ParamHandle_Solenoid ) ) ) return;
            Uart_Solenoid_Send ( ( uint8_t* ) ( Param_RXBuf ), ( sizeof ( ParamHandle_Solenoid ) + 4 ) );
            memset ( Param_RXBuf, 0x00, 256 );
        }
        break;
        case Dev_ForkLift_485Hub:
        {
            if ( 0 == ( * ( ParamStruct_Hub485 * ) &Param_RXBuf[2] ).Reg_Mode ) {
                if ( 0 == memcmp ( &ParamHandle_Hub485_Body, &Param_RXBuf[2], sizeof ( ParamHandle_Hub485_Body ) ) ) return;
                Uart_Solenoid_Send ( ( uint8_t* ) ( &RXHandle1 ), RXHandle1.u16Length );
            } else if ( 3 == ( * ( ParamStruct_Hub485 * ) &Param_RXBuf[2] ).Reg_Mode )
            {
                if ( 0 == memcmp ( &ParamHandle_Hub485_Fork, &Param_RXBuf[2], sizeof ( ParamHandle_Hub485_Fork ) ) ) return;
                Uart_Solenoid_Send ( ( uint8_t* ) ( &RXHandle1 ), RXHandle1.u16Length );
            }
        }
        break;
        default:
            break;
        }
    }

    if ( PARABURNHEAD == * ( uint16_t * ) Param_RXBuf )
    {
        switch ( ( * ( ParamStruct_Hub485 * ) &Param_RXBuf[2] ).Reg_Dev_Ver )
        {
        case Dev_ForlLift_Control:
        {
            memcpy ( ( uint8_t * ) &ParamHandle_Control, &Param_RXBuf[2], sizeof ( ParamHandle_Control ) );
            if ( 0 == Param2Eeprom ( ) )
            {
                ParaBurnAck ();
//					__disable_irq();
//					__NVIC_SystemReset();
            }
            else
            {
                ParaBurnNack ();
            }
            memset ( Param_RXBuf, 0x00, 256 );
        }
        break;
        case Dev_ForkLift_Solenoid:
        {
            Uart_Solenoid_Send ( ( uint8_t* ) ( Param_RXBuf ), ( sizeof ( ParamHandle_Solenoid ) + 4 ) );
            memset ( Param_RXBuf, 0x00, 256 );
        }
        break;
        default:
            break;
        }
    }
}

static void ParaBurnAck ( void )
{
    uint16_t CRC_Value;
    * ( uint16_t * ) DbugCom_TXBuf = PARABURNHEAD;
    * ( uint16_t * ) &DbugCom_TXBuf[2] = Dev_ForlLift_Control;
    DbugCom_TXBuf[4] = 0x01;
    DbugCom_TXBuf[5] = 0x00;
    CRC_Value = Modbus_GetCRC16 ( ( uint8_t * ) &DbugCom_TXBuf, 6 );
    * ( uint16_t * ) &DbugCom_TXBuf[6] = CRC_Value;

    Uart_Dbug_Send ( DbugCom_TXBuf, 8 );
}

static void ParaBurnNack ( void )
{
    uint16_t CRC_Value;
    * ( uint16_t * ) DbugCom_TXBuf = PARABURNHEAD;
    * ( uint16_t * ) &DbugCom_TXBuf[2] = Dev_ForlLift_Control;
    DbugCom_TXBuf[4] = 0x01;
    DbugCom_TXBuf[5] = 0x01;
    CRC_Value = Modbus_GetCRC16 ( ( uint8_t * ) &DbugCom_TXBuf, 6 );
    * ( uint16_t * ) &DbugCom_TXBuf[6] = CRC_Value;

    Uart_Dbug_Send ( DbugCom_TXBuf, 8 );
}
#endif
IAPAPP_FLAG IapAppFlag;
uint8_t flaerr, WriteCnt;

void Set_BootFlg ( void )
{
    WriteCnt = 3;
    IapAppFlag.all = ( * ( uint16_t* ) ( ADDRESS_FLAG ) );
    IapAppFlag.bit.AppWriteFlg = IAP_FLAG_Marked;   //配置标记
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
        IapAppFlag.bit.JmpNewOK = IAP_FLAG_Marked;   //配置标记
        flaerr = Flash_HalfWordWrite ( ADDRESS_FLAG, & ( IapAppFlag.all ), 1 );
        while ( flaerr && WriteCnt )
        {
            WriteCnt--;
            flaerr = Flash_HalfWordWrite ( ADDRESS_FLAG, & ( IapAppFlag.all ), 1 );
        }
        __disable_irq();
        __NVIC_SystemReset();//再次重启解除DMA异常状态
    }
}
void StoreVersion_Factory ( void )
{
    WriteCnt = 3;
    ParamHandle_Control.Reg_SW_Factory = ( * ( uint32_t* ) ( ADDRESS_VerFactory ) );
    if ( ParamHandle_Control.Reg_SW_Factory != ParamHandle_Control.Reg_SW_Ver )
    {
        ParamHandle_Control.Reg_SW_Factory = ParamHandle_Control.Reg_SW_Ver;
        flaerr = Flash_WordWrite ( ADDRESS_VerFactory, & ParamHandle_Control.Reg_SW_Factory, 1 );
        while ( flaerr && WriteCnt )
        {
            WriteCnt--;
            flaerr = Flash_WordWrite ( ADDRESS_VerFactory, & ParamHandle_Control.Reg_SW_Factory, 1 );
        }
    }
}
void StoreVersion_NewApp ( void )
{
    WriteCnt = 3;
    ParamHandle_Control.Reg_SW_NewApp = ( * ( uint32_t* ) ( ADDRESS_VerNewApp ) );
    if ( ParamHandle_Control.Reg_SW_NewApp != ParamHandle_Control.Reg_SW_Ver )
    {
        ParamHandle_Control.Reg_SW_NewApp = ParamHandle_Control.Reg_SW_Ver;
        flaerr = Flash_WordWrite ( ADDRESS_VerNewApp, & ParamHandle_Control.Reg_SW_NewApp, 1 );
        while ( flaerr && WriteCnt )
        {
            WriteCnt--;
            flaerr = Flash_WordWrite ( ADDRESS_VerNewApp, & ParamHandle_Control.Reg_SW_NewApp, 1 );
        }
    }
}
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


