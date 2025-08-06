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

//-----------------------------------------------------------------------------
#ifndef _Debug__H_
#define _Debug__H_
//-------------------- include files ----------------------------------------
#include "Uart_Dbug.h"
#include "StFlash.h"
#include "DataTypesDef.h"
//-------------------- public definitions -----------------------------------
#define Debug_Data      0   //用于输出数据
#define Debug_Handle    1   //用于手持终端

#define DebugMode       Debug_Data    //


#define Router_ID_ControlBoard      0x1000
#define Router_ID_SolenoidBoard      0x1100
#define Router_ID_SteeringWheel     0x1200
#define Router_ID_485HubBody        0x1110
#define Router_ID_485HubFork        0x1120

//#define Dev_Router_ID               Router_ID_ControlBoard
//#define SolenoidCom_ID               ( 0x0F00 & Router_ID_SolenoidBoard )
//#define SteeringWheelCom_ID         ( 0x0F00 & Router_ID_SteeringWheel )



struct IAPAPP_FLAG_BIT  // write app flag
{
    uint16_t AppWriteFlg    : 1;        //等待升级，由App标记（为0），Boot清除（）
    uint16_t JmpErr         : 1;        //跳转失败，由Boot标记，Boot清除，更改指针跳往出厂App
    uint16_t JmpNewOK       : 1;        //跳转成功，由NewApp标记，Boot清除，如无升级需求正常跳转

    uint16_t reserved: 13;
};
#define IAP_FLAG_Marked     0
#define IAP_FLAG_Clear      1
typedef union
{
    uint16_t                    all;
    struct IAPAPP_FLAG_BIT bit;
} IAPAPP_FLAG;
#define ADDRESS_FLAG            ADDR_FLASH_SECTOR_1
#define ADDRESS_VerBoot         ADDR_FLASH_SECTOR_0 + 0x00003FFC
#define ADDRESS_VerNewApp       ADDR_FLASH_SECTOR_6
#define ADDRESS_VerFactory      ADDR_FLASH_SECTOR_9

#define DIAGHEAD        0xB004
#define LOGHEAD         0xB005
#define BOOTHEAD        0xB007
#define PARAHEAD        0xB008
#define PARABURNHEAD    0xB00B
typedef struct {
    uint16_t u16Cmd;
    uint16_t u16Data;   //数据
    uint8_t u8Data[8];   //数据
} DefRxReg_RCMODE;

typedef struct {
    uint16_t u16SyncWord;    //0xAA55
    uint16_t u16SN[2];
    uint16_t u16DataLen;      //单位byte
    
} DComHeader;

typedef struct {
    DComHeader TxHeader;
    uint16_t u16CmdFb;
    uint16_t u16Data;   //数据
    uint16_t u16Crc;
} DefTxReg_RCMODE;

typedef struct
{
    uint16_t u16FrameHeader;
    uint16_t u16SN_L;
    uint16_t u16SN_H;
    uint16_t u16Length;
    uint16_t u16Cmd_ID;
    uint16_t u16Data;
    uint16_t u16Crc;
} S_WIRELESS_FRAME;

typedef struct
{
    uint16_t u16FrameHeader;
    uint16_t u16SN_L;
    uint16_t u16SN_H;
    uint16_t u16Length;
    uint16_t u16Cmd_ID;
    uint8_t u8Data[8];
    uint16_t u16Crc;
} S_WIRELESS_FRAMEx;
//-------------------- public data ------------------------------------------
//extern RXStruct RXHandle1;
extern uint32_t WirelessBDSN;
extern uint8_t RCMODEComFlag;
extern uint8_t u8BootFlg_Solenoid,u8BootFlg_SteeringWheel;
extern DefRxReg_RCMODE RegReal_RCMODECmd;
extern DefTxReg_RCMODE TxRegReal_RCMODE;
//-------------------- public functions -------------------------------------

void DbugCom_ReceiveData ( void );
void DbugCom_SendPkg ( uint8_t Num );
//void BootProcess ( void );
void Set_BootFlg ( void );
void Set_BootSuccessFlg ( void );
void StoreVersion_Factory ( void );
void StoreVersion_NewApp ( void );
void ParamReceiveProcess ( void );
void RCMODE_Send(void);
#endif // _XXX_H_

//-----------------------End of file------------------------------------------
/** @}*/
