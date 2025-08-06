/**
* 版权所有(C)
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
* <日期> | <版本> | <作者> | <描述>
*
* xxxx/xx/xx | 1.0.0 | MUNIU | 创建文件
*
*/
//------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
#ifndef _HostProcess_H_
#define _HostProcess_H_

#include "stdint.h"
#include "AngleMath.h"
#include "StFlash.h"

#define Reg_UpDataCmd           0x50     //在线升级跳转ID
/*******************指令包ID********************/
#define Reg_PosCtrlCmd          0x140
#define Reg_SensorCaliCmd       0x141
#define Reg_FaultClearCmd       0x142
#define Reg_ParamManageCmd      0x143
#define Reg_HeartCmd            0x144
#define Reg_ResetCmd            0x145
/*******************应答包ID********************/
#define Reg_PosCtrlFB           0x500
#define Reg_SensorCaliFB        0x501
#define Reg_FaultClearFB        0x502
#define Reg_ParamManageFB       0x503
#define Reg_HeartFB             0x504
#define Reg_ResetFB             0x505
#define Reg_FaultCodeFB         0x510
#define Reg_SensorDataFB        0x511
#define Reg_BoardState1         0x512
#define Reg_BoardState2         0x513
#define Reg_BoardState3         0x514

/*******************CAN数据帧格式定义********************/
#define CMD_FIFO_SIZE                   8        //数据帧长度
#define CanRxDefault { 0, 0, 0, 0 }
#define CanTxDefault { 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF }
typedef struct
{
    uint32_t ID;
    uint32_t Len;
    uint8_t Buff[CMD_FIFO_SIZE];
} UserFrame_t;

/*******************在线升级相关定义********************/
#define IAP_FLAG_Marked     0
#define IAP_FLAG_Clear      1

#define ADDRESS_FLAG            ADDR_FLASH_SECTOR_1
#define ADDRESS_VerBoot         ADDR_FLASH_SECTOR_0 + 0x00003FFC
#define ADDRESS_VerNewApp       ADDR_FLASH_SECTOR_6
#define ADDRESS_VerFactory      ADDR_FLASH_SECTOR_9

struct IAPAPP_FLAG_BIT  // write app flag
{
    uint16_t AppWriteFlg    : 1;        //等待升级，由App标记（为0），Boot清除（）
    uint16_t JmpErr         : 1;        //跳转失败，由Boot标记，Boot清除，更改指针跳往出厂App
    uint16_t JmpNewOK       : 1;        //跳转成功，由NewApp标记，Boot清除，如无升级需求正常跳转

    uint16_t reserved: 13;
};

typedef union
{
    uint16_t                    all;
    struct IAPAPP_FLAG_BIT bit;
} IAPAPP_FLAG;

typedef struct {
    uint16_t u16Cmd;
    uint8_t u8Data[6];
} BootRxStruct;

typedef enum {
    BOOT_CMD_NONE = 0,
    BOOT_CMD_LINK,          //连接
    BOOT_CMD_WRITE,         //烧录
    BOOT_CMD_END,           //结束
    BOOT_CMD_WAIT
} BootOpt;

/*******************参数管理相关定义********************/
#define ADDRESS_FMWParaREAD     1000
typedef enum
{
    Para_READ = 0,
    Para_WRITE,
} E_PAPAM_OPT;

typedef union {
    uint16_t bytes[2];
    uint32_t all;
} U16ToU32Type;
/*******************CANTASK定义********************/
typedef enum {
    Step_PosCtrlFB = 0,
    Step_SensorCaliFB,
    Step_FaultClearFB,
    Step_ParamManageFB,
    Step_HeartFB,
    Step_ResetFB,
    Step_FaultCodeFB,
    Step_SensorDataFB,
    Step_State1FB,
    Step_State2FB,
    Step_State3FB,
    
    StepS_None,
} CanTask_StepS;
/*******************CMD数据帧详细定义********************/
typedef struct
{
    uint16_t u16Mode;                     //位置控制模式
    uint16_t u16SN;                       //位置控制SN
    uint16_t u16EN;                       //位置控制使能
    uint16_t u16Ref;                      //位置给定值
} RegDef_PosCtrlCmd;

typedef struct
{
    uint16_t u16Dev;                      //标定设备
    uint16_t u16SN;                       //SN
    uint16_t u16EN;                       //EN
    uint16_t u16Ref;                      //预留
} RegDef_SensorCaliCmd;

typedef struct
{
    uint16_t u16EN;                       //EN
    uint16_t u16SN;                       //SN
    uint32_t u32Bits;                     //具体位
} RegDef_FaultClearCmd;

typedef struct
{
    uint16_t u16Addr;                     //参数地址
    uint16_t u16SN;                       //SN
    uint16_t u16Mode;                     //读|写模式
    uint16_t u16Value;                    //参数值
} RegDef_ParamManageCmd;

typedef struct
{
    uint16_t u16HeartCnt;                 //心跳
    uint16_t u16Ref1;                     //预留
    uint16_t u16Ref2;                     //预留
    uint16_t u16Ref3;                     //预留
} RegDef_HeartCmd;

typedef struct
{
    uint16_t u16Delay;                    //复位延时时间，单位10mS
    uint16_t u16SN;                       //SN
    uint16_t u16Ref1;                     //预留
    uint16_t u16Ref2;                     //预留
} RegDef_ResetCmd;

/*******************FB数据帧详细定义********************/
typedef struct
{
    uint16_t u16Mode;                     //位置控制模式
    uint16_t u16SN;                       //位置控制SN
    uint16_t u16PosFbk;                   //位置反馈值
    uint16_t u16EndofCtrl;                //EndofCtrl状态
} RegDef_PosCtrlFB;

typedef struct
{
    uint16_t u16Dev;                      //传感器类型
    uint16_t u16SN;                       //SN
    uint16_t u16EndofCtrl;                //EndofCtrl状态
    uint16_t u16Res;                      //预留
} RegDef_SensorCaliFB;

typedef struct
{
    uint16_t u16EndofCtrl;                //EndofCtrl状态
    uint16_t u16SN;                       //SN
    uint32_t u32Bits;                     //具体位
} RegDef_FaultClearFB;

typedef struct
{
    uint16_t u16Addr;                     //参数地址
    uint16_t u16SN;                       //SN
    uint16_t u16Mode;                     //读|写模式
    uint16_t u16Value;                    //参数值
} RegDef_ParamManageFB;

typedef struct
{
    uint16_t u16HeartCnt;                 //心跳
    uint16_t u16Ref1;                     //预留
    uint16_t u16Ref2;                     //预留
    uint16_t u16Ref3;                     //预留
} RegDef_HeartFB;

typedef struct
{
    uint16_t u16ResetTime;                //复位延时时间，单位10mS
    uint16_t u16SN;                       //SN
    uint16_t u16Ref1;                     //预留
    uint16_t u16Ref2;                     //预留
} RegDef_ResetFB;

typedef struct
{
    uint32_t u32Fault;                    //故障码
    uint32_t u32Warning;                  //警告码
} RegDef_FaultCodeFB;

typedef struct
{
    uint16_t u16Ref1;            //预留
    uint16_t u16Ref2;            //预留
    uint16_t u16Ref3;            //预留
    uint16_t u16Ref4;            //预留
} RegDef_SensorDataFB;

typedef struct
{
    uint16_t u16Ref1;            //预留
    uint16_t u16Ref2;            //预留
    uint16_t u16Ref3;            //预留
    int16_t s16Ref4;             //预留
} RegDef_BoardState1;

typedef struct
{
    uint16_t u16Ref1;            //预留
    uint16_t u16Ref2;            //预留
    uint16_t u16Ref3;            //预留
    uint16_t u16Ref4;            //预留
} RegDef_BoardState2;

typedef struct
{
    uint16_t u16Ref1;            //预留
    uint16_t u16Ref2;            //预留
    uint16_t u16Ref3;            //预留
    uint16_t u16Ref4;            //预留
} RegDef_BoardState3;
//-------------------- public data ------------------------------------------
extern uint16_t g_u16SensorCaliStep, g_u16FultClearStep;
extern RegDef_PosCtrlCmd RegReal_PosCtrlCmd;
extern RegDef_SensorCaliCmd RegReal_SensorCaliCmd;
extern RegDef_FaultClearCmd RegReal_FaultClearCmd;
extern RegDef_SensorCaliFB RegReal_SensorCaliFB;
extern RegDef_ResetCmd RegReal_ResetCmd;

//-------------------- public functions -------------------------------------
void Can_Host_Receive ( void );
void CanTaskProcess ( void );
void CanFBDataObtain ( void );
void SoftReset ( void );
void BootProcess ( void );
void Set_BootSuccessFlg ( void );
void StoreVersion_NewApp ( void );
void StoreVersion_Factory ( void );
void ParamRead_Write ( void );

#endif  /* _HostProcess_H_ */

//-----------------------End of file------------------------------------------
/** @}*/




