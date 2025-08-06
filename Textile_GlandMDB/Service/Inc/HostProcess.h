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

//-----------------------------------------------------------------------------
#ifndef _HostProcess_H_
#define _HostProcess_H_

#include "stdint.h"
#include "AngleMath.h"
#include "StFlash.h"

#define Reg_UpDataCmd           0x50     //����������תID
/*******************ָ���ID********************/
#define Reg_PosCtrlCmd          0x140
#define Reg_SensorCaliCmd       0x141
#define Reg_FaultClearCmd       0x142
#define Reg_ParamManageCmd      0x143
#define Reg_HeartCmd            0x144
#define Reg_ResetCmd            0x145
/*******************Ӧ���ID********************/
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

/*******************CAN����֡��ʽ����********************/
#define CMD_FIFO_SIZE                   8        //����֡����
#define CanRxDefault { 0, 0, 0, 0 }
#define CanTxDefault { 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF }
typedef struct
{
    uint32_t ID;
    uint32_t Len;
    uint8_t Buff[CMD_FIFO_SIZE];
} UserFrame_t;

/*******************����������ض���********************/
#define IAP_FLAG_Marked     0
#define IAP_FLAG_Clear      1

#define ADDRESS_FLAG            ADDR_FLASH_SECTOR_1
#define ADDRESS_VerBoot         ADDR_FLASH_SECTOR_0 + 0x00003FFC
#define ADDRESS_VerNewApp       ADDR_FLASH_SECTOR_6
#define ADDRESS_VerFactory      ADDR_FLASH_SECTOR_9

struct IAPAPP_FLAG_BIT  // write app flag
{
    uint16_t AppWriteFlg    : 1;        //�ȴ���������App��ǣ�Ϊ0����Boot�������
    uint16_t JmpErr         : 1;        //��תʧ�ܣ���Boot��ǣ�Boot���������ָ����������App
    uint16_t JmpNewOK       : 1;        //��ת�ɹ�����NewApp��ǣ�Boot�����������������������ת

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
    BOOT_CMD_LINK,          //����
    BOOT_CMD_WRITE,         //��¼
    BOOT_CMD_END,           //����
    BOOT_CMD_WAIT
} BootOpt;

/*******************����������ض���********************/
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
/*******************CANTASK����********************/
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
/*******************CMD����֡��ϸ����********************/
typedef struct
{
    uint16_t u16Mode;                     //λ�ÿ���ģʽ
    uint16_t u16SN;                       //λ�ÿ���SN
    uint16_t u16EN;                       //λ�ÿ���ʹ��
    uint16_t u16Ref;                      //λ�ø���ֵ
} RegDef_PosCtrlCmd;

typedef struct
{
    uint16_t u16Dev;                      //�궨�豸
    uint16_t u16SN;                       //SN
    uint16_t u16EN;                       //EN
    uint16_t u16Ref;                      //Ԥ��
} RegDef_SensorCaliCmd;

typedef struct
{
    uint16_t u16EN;                       //EN
    uint16_t u16SN;                       //SN
    uint32_t u32Bits;                     //����λ
} RegDef_FaultClearCmd;

typedef struct
{
    uint16_t u16Addr;                     //������ַ
    uint16_t u16SN;                       //SN
    uint16_t u16Mode;                     //��|дģʽ
    uint16_t u16Value;                    //����ֵ
} RegDef_ParamManageCmd;

typedef struct
{
    uint16_t u16HeartCnt;                 //����
    uint16_t u16Ref1;                     //Ԥ��
    uint16_t u16Ref2;                     //Ԥ��
    uint16_t u16Ref3;                     //Ԥ��
} RegDef_HeartCmd;

typedef struct
{
    uint16_t u16Delay;                    //��λ��ʱʱ�䣬��λ10mS
    uint16_t u16SN;                       //SN
    uint16_t u16Ref1;                     //Ԥ��
    uint16_t u16Ref2;                     //Ԥ��
} RegDef_ResetCmd;

/*******************FB����֡��ϸ����********************/
typedef struct
{
    uint16_t u16Mode;                     //λ�ÿ���ģʽ
    uint16_t u16SN;                       //λ�ÿ���SN
    uint16_t u16PosFbk;                   //λ�÷���ֵ
    uint16_t u16EndofCtrl;                //EndofCtrl״̬
} RegDef_PosCtrlFB;

typedef struct
{
    uint16_t u16Dev;                      //����������
    uint16_t u16SN;                       //SN
    uint16_t u16EndofCtrl;                //EndofCtrl״̬
    uint16_t u16Res;                      //Ԥ��
} RegDef_SensorCaliFB;

typedef struct
{
    uint16_t u16EndofCtrl;                //EndofCtrl״̬
    uint16_t u16SN;                       //SN
    uint32_t u32Bits;                     //����λ
} RegDef_FaultClearFB;

typedef struct
{
    uint16_t u16Addr;                     //������ַ
    uint16_t u16SN;                       //SN
    uint16_t u16Mode;                     //��|дģʽ
    uint16_t u16Value;                    //����ֵ
} RegDef_ParamManageFB;

typedef struct
{
    uint16_t u16HeartCnt;                 //����
    uint16_t u16Ref1;                     //Ԥ��
    uint16_t u16Ref2;                     //Ԥ��
    uint16_t u16Ref3;                     //Ԥ��
} RegDef_HeartFB;

typedef struct
{
    uint16_t u16ResetTime;                //��λ��ʱʱ�䣬��λ10mS
    uint16_t u16SN;                       //SN
    uint16_t u16Ref1;                     //Ԥ��
    uint16_t u16Ref2;                     //Ԥ��
} RegDef_ResetFB;

typedef struct
{
    uint32_t u32Fault;                    //������
    uint32_t u32Warning;                  //������
} RegDef_FaultCodeFB;

typedef struct
{
    uint16_t u16Ref1;            //Ԥ��
    uint16_t u16Ref2;            //Ԥ��
    uint16_t u16Ref3;            //Ԥ��
    uint16_t u16Ref4;            //Ԥ��
} RegDef_SensorDataFB;

typedef struct
{
    uint16_t u16Ref1;            //Ԥ��
    uint16_t u16Ref2;            //Ԥ��
    uint16_t u16Ref3;            //Ԥ��
    int16_t s16Ref4;             //Ԥ��
} RegDef_BoardState1;

typedef struct
{
    uint16_t u16Ref1;            //Ԥ��
    uint16_t u16Ref2;            //Ԥ��
    uint16_t u16Ref3;            //Ԥ��
    uint16_t u16Ref4;            //Ԥ��
} RegDef_BoardState2;

typedef struct
{
    uint16_t u16Ref1;            //Ԥ��
    uint16_t u16Ref2;            //Ԥ��
    uint16_t u16Ref3;            //Ԥ��
    uint16_t u16Ref4;            //Ԥ��
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




