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
#ifndef _SolenoidProcess__H_
#define _SolenoidProcess__H_
//-------------------- include files ----------------------------------------
#include "Uart_pullrod.h"
#include "AngleMath.h"
#include "MODBusCRC.h"
//-------------------- public definitions -----------------------------------
#define RS232_TxBufLength_Solenoid   256
#define RS232_RxBufLength_Solenoid   256

typedef struct {
    _iq Spd;
    uint16_t Pos;
    uint16_t Pressure;
    uint16_t Volt;
    uint16_t Current;
} Solenoid_Info;
#define Solenoid_InfoDefault { \
0,/* Spd*/  \
0,/* Pos*/  \
0,/* Volt*/  \
0,/* Current*/  \
0/* SolenoidSta*/  \
}
//-------------------- public data ------------------------------------------

typedef enum {
    RodID_Lift = 0,
    RodID_Claw,
    RodID_BowRise,
    RodID_Rotate,
    RodID_Brake,
    RodID_SideShift,
    RodID_Claw2,
    RodID_Num,
} SolenoidID;
typedef enum {
    HeightID = 0,
    ClawOpenID,
    BowRiseID,
    RotateID,
    BrakeSenID,
} PosSensorID;
typedef struct {
    _iq Pos_Q_1m;   //Q格式长度，IQ(1)代表1m，精度(1/65535)m
    _iq Pos_Q_1m_Last;
    _iq Pos_Src;    //传感器原始数据 单位微米
    _iq Spd_Q;      //Q格式速度，_IQ(1)代表1m/s
    uint32_t LastTick;
    uint32_t NowTick;
} Sensor_Info;

typedef struct
{
    uint16_t Lift : 1;
    uint16_t Claw : 1;
    uint16_t BowRise : 1;
    uint16_t Rotate : 1;
    uint16_t SideShift : 1;
    uint16_t Brake : 1;
    uint16_t RadLift : 1;
    uint16_t rev : 9;
} Mask_BITS;

typedef union
{
    uint16_t all;
    Mask_BITS bit;
} SolenoidMask;

typedef struct
{
    uint16_t unSN; //命令序号，ack回应使用
    int16_t EN; //上位机控启停
    uint16_t unPos; //目标Lift高度 32767 = 6m
    uint16_t CtrlMode; //控制模式
} Cmd;

typedef struct            //EndOfCtrl判断结构体
{
    uint8_t liftEndOfCtrl;
    uint8_t RotateEndOfCtrl;
    uint8_t BowRiseEndOfCtrl;
    uint8_t ClawEndOfCtrl;
    uint8_t SideShiftEndOfCtrl;
    uint8_t Rad_Lift_EndOfCtrl;
    uint8_t SlowSpd_EndOfCtrl;
    uint8_t BreakCanel_EndOfCtrl;
    uint8_t Brake_EndOfCtrl;
    uint8_t TelescopicMBDRelay_EndOfCtrl;
} Cmddown;

typedef struct
{
    uint16_t u16SyncWord;   //HEAD
    uint8_t u8ID, u8Fun;
    uint8_t Res0; //
    uint8_t Res1; //
    SolenoidMask CmdSet;    //接收到命令后写入变量同时置1，接收到推杆相应ACK后置0，用来兼容连续和断续命令模式
    Cmd Lift;
    Cmd Claw;
    Cmd BowRise;
    Cmd Rotate;
    Cmd SideShift;
    Cmd RadLift;
    uint16_t Weight;
    uint16_t Res2;
    uint16_t Res3;
    uint16_t Res4;
    uint16_t u16StaFlg;                            //bit0:复位置1
    uint16_t LiftMode;                             //升降模式：0是位置伺服，1是下降到底
    uint16_t u16CRC;
} SolenoidCommand;

typedef struct {
    uint16_t u16SyncWord;   //HEAD
    uint8_t u8ID, u8Fun;
    uint16_t unBrakePercentage;
    uint16_t unClampHight;                  //抱夹当前高度
    uint16_t unClampOpenDegree;             //抱夹当前开度
    uint16_t unBowRiseDegree;               //俯仰角度
    uint16_t unRotateDegree;                //旋转角度
    uint16_t unSideShiftDegree;             //侧移位置
    uint16_t unLiftRaderDegree;             //升降机构位置

    uint16_t unLift_Pressure;               //升降油压
    uint16_t unClaw_Pressure;               //抱夹油压
    uint16_t unSideShift_Pressure1;         //侧移油压
    uint16_t unSideShift_Pressure2;         //侧移油压

    uint16_t unLiftAck;                     //升降命令SN
    uint16_t unClampAck;                    //抱夹命令SN
    uint16_t unBowRiseAck;                  //俯仰命令SN
    uint16_t unRotateAck;                   //旋转命令SN
    uint16_t unSideShiftAck;                //侧移命令SN
    uint16_t unLiftRaderAck;                //升降机构命令SN

    uint16_t SolenoidEndOfCtrl;             //EndOfCtrl； bit/0~3/4~7/8~11/12~15,分别代表升降，抱夹，俯仰，旋转
    uint16_t SolenoidEndOfCtrl2;            //侧移机构EndOfCtrl；
    uint16_t PosStopFlag;                   //定点停车标志
    uint32_t u32FaultNum;                   //故障编码
    uint16_t g_u16WarningNum;               //警告编码
    uint16_t u16StaFlg;                     //bit0:初始化时状态为1，5秒后置为3，若检测到一次从1到3的变化，即重启过一次
    uint16_t ClawState;                     //抱夹是否夹纸卷状态
    uint16_t  SideShiftState;
    uint16_t  Res1;
    uint16_t u16CRC;
} SolenoidReport_Pack;


#define Solenoid_Opt_Cmd          1   //位置给定
#define Solenoid_Opt_Report       2   //自动上传,100Hz
#define Solenoid_Opt_Read         3   //读取状态
#define Solenoid_Opt_SetSta       4   //设置状态
#define Solenoid_Opt_Param_Solenoid       5   //上传推杆参数
#define Solenoid_Opt_Param_Hub485Fork    6   //上传Hub485参数
#define Solenoid_Opt_Param_Hub485Body    7   //上传Hub485参数

#define Solenoid_Reg_EN          0x00    //1-使能，0-禁使能


extern Solenoid_Info g_sSolenoid_Info1[RodID_Num];
extern SolenoidCommand SolenoidCtrlCmd;

extern SolenoidReport_Pack SolenoidReport;
extern uint16_t ClawState;                 //抱夹夹紧状态1夹紧，0未夹紧
extern Cmddown SolenoidCmddown;          //液压板运动EndOfCtrl反馈;

//-------------------- public functions -------------------------------------
void Solenoid_Init ( void );
void SetSolenoidReg ( uint8_t RegAddr, int16_t s16Value, SolenoidID ID );
void SetSolenoidReg32 ( uint8_t RegAddr, int32_t s32Value, SolenoidID ID );
void SetSolenoidPos ( void );

void SolenoidReceivPack ( void );
void SolenoidSendPack ( void );
_iq ramper ( _iq in, _iq out, _iq rampDelta );
#endif // _SolenoidProcess_H_

//-----------------------End of file------------------------------------------
/** @}*/
