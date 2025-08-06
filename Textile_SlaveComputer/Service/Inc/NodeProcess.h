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
#ifndef _NodeProcess_H_
#define _NodeProcess_H_

#include "stdint.h"
#include "AngleMath.h"

#define CMD_FIFO_SIZE                512
#define Reg_ValveLiftCmd             0x100          //升降指令  --液压板   
//#define Reg_ValveClawCmd             0x110          //开合指令
#define Reg_ValveBowRiseCmd          0x102          //俯仰指令
#define Reg_ValveRotateCmd           0x103          //旋转指令
#define Reg_ValveSideShiftCmd        0x104          //侧移指令
#define Reg_ValveLiftRaderCmd        0x105          //升降机构指令
#define Reg_ValveSensorCaliCmd       0x106          //传感器标定指令
#define Reg_ValveFaultClearCmd       0x107          //故障清除指令
#define Reg_ValveFaultEnableCmd      0x108          //故障屏蔽指令
#define Reg_ValveCtrlState           0x109          //下位机状态发送

#define Reg_SteerPosCtrlCmd          0x180          //转向控制  --转向板
#define Reg_SteerSensorCaliCmd       0x181          //传感器标定指令
#define Reg_SteerFaultClearCmd       0x182          //故障清除指令
#define Reg_SteerFaultEnableCmd      0x183          //故障屏蔽指令
#define Reg_SteerCtrlState           0x184          //下位机状态发送
#define Reg_LightCtrlState           0x101          //灯光控制


#define Reg_BootModeFB               0x40
#define Reg_IMUState                 0x50           //IMU

//#define Reg_ValveSensorPos1FB        0x200          //液压板抱夹状态反馈1
//#define Reg_ValveSensorPos2FB        0x201          //液压板抱夹状态反馈2
//#define Reg_ValveCmd1SNFB            0x202          //液压板抱夹SN反馈1
//#define Reg_ValveCmd2SNFB            0x203          //液压板抱夹SN反馈2
//#define Reg_ValveCtrlMode1FB         0x204          //液压板控制模式反馈1
//#define Reg_ValveCtrlMode2FB         0x205          //液压板控制模式反馈2
//#define Reg_ValveState1FB            0x206          //液压板控制结果反馈1
//#define Reg_ValvePressureFB          0x207          //液压板控制结果反馈2
//#define Reg_ValveFaultCodeFB         0x208          //液压板故障反馈
//#define Reg_ValveCaliStateFB         0x209          //液压板标定结果反馈
//#define Reg_ValveState2FB            0x20A          //液压板故障清除反馈

//#define Reg_SteerPosCtrlFB           0x300          //转向控制反馈1
//#define Reg_SteerSensorDataFB        0x305          //转向控制反馈2
//#define Reg_SteerFaultCodeFB         0x302          //转向控制故障反馈
//#define Reg_SteerSensorCaliFB        0x303          //转向控制传感器标定反馈
//#define Reg_SteerFaultCtrlFB         0x304          //故障清除/屏蔽反馈

#define Reg_RadarSend                0x60          //雷达区域选择
#define Reg_RadarRev                 0x61
extern int16_t  RadarLife, DrawWireLife;
/*Textiles*/
/*控制*/
#define Reg_RotatingMBDCmd                          0x110
#define Reg_TelescopicMBDCmd                        0x120
#define Reg_HuggingMBDCmd                           0x130
#define Reg_GlandMBDCmd                             0x140
/*零点标定*/
#define Reg_RotatingMBDCalaCmd                      0x111
#define Reg_TelescopicMBDCalaCmd                    0x121
#define Reg_HuggingMBDCalaCmd                       0x131
#define Reg_GlandMBDCalaCmd                         0x141
/*故障清除*/
#define Reg_RotatingMBDFaultClearCmd                0x112
#define Reg_TelescopicMBDFaultClearCmd              0x122
#define Reg_HuggingMBDFaultClearCmd                 0x132
#define Reg_GlandMBDFaultClearCmd                   0x142
/*参数管理*/
#define Reg_RotatingMBDParamCmd                     0x113
#define Reg_TelescopicMBDParamCmd                   0x123
#define Reg_HuggingMBDParamCmd                      0x133
#define Reg_GlandMBDParamCmd                        0x143
/*心跳*/
#define Reg_RotatingMBDWatchDogCmd                  0x114
#define Reg_TelescopicMBDWatchDogCmd                0x124
#define Reg_HuggingMBDWatchDogCmd                   0x134
#define Reg_GlandMBDWatchDogCmd                     0x144
/*复位*/
#define Reg_RotatingMBDResetCmd                     0x115
#define Reg_TelescopicMBDResetCmd                   0x125
#define Reg_HuggingMBDResetCmd                      0x135
#define Reg_GlandMBDResetCmd                        0x145

/*控制ack*/
#define Reg_RotatingMBDCtrlFB                       0x200
#define Reg_TelescopicMBDCtrlFB                     0x300
#define Reg_HuggingMBDCtrlFB                        0x400
#define Reg_GlandMBDCtrlFB                          0x500
/*零点标定ack*/
#define Reg_RotatingMBDCalaFB                       0x201
#define Reg_TelescopicMBDCalaFB                     0x301
#define Reg_HuggingMBDCalaFB                        0x401
#define Reg_GlandMBDCalaFB                          0x501
/*故障清除ack*/
#define Reg_RotatingMBDFaultClearFB                 0x202
#define Reg_TelescopicMBDFaultClearFB               0x302
#define Reg_HuggingMBDFaultClearFB                  0x402
#define Reg_GlandMBDFaultClearFB                    0x502
/*参数管理ack*/
#define Reg_RotatingMBDParamFB                      0x203
#define Reg_TelescopicMBDParamFB                    0x303
#define Reg_HuggingMBDParamFB                       0x403
#define Reg_GlandMBDParamFB                         0x503
/*心跳*/
#define Reg_RotatingMBDWatchDogFB                   0x204
#define Reg_TelescopicMBDWatchDogFB                 0x304
#define Reg_HuggingMBDWatchDogFB                    0x404
#define Reg_GlandMBDWatchDogFB                      0x504
/*复位*/
#define Reg_RotatingMBDResetFB                      0x205
#define Reg_TelescopicMBDResetFB                    0x305
#define Reg_HuggingMBDResetFB                       0x405
#define Reg_GlandMBDResetFB                         0x505
/*故障状态*/
#define Reg_RotatingMBDFaultStaFB                   0x210
#define Reg_TelescopicMBDFaultStaFB                0x310
#define Reg_HuggingMBDFaultStaFB                    0x410
#define Reg_GlandMBDFaultStaFB                      0x510
/*传感器状态*/
#define Reg_RotatingMBDSensorStaFB                  0x211
#define Reg_TelescopicMBDSensorStaFB               0x311
#define Reg_HuggingMBDSensorStaFB                   0x411
#define Reg_GlandMBDSensorStaFB                     0x511
/*运行状态1*/
#define Reg_RotatingMBDRunningSta1FB                0x212
#define Reg_TelescopicMBDRunningSta1FB             0x312
#define Reg_HuggingMBDRunningSta1FB                 0x412
#define Reg_GlandMBDRunningSta1FB                   0x512
/*运行状态2*/
#define Reg_RotatingMBDRunningSta2FB                0x213
#define Reg_TelescopicMBDRunningSta2FB             0x313
#define Reg_HuggingMBDRunningSta2FB                 0x413
#define Reg_GlandMBDRunningSta2FB                   0x513
/*运行状态3*/
#define Reg_RotatingMBDRunningSta3FB                0x214
#define Reg_TelescopicMBDRunningSta3FB             0x314
#define Reg_HuggingMBDRunningSta3FB                 0x414
#define Reg_GlandMBDRunningSta3FB                   0x514

#define Step_DrawWireSensor                     0x08
#define Step_TelescopicMBDRelayCmd                  0x126
#define Step_TelescopicMBDPushMotorCmd              0x127
#define Step_TelescopicMBDRelayCmdFB                0x306
typedef enum
{
    Para_READ = 0,
    Para_WRITE,
} E_PAPAM_OPT;
/*控制*/
typedef struct//0x60
{  
   uint8_t u8RadarSw;
   uint8_t u8WorkDir;
   int16_t s16TurnDeg;
   uint8_t Res[4];
}RegDef_RadarSend;
extern RegDef_RadarSend  RegReal_RadarSend;
typedef struct//0x61
{  
   uint8_t u8LeftRadarInf;
   uint8_t u8RightRadarInf;
   uint8_t u8RadarInf;
   uint8_t Res[5];
}RegDef_RadarRev;
extern RegDef_RadarRev RegReal_RadarRev;
typedef struct//0x126
{
   uint16_t u16EN;
   uint16_t u16SN;
   uint16_t u16Cmd;
   uint16_t Res;
}RegDef_TelescopicMBDRelayCmd;
typedef struct//0x306
{
   uint16_t u16SN;      //Sn
   uint16_t u16Sta;     //运动状态
   uint16_t u16RelayCmdFb;   //舵机状态
   uint16_t u16TouchSwSta;//接触开关状态
}RegDef_TelescopicMBDRelayCmdFB;
extern RegDef_TelescopicMBDRelayCmd RegReal_TelescopicMBDRelayCmd;
extern RegDef_TelescopicMBDRelayCmdFB RegReal_TelescopicMBDRelayCmdFB;
/*控制*/
typedef struct//0x110
{
    uint16_t u16Cmd;
    uint16_t u16SN;
    uint16_t u16EN;
    uint16_t u16Ref;
} RegDef_MotorMBDCmd;
extern RegDef_MotorMBDCmd RegReal_RotatingMBDCmd;
extern RegDef_MotorMBDCmd RegReal_TelescopicMBDCmd;//0x120
extern RegDef_MotorMBDCmd RegReal_HuggingMBDCmd;//0x130
extern RegDef_MotorMBDCmd RegReal_GlandMBDCmd;//0x140
//typedef struct//0x120
//{
//    uint16_t u16Cmd;
//    uint16_t u16SN;
//    uint16_t u16EN;
//    uint16_t u16Ref;
//} RegDef_TelescopicMBDCmd;

//typedef struct
//{
//   uint16_t u16Cmd;
//    uint16_t u16SN;
//    uint16_t u16EN;
//    uint16_t u16Ref;
//} RegDef_HuggingMBDCmd;//0x130

//typedef struct
//{
//   uint16_t u16Cmd;
//    uint16_t u16SN;
//    uint16_t u16EN;
//    uint16_t u16Ref;
//} RegDef_GlandMBDCmd;//0x140
/*标定*/
typedef struct//0x111
{
    uint16_t u16Cmd;
    uint16_t u16SN;
    uint16_t u16EN;
    uint16_t u16Ref;
} RegDef_MotorMBDCalaCmd;
//typedef struct//0x121
//{
//    uint16_t u16Cmd;
//    uint16_t u16SN;
//    uint16_t u16EN;
//    uint16_t u16Ref;
//}RegDef_TelescopicMBDCalaCmd;

//typedef struct//0x131
//{
//    uint16_t u16Cmd;
//    uint16_t u16SN;
//    uint16_t u16EN;
//    uint16_t u16Ref;
//} RegDef_HuggingMBDCalaCmd;

//typedef struct//0x141
//{
//    uint16_t u16Cmd;
//    uint16_t u16SN;
//    uint16_t u16EN;
//    uint16_t u16Ref;
//} RegDef_GlandMBDCalaCmd;
typedef struct//0x08
{
    uint8_t u8DataLen;
    uint8_t u8Id;
    uint8_t u8Cmd;
    uint8_t u8Databuf[5];
}RegDef_DrawWireSensor;
extern RegDef_DrawWireSensor Reg_DrawWireSensorFb;
typedef struct//0x112
{
    uint16_t u16EN;
    uint16_t u16SN;  
    uint32_t u32FaultBit;
} RegDef_MotorMBDFaultClearCmd;
//extern RegDef_RotatingFaultClearCmd  Reg_RotatingFaultClearCmd ;
//typedef struct//0x122
//{
//    uint16_t u16EN;
//    uint16_t u16SN;  
//    uint32_t u32FaultBit;
//} RegDef_TelescopicMBDFaultClearCmd;

//typedef struct//0x132
//{
//    uint16_t u16EN;
//    uint16_t u16SN;  
//    uint32_t u32FaultBit;
//} RegDef_HuggingMBDFaultClearCmd;

//typedef struct//0x142
//{
//    uint16_t u16EN;
//    uint16_t u16SN;  
//    uint32_t u32FaultBit;
//} RegDef_GlandMBDFaultClearCmd;

typedef struct//0x113
{
    uint16_t u16Addr;                       //寄存器地址
    uint16_t u16SN;                         //指令SN
    E_PAPAM_OPT mod;                        //0-读，1-写（非伺服状态生效）
    uint16_t u16Value;                      //寄存器值
} RegDef_MotorMBDParamCmd;
//extern RegDef_RotatingParamCmd Reg_RotatingParamCmd;
//typedef struct//0x123
//{
//    uint16_t u16Addr;                       //寄存器地址
//    uint16_t u16SN;                         //指令SN
//    E_PAPAM_OPT mod;                        //0-读，1-写（非伺服状态生效）
//    uint16_t u16Value;                      //寄存器值
//} RegDef_TelescopicMBDParamCmd;

//typedef struct//0x133
//{
//    uint16_t u16Addr;                       //寄存器地址
//    uint16_t u16SN;                         //指令SN
//    E_PAPAM_OPT mod;                        //0-读，1-写（非伺服状态生效）
//    uint16_t u16Value;                      //寄存器值
//} RegDef_HuggingMBDParamCmd;

//typedef struct//0x143
//{
//    uint16_t u16Addr;                       //寄存器地址
//    uint16_t u16SN;                         //指令SN
//    E_PAPAM_OPT mod;                        //0-读，1-写（非伺服状态生效）
//    uint16_t u16Value;                      //寄存器值
//} RegDef_GlandMBDParamCmd;

typedef struct//0x114
{
    uint16_t u16SN;                         //指令SN
    uint16_t res1;                          //预留
    uint16_t res2;                          //预留
    uint16_t res3;                          //预留
} RegDef_MotorMBDWatchDogCmd;

//extern RegDef_RotatingWatchDogCmd  Reg_RotatingWatchDogCmd ;
//typedef struct//0x124
//{
//    uint16_t u16SN;                         //指令SN
//    uint16_t res1;                          //预留
//    uint16_t res2;                          //预留
//    uint16_t res3;                          //预留
//} RegDef_TelescopicMBDWatchDogCmd;

//typedef struct//0x134
//{
//    uint16_t u16SN;                         //指令SN
//    uint16_t res1;                          //预留
//    uint16_t res2;                          //预留
//    uint16_t res3;                          //预留
//} RegDef_HuggingMBDWatchDogCmd;

//typedef struct//0x144
//{
//    uint16_t u16SN;                         //指令SN
//    uint16_t res1;                          //预留
//    uint16_t res2;                          //预留
//    uint16_t res3;                          //预留
//} RegDef_GlandMBDWatchDogCmd;

typedef struct//0x115
{
    uint16_t u16Delay;                      //延时复位，单位mS
    uint16_t u16SN;                         //指令SN
    uint16_t u16Res1;                       //预留
    uint16_t u16Res2;                       //预留
} RegDef_MotorMBDResetCmd;
//extern RegDef_RotatingReset Reg_RotatingReset;
//typedef struct//0x125
//{
//    uint16_t u16Delay;                      //延时复位，单位mS
//    uint16_t u16SN;                         //指令SN
//    uint16_t u16Res1;                       //预留
//    uint16_t u16Res2;                       //预留
//} RegDef_TelescopicMBDResetCmd;

//typedef struct//0x135
//{
//    uint16_t u16Delay;                      //延时复位，单位mS
//    uint16_t u16SN;                         //指令SN
//    uint16_t u16Res1;                       //预留
//    uint16_t u16Res2;                       //预留
//} RegDef_HuggingMBDResetCmd;

//typedef struct//0x145
//{
//    uint16_t u16Delay;                      //延时复位，单位mS
//    uint16_t u16SN;                         //指令SN
//    uint16_t u16Res1;                       //预留
//    uint16_t u16Res2;                       //预留
//} RegDef_GlandMBDResetCmd;

typedef struct//0x200
{
    uint16_t u16CmdFB;
    uint16_t u16SN;
    uint16_t u16Ack;
    uint16_t u16Sta;
} RegDef_RotatingMBDCtrlFB;
extern RegDef_RotatingMBDCtrlFB RegReal_RotatingMBDCtrlFB;
typedef struct//0x300
{
    uint16_t u16CmdFB;
    uint16_t u16SN;
    uint16_t u16Ack;
    uint16_t u16Sta;
} RegDef_TelescopicMBDCtrlFB;
extern RegDef_TelescopicMBDCtrlFB RegReal_TelescopicMBDCtrlFB;
typedef struct//0x400
{
    uint16_t u16CmdFB;
    uint16_t u16SN;
    uint16_t u16Ack;
    uint16_t u16Sta;
} RegDef_HuggingMBDCtrlFB;
extern RegDef_HuggingMBDCtrlFB RegReal_HuggingMBDCtrlFB;
typedef struct//0x500
{
    uint16_t u16CmdFB;
    uint16_t u16SN;
    uint16_t u16Ack;
    uint16_t u16Sta;
} RegDef_GlandMBDCtrlFB;
extern RegDef_GlandMBDCtrlFB RegReal_GlandMBDCtrlFB;
typedef struct//0x201
{
    uint16_t u16CmdFB;
    uint16_t u16SN;
    uint16_t u16Sta;
    uint16_t res1;
} RegDef_RotatingMBDCalaFB;
//extern RegDef_RotatingMBDCalaFB Reg_RotatingMBDCalaFB;
typedef struct//0x301
{
    uint16_t u16CmdFB;
    uint16_t u16SN;
    uint16_t u16Sta;
    uint16_t res1;
} RegDef_TelescopicMBDCalaFB;

typedef struct//0x401
{
    uint16_t u16CmdFB;
    uint16_t u16SN;
    uint16_t u16Sta;
    uint16_t res1;
} RegDef_HuggingMBDCalaFB;

typedef struct//0x501
{
    uint16_t u16CmdFB;
    uint16_t u16SN;
    uint16_t u16Sta;
    uint16_t res1;
} RegDef_GlandMBDCalaFB;

typedef struct//0x202
{
    uint16_t u16CmdFB;
    uint16_t u16SN;
    uint32_t u32FaultBitFB;
} RegDef_RotatingMBDFaultClearFB;
//extern RegDef_RotatingMBDFaultClearFB  Reg_RotatingMBDFaultClearFB ;
typedef struct//0x302
{
    uint16_t u16CmdFB;
    uint16_t u16SN;
    uint32_t u32FaultBitFB;
} RegDef_TelescopicMBDFaultClearFB;

typedef struct//0x402
{
    uint16_t u16CmdFB;
    uint16_t u16SN;
    uint32_t u32FaultBitFB;
} RegDef_HuggingMBDFaultClearFB;

typedef struct//0x502
{
    uint16_t u16CmdFB;
    uint16_t u16SN;
    uint32_t u32FaultBitFB;
} RegDef_GlandMBDFaultClearFB;

typedef struct//0x203
{
    uint16_t u16Addr;                       //寄存器地址
    uint16_t u16SN;                         //指令SN
    E_PAPAM_OPT mod;                        //0-读，1-写（非伺服状态生效）
    uint16_t u16Value;                      //寄存器值
} RegDef_RotatingMBDParamFB;
//extern RegDef_RotatingParamFB Reg_RotatingParamFB;
typedef struct//0x303
{
    uint16_t u16Addr;                       //寄存器地址
    uint16_t u16SN;                         //指令SN
    E_PAPAM_OPT mod;                        //0-读，1-写（非伺服状态生效）
    uint16_t u16Value;                      //寄存器值
} RegDef_TelescopicMBDParamFB;

typedef struct//0x403
{
    uint16_t u16Addr;                       //寄存器地址
    uint16_t u16SN;                         //指令SN
    E_PAPAM_OPT mod;                        //0-读，1-写（非伺服状态生效）
    uint16_t u16Value;                      //寄存器值
} RegDef_HuggingMBDParamFB;

typedef struct//0x503
{
    uint16_t u16Addr;                       //寄存器地址
    uint16_t u16SN;                         //指令SN
    E_PAPAM_OPT mod;                        //0-读，1-写（非伺服状态生效）
    uint16_t u16Value;                      //寄存器值
} RegDef_GlandMBDParamFB;

typedef struct//0x204
{
    uint16_t u16SN;                         //指令SN
    uint16_t res1;                          //预留
    uint16_t res2;                          //预留
    uint16_t res3;                          //预留
} RegDef_RotatingMBDWatchDogFB;
//extern RegDef_RotatingWatchDogFB Reg_RotatingWatchDogFB;
typedef struct//0x304
{
    uint16_t u16SN;                         //指令SN
    uint16_t res1;                          //预留
    uint16_t res2;                          //预留
    uint16_t res3;                          //预留
} RegDef_TelescopicMBDWatchDogFB;

typedef struct//0x404
{
    uint16_t u16SN;                         //指令SN
    uint16_t res1;                          //预留
    uint16_t res2;                          //预留
    uint16_t res3;                          //预留
} RegDef_HuggingMBDWatchDogFB;

typedef struct//0x504
{
    uint16_t u16SN;                         //指令SN
    uint16_t res1;                          //预留
    uint16_t res2;                          //预留
    uint16_t res3;                          //预留
} RegDef_GlandMBDWatchDogFB;

typedef struct//0x205
{
    uint16_t u16Delay;                      //延时复位，单位mS
    uint16_t u16SN;                         //指令SN
    uint16_t u16Res1;                       //预留
    uint16_t u16Res2;                       //预留
} RegDef_RotatingMBDResetFB;
//extern RegDef_RotatingReset Reg_RotatingReset;
typedef struct//0x305
{
    uint16_t u16Delay;                      //延时复位，单位mS
    uint16_t u16SN;                         //指令SN
    uint16_t u16Res1;                       //预留
    uint16_t u16Res2;                       //预留
} RegDef_TelescopicMBDResetFB;

typedef struct//0x405
{
    uint16_t u16Delay;                      //延时复位，单位mS
    uint16_t u16SN;                         //指令SN
    uint16_t u16Res1;                       //预留
    uint16_t u16Res2;                       //预留
} RegDef_HuggingMBDResetFB;

typedef struct//0x505
{
    uint16_t u16Delay;                      //延时复位，单位mS
    uint16_t u16SN;                         //指令SN
    uint16_t u16Res1;                       //预留
    uint16_t u16Res2;                       //预留
} RegDef_GlandMBDResetFB;

typedef struct//0x210
{
    uint32_t u32Fault;                    //故障码
    uint32_t u32Warning;                  //警告码
} RegDef_RotatingMBDFaultStaFB;
//extern RegDef_RotatingFaultReset Reg_RotatingFaultReset;
typedef struct//0x310
{
    uint32_t u32Fault;                    //故障码
    uint32_t u32Warning;                  //警告码
} RegDef_TelescopicMBDFaultStaFB;

typedef struct//0x410
{
    uint32_t u32Fault;                    //故障码
    uint32_t u32Warning;                  //警告码
} RegDef_HuggingMBDFaultStaFB;

typedef struct//0x510
{
    uint32_t u32Fault;                    //故障码
    uint32_t u32Warning;                  //警告码
} RegDef_GlandMBDFaultStaFB;
extern RegDef_RotatingMBDFaultStaFB RegReal_RotatingMBDFaultStaFB;//0x210
extern RegDef_TelescopicMBDFaultStaFB RegReal_TelescopicMBDFaultStaFB;//0x310
extern RegDef_HuggingMBDFaultStaFB RegReal_HuggingMBDFaultStaFB;//0x410
extern RegDef_GlandMBDFaultStaFB RegReal_GlandMBDFaultStaFB;//0x510
typedef struct//0x211
{
    uint16_t u16SensorSta1;
    uint16_t u16SensorSta2;
    uint16_t u16SensorSta3;
    uint16_t u16SensorSta4;
} RegDef_RotatingMBDSensorStaFB;
//extern RegDef_RotatingMBDSensorStaFB Reg_RotatingMBDSensorStaFB;
typedef struct//0x311
{
    uint16_t u16SensorSta1;
    uint16_t u16SensorSta2;
    uint16_t u16SensorSta3;
    uint16_t u16SensorSta4;
} RegDef_TelescopicMBDSensorStaFB;

typedef struct//0x411
{
    uint16_t u16SensorSta1;
    uint16_t u16SensorSta2;
    uint16_t u16SensorSta3;
    uint16_t u16SensorSta4;
} RegDef_HuggingMBDSensorStaFB;

typedef struct//0x511
{
    uint16_t u16SensorSta1;
    uint16_t u16SensorSta2;
    uint16_t u16SensorSta3;
    uint16_t u16SensorSta4;
} RegDef_GlandMBDSensorStaFB;

typedef struct//0x212
{
    uint16_t u16RunningSta1;
    uint16_t u16RunningSta2;
    uint16_t u16RunningSta3;
    uint16_t u16RunningSta4;
} RegDef_RotatingMBDRunningSta1FB;
//extern RegDef_RotatingMBDRunningSta1FB Reg_RotatingMBDRunningSta1FB;
typedef struct//0x312
{
    uint16_t u16RunningSta1;
    uint16_t u16RunningSta2;
    uint16_t u16RunningSta3;
    uint16_t u16RunningSta4;
} RegDef_TelescopicMBDRunningSta1FB;

typedef struct//0x412
{
    uint16_t u16RunningSta1;
    uint16_t u16RunningSta2;
    uint16_t u16RunningSta3;
    uint16_t u16RunningSta4;
} RegDef_HuggingMBDRunningSta1FB;

typedef struct//0x512
{
    uint16_t u16RunningSta1;
    uint16_t u16RunningSta2;
    uint16_t u16RunningSta3;
    uint16_t u16RunningSta4;
} RegDef_GlandMBDRunningSta1FB;
extern int16_t m_nRotatingMBLife, m_nTelescopicMBLife, m_nHuggingMBLife, m_nGlandMBLife;
extern int8_t ResetCheck ( void );
extern void ParamRead_Write ( void );
typedef enum{
    ValveLiftCmd = 0,
    ValveClawCmd,
    ValveBowRiseCmd,
    ValveRotateCmd,
    ValveSideShiftCmd,
    ValveLiftRaderCmd,
    ValveSensorCaliCmd,
    ValveFaultClearCmd,
    ValveFaultEnableCmd,
    ValveCtrlState,
    ValveCmdNum,
    SteerPosCtrlCmd,
    SteerSensorCaliCmd,
    SteerFaultClearCmd,
    SteerFaultEnableCmd,
    SteerCtrlState,
    SteerCmdNum,
    LightCmd,
    RaderCmd,
    StepS_None,
}CanTask2_StepS;

typedef struct
{
    uint32_t ID;
    uint32_t Len;
    uint8_t Buff[CMD_FIFO_SIZE];
} UserFrame2_t;

typedef struct {
    uint16_t u16Dev_ID;     //路由表值
    uint16_t Opt;
    uint16_t u16Cnt;        //计数
    uint16_t u16Ack;        //数据
} BootRxStruct;
extern BootRxStruct BootRxHandle;

typedef struct
{
    uint16_t u16SN;                    //命令序号，ack回应使用
    uint16_t unEN;                       //使能
    uint16_t unLiftPos;                  //目标Lift高度 32767 = 6m
    uint16_t LiftMode;                   //升降模式：0是位置伺服，1是下降到底
    
} RegDef_ValveLiftCmd;
extern RegDef_ValveLiftCmd RegReal_ValveLiftCmd;  //寄存器0x100

typedef struct
{
    uint16_t u16SN;                    //命令序号，ack回应使用
    uint16_t unEN;                       //使能
    uint16_t unClawPos;                  //目标Claw位置
    uint16_t unRes;                      //预留
} RegDef_ValveClawCmd;
extern RegDef_ValveClawCmd RegReal_ValveClawCmd;  //寄存器0x101

typedef struct
{
    uint16_t u16SN;                    //命令序号，ack回应使用
    uint16_t unEN;                       //使能
    uint16_t unBowRisePos;               //目标位置
    uint16_t unRes;                      //预留
} RegDef_ValveBowRiseCmd;
extern RegDef_ValveBowRiseCmd RegReal_ValveBowRiseCmd;  //寄存器0x102

typedef struct
{
    uint16_t u16SN;                    //命令序号，ack回应使用
    uint16_t unEN;                       //使能
    uint16_t unRotatePos;                  //目标Rotate位置
    uint16_t unRes;                      //预留
} RegDef_ValveRotateCmd;
extern RegDef_ValveRotateCmd RegReal_ValveRotateCmd;  //寄存器0x103

typedef struct
{
    uint16_t u16SN;                    //命令序号，ack回应使用
    uint16_t unEN;                       //使能
    uint16_t unSideShiftPos;             //目标SideShift位置
    uint16_t SideMoveMode;                      //预留
} RegDef_ValveSideShiftCmd;                 //104寄存器
extern RegDef_ValveSideShiftCmd RegReal_ValveSideShiftCmd;  //寄存器0x104

typedef struct
{
    uint16_t u16SN;                    //命令序号，ack回应使用
    uint16_t unEN;                       //使能
    uint16_t unLiftRaderPos;             //目标LiftRader位置
    uint16_t unRes;                      //预留
} RegDef_ValveLiftRaderCmd;                 //105寄存器
extern RegDef_ValveLiftRaderCmd RegReal_ValveLiftRaderCmd;  //寄存器0x105

typedef struct
{
    uint16_t u16MotorCoderCaliSN;          //电机码盘标定SN
    uint16_t u16MotorCoderCaliCmd;         //电机码盘标定Cmd
    uint16_t u16WheelCoderCaliSN;          //后轮码盘标定SN
    uint16_t u16WheelCoderCaliCmd;         //后轮码盘标定Cmd
} RegDef_ValveSensorCaliCmd;
extern RegDef_ValveSensorCaliCmd RegReal_ValveSensorCaliCmd;  //寄存器0x106

typedef struct
{
    uint16_t u16FaultClearCmd;           //电机码盘标定
    uint16_t u16Res1;                    //预留
    uint16_t u16Res2;                    //预留
    uint16_t u16Res3;                    //预留
} RegDef_ValveFaultClearCmd;
extern RegDef_ValveFaultClearCmd RegReal_ValveFaultClearCmd;  //寄存器0x107

typedef struct
{
    uint16_t u16FaultClearCmd;           //电机码盘标定
    uint16_t u16Res1;                    //预留
    uint16_t u16Res2;                    //预留
    uint16_t u16Res3;                    //预留
} RegDef_ValveFaultEnableCmd;
extern RegDef_ValveFaultEnableCmd RegReal_ValveFaultEnableCmd;  //寄存器0x108

typedef struct
{
    uint16_t u16CtrlMode;                //控制模式
    uint16_t u16Res1;                    //预留
    uint16_t u16Res2;                    //预留
    uint16_t u16Res3;                    //预留
} RegDef_ValveCtrlState;
extern RegDef_ValveCtrlState RegReal_ValveCtrlState;  //寄存器0x109

typedef struct
{
    uint16_t u16Mode;                     //位置控制模式
    uint16_t u16SN;                       //位置控制SN
    uint16_t u16EN;                       //位置控制使能
    uint16_t u16PosRef;                   //位置给定值
} RegDef_SteerPosCtrlCmd;
extern RegDef_SteerPosCtrlCmd RegReal_SteerPosCtrlCmd;  //寄存器0x180

typedef struct
{
    uint16_t u16MotorCoderCaliSN;          //电机码盘标定SN
    uint16_t u16MotorCoderCaliCmd;         //电机码盘标定Cmd
    uint16_t u16WheelCoderCaliSN;          //后轮码盘标定SN
    uint16_t u16WheelCoderCaliCmd;         //后轮码盘标定Cmd
} RegDef_SteerSensorCaliCmd;
extern RegDef_SteerSensorCaliCmd RegReal_SteerSensorCaliCmd;  //寄存器0x181

typedef struct
{
    uint16_t u16FaultClearCmd;           //电机码盘标定
    uint16_t u16Res1;                    //左轮码盘标定
    uint16_t u16Res2;                    //右轮码盘标定
    uint16_t u16Res3;                    //预留
} RegDef_SteerFaultClearCmd;
extern RegDef_SteerFaultClearCmd RegReal_SteerFaultClearCmd;  //寄存器0x182

typedef struct
{
    uint16_t u16FaultClearCmd;           //电机码盘标定
    uint16_t u16CtrlMode;                //左轮码盘标定
    uint16_t u16Res2;                    //右轮码盘标定
    uint16_t u16Res3;                    //预留
} RegDef_SteerFaultEnableCmd;
extern RegDef_SteerFaultEnableCmd RegReal_SteerFaultEnableCmd;  //寄存器0x183

typedef struct
{
    uint16_t u16CtrlMode;                //控制模式
    uint16_t u16Res1;                    //预留
    uint16_t u16Res2;                    //预留
    uint16_t u16Res3;                    //预留
} RegDef_SteerCtrlState;
extern RegDef_SteerCtrlState RegReal_SteerCtrlState;  //寄存器0x184

typedef struct
{
    uint16_t unClampHight;               //抱夹高度
    uint16_t unClampOpenDegree;          //抱夹开度
    uint16_t unBowRiseDegree;            //俯仰位置
    uint16_t unRotateDegree;             //旋转位置
} RegDef_ValveSensorPos1FB;              //200寄存器
extern RegDef_ValveSensorPos1FB RegReal_ValveSensorPos1FB;

typedef struct
{
    uint16_t unSideShiftDegree;          //侧移位置
    uint16_t unLiftMotorHight;           //伺服升降位置
    uint16_t res1;                       //预留
    uint16_t res2;                       //预留
} RegDef_ValveSensorPos2FB;              //201寄存器

typedef struct
{
    uint16_t unLiftAck;                  //升降SN
    uint16_t unClawAck;                  //抱夹SN
    uint16_t unBowRiseAck;               //俯仰SN
    uint16_t unRotateAck;                //旋转SN
} RegDef_ValveCmd1SNFB;                  //202寄存器

typedef struct
{
    uint16_t unSideShiftAck;             //侧移SN
    uint16_t unLiftMotorAck;             //伺服升降SN
    uint16_t res1;                       //预留
    uint16_t res2;                       //预留
} RegDef_ValveCmd2SNFB;                  //203寄存器

typedef struct
{
    uint16_t unLiftCtrlMode;             //升降控制模式
    uint16_t unClawCtrlMode;             //开合控制模式
    uint16_t unBowRiseCtrlMode;          //俯仰控制模式
    uint16_t unRotateCtrlMode;           //旋转控制模式
} RegDef_ValveCtrlMode1FB;               //204寄存器

typedef struct
{
    uint16_t unSideShiftCtrlMode;        //侧移控制模式
    uint16_t unLiftMotorCtrlMode;        //伺服升降控制模式
    uint16_t res1;                       //预留
    uint16_t res2;                       //预留
} RegDef_ValveCtrlMode2FB;               //205寄存器

typedef struct
{
    uint16_t u16Valve1Flag;              //液压动作状态1，上传给上位机告知已经收到； bit/0~3/4~7/8~11/12~15,分别代表升降，抱夹，俯仰，旋转
    uint16_t u16Valve2Flag;              //液压动作状态2，上传给上位机告知已经收到； bit/0~3/4~7,分别代表侧移，伺服升降
    uint16_t u16ClawState;               //抱夹抱紧状态
    uint16_t u16SideShiftState;          //侧移停止状态，0:初始化或接收到新指令清零；1：到位停止；2：侧移到底停止；3：超时停止或指令错误
} RegDef_ValveState1FB;
extern RegDef_ValveState1FB RegReal_ValveState1FB;        //寄存器0x206

typedef struct
{
    uint16_t u16Lift_Pressure;           //升降油压
    uint16_t u16Claw_Pressure;           //抱夹油压
    uint16_t u16SideShift_Pressure1;     //侧移左油压
    uint16_t u16SideShift_Pressure2;     //侧移右油压
} RegDef_ValvePressureFB;                //207寄存器
extern RegDef_ValvePressureFB RegReal_ValvePressureFB;

typedef struct
{
    uint32_t u32FaultNum;               //故障编码
    uint32_t u32WarningNum;             //警告编码
} RegDef_ValveFaultCodeFB;
extern RegDef_ValveFaultCodeFB RegReal_ValveFaultCodeFB;        //寄存器0x208

typedef struct
{
    uint16_t u16ClawCaliState;           //开合码盘标定
    uint16_t u16RotateCaliState;         //旋转码盘标定
    uint16_t res1;                       //预留
    uint16_t res2;                       //预留
} RegDef_ValveCaliStateFB;               //209寄存器

typedef struct
{
    uint16_t u16FaultClear;              //故障清除
    uint16_t u16FaultShield;             //故障屏蔽
    uint16_t u16StaFlg;                  //复位次数
    uint16_t res1;                       //预留
} RegDef_ValveState2FB;                  //20A寄存器

typedef struct
{
    uint16_t u16Mode;                     //位置控制模式
    uint16_t u16SN;                       //位置控制SN
    uint16_t u16PosFbk;                   //位置反馈值
    uint16_t u16EndofCtrl;                //EndofCtrl状态
} RegDef_SteerPosCtrlFB;
extern RegDef_SteerPosCtrlFB RegReal_SteerPosCtrlFB;  //寄存器0x300

typedef struct
{
    uint16_t u16LeftEncoder;              //左后轮编码器值
    uint16_t u16RightEncoder;             //右后轮编码器值
    uint16_t u16Pressure;                 //转向油压
    uint16_t u16Res;                      //预留
} RegDef_SteerSensorDataFB;
extern RegDef_SteerSensorDataFB RegReal_SteerSensorDataFB;  //寄存器0x301

typedef struct
{
    uint32_t u32Fault;                    //故障码
    uint32_t u32Warning;                  //警告码
} RegDef_SteerFaultCodeFB;
extern RegDef_SteerFaultCodeFB RegReal_SteerFaultCodeFB;  //寄存器0x302

typedef struct
{
    uint16_t u16MotorCoderCaliACK;        //左后轮编码器值
    uint16_t u16MotorCoderCaliRes;             //右后轮编码器值
    uint16_t u16WheelCoderCaliACK;                 //转向油压
    uint16_t u16WheelCoderCaliRes;                      //预留
} RegDef_SteerSensorCaliFB;
extern RegDef_SteerSensorCaliFB RegReal_SteerSensorCaliFB;  //寄存器0x303

typedef struct
{
    uint16_t u16FaultClearFB;             //故障清除FB
    uint16_t u16FaultEnableFB;            //故障屏蔽EB
    uint16_t u16ResetCnt;                 //复位次数
    uint16_t u16Res;                      //预留
} RegDef_SteerFaultCtrlFB;
extern RegDef_RotatingMBDCtrlFB RegReal_RotatingMBDCtrlFB;//0x200
extern RegDef_TelescopicMBDCtrlFB RegReal_TelescopicMBDCtrlFB;//0x300
extern RegDef_HuggingMBDCtrlFB RegReal_HuggingMBDCtrlFB;//0x400
extern RegDef_GlandMBDCtrlFB RegReal_GlandMBDCtrlFB;//0x500
extern RegDef_RotatingMBDRunningSta1FB RegReal_RotatingMBDRunningSta2FB;//0x213
extern uint8_t u8BootFB_Flag;
//-------------------- public functions -------------------------------------
void CanTask2Process ( void );
void Can_Node_Receive ( void );
int Can2Task_Send ( uint32_t Cmd_ID );
void Can_Node_Receive ( void );
int8_t MotorCalaCheck ( void );
//故障清除
int8_t  FualtClear ( void );
#endif // _NodeProcess_H_

//-----------------------End of file------------------------------------------
/** @}*/






