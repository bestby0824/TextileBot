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
#ifndef _VCUProcess_H_
#define _VCUProcess_H_

#include "stdint.h"
#include "AngleMath.h"
#include "DataTypesDef.h"
#define CMD_FIFO_SIZE                512
//#define Equip_HyDriveBoard           0x01
//#define Equip_SteerDriveBoard        0x02

//#define Reg_ChassisParam             0x201
//#define Reg_ForkParam                0x301
//#define Reg_PartsAndValve1Param      0x401
//#define Reg_PartsAndValve2Param      0x501

//#define Reg_ForkLiftState1           0x181
//#define Reg_ForkLiftState2           0x281
//#define Reg_ForkLiftState3           0x381
//#define Reg_ForkLiftState4           0x481
//#define Reg_ForkLiftState5           0x182
//#define Reg_ForkLiftState6                      0x19E
//#define Reg_HARTCMD                             0x1A3//

/*textile*/
#define Step_AGVCtrlCmd1                        0x226
#define Step_AGVCtrlCmd2                        0x222
#define Step_AGVBMSCtrlCmd                      0x6AC
#define Step_AGVTurnCtrlCmd1                    0x201//10ms---100hz
#define Step_AGVTurnCtrlCmd2                    0x301

#define Step_AGVChargeSW                        0x1F0//0x101
#define Step_AGVBMSStaOrigDataFb                0x2F4
#define Step_AGVDataFb1                         0x1A6//20ms---50Hz
#define Step_AGVDataFb2                         0x1A7//20ms---50Hz
#define Step_AGVPumpCtrFb                       0x1A2//20ms---50Hz

#define Step_AGVTurnFb                          0x181//10ms---100hz
#define Step_RCCtrlCmdFb                        0x88//手柄
#define Step_RCWarningFb                        0x89
#define BrakeDeep       50

typedef struct
{
    uint32_t ID;
    uint32_t Len;
    uint8_t Buff[CMD_FIFO_SIZE];
} UserFrame_t;
extern UserFrame_t Tx_frame;

struct Reg_ChassisEnabled_Bits
{
    uint8_t AutoModeEnabled                 : 1; //自动模式使能：1有效，0无效 置1时，VCU响应切换到自动模式
    uint8_t WalkingEnabled                  : 1; //行走使能：1有效，0无效
    uint8_t BrakeEnabled                    : 1; //制动使能：1有效，0无效
    uint8_t SteeringEnabled                 : 1; //转向使能：1有效，0无效
    uint8_t ChargingEnabled                 : 1; //自动充电使能：1有效，0无效
    uint8_t ForwardEnabled                  : 1; //前进使能：1有效，0无效
    uint8_t ReverseEnabled                  : 1; //后退使能：1有效，0无效
    uint8_t ParkingBrakeEnabled             : 1; //驻车制动使能：1有效，0无效
};
typedef union
{
    uint8_t all;
    struct Reg_ChassisEnabled_Bits bit;
} Reg_ChassisEnabled_Byte;
typedef struct
{
    Reg_ChassisEnabled_Byte u8ChassisEnabled;   //底盘使能
    uint8_t u8SerServiceBrakes;                 //行车制动
    uint8_t u8SteerTargetAngle_Low;             //转向目标角度低字节
    uint8_t u8SteerTargetAngle_High;            //转向目标角度高字节
    uint8_t u8ChassisMotorTargetSpd_Low;        //行走电机目标转速低字节
    uint8_t u8ChassisMotorTargetSpd_High;       //行走电机目标转速高字节
    uint8_t u8ChassisMotorAcc;                  //行走电机加速率
    uint8_t u8ChassisMotorDec;                  //行走电机减速率
} RegDef_ChassisParam;
extern RegDef_ChassisParam RegReal_ChassisParam;//寄存器0x201

struct Reg_ForkEnabled_Bits
{
    uint8_t PumpMotorEnabled                : 1; //油泵电机使能：1使能，0禁止
    uint8_t PumpMotorBrake                  : 1; //油泵电机刹车：1有效，0无效
    uint8_t Rotate_or_Claw                  : 1; //旋转和抱夹切换：1抱夹，0旋转
    uint8_t res0                            : 1;                    
    uint8_t res1                            : 1;             
    uint8_t res2                            : 1;               
    uint8_t res3                            : 1;               
    uint8_t res4                            : 1;               
};
typedef union
{
    uint8_t all;
    struct Reg_ForkEnabled_Bits bit;
} Reg_ForkEnabled_Byte;
typedef struct
{
    Reg_ForkEnabled_Byte u8ForkEnabled;         //叉子使能
    uint8_t u8PumpMotorTargetSpd_Low;           //油泵电机目标转速低字节
    uint8_t u8PumpMotorTargetSpd_High;          //油泵电机目标转速高字节
    uint8_t u8BreakCurrent;                     //电电磁阀电流控制偏移量，-10 ~ 0，默认值10（10A，最大允许10A）
    uint8_t u8BreakSpeed;                       //电电磁阀速度控制偏移量，-90 ~ 10，默认值90（90%，最大允许100%）
    uint8_t u8PumpMotorAcc;                     //油泵电机加速率
    uint8_t u8PumpMotorDec;                     //油泵电机减速率
    uint8_t res2;                               //预留 
} RegDef_ForkParam;
extern RegDef_ForkParam RegReal_ForkParam;      //寄存器0x301

struct Reg_PartSwitch_Bits
{
    uint8_t TurnLeftLight                   : 1; //左向灯：1亮闪，0灭
    uint8_t TurnRightLight                  : 1; //右向灯：1亮闪，0灭
    uint8_t Headlights                      : 1; //前大灯：1亮，0灭
    uint8_t WidthIndicator                  : 1; //示宽灯：1亮，0灭
    uint8_t WarningLights                   : 1; //警示灯：1亮，0灭
    uint8_t Horn                            : 1; //喇叭：1鸣笛，0无效
    uint8_t ReversingLightsAndBeeps         : 1; //倒车灯及倒车蜂鸣：1有效，0无效
    uint8_t Wiper                           : 1; //雨刮器：1有效，0无效
};
typedef union
{
    uint8_t all;
    struct Reg_PartSwitch_Bits bit;
} Reg_PartSwitch_Byte;
typedef struct
{
    Reg_PartSwitch_Byte u8PartSwitch;           //设备开关
    uint8_t res0;                               //预留
    uint8_t LiftValveStartValue_Low;            //起升比例阀启动值低字节
    uint8_t LiftValveStartValue_High;           //起升比例阀启动值高字节
    uint8_t DropValveStartValue_Low;            //下降比例阀启动值低字节
    uint8_t DropValveStartValue_High;           //下降比例阀启动值高字节
    uint8_t ForwardValveStartValue_Low;         //前倾比例阀启动值低字节
    uint8_t ForwardValveStartValue_High;        //前倾比例阀启动值高字节 
} RegDef_PartsAndValve1Param;
extern RegDef_PartsAndValve1Param RegReal_PartsAndValve1Param;      //寄存器0x401

typedef struct
{
    uint8_t ReclineValveStartValue_Low;         //后仰比例阀启动值低字节
    uint8_t ReclineValveStartValue_High;        //后仰比例阀启动值高字节
    uint8_t ShiftLeftValveStartValue_Low;       //左移比例阀启动值低字节
    uint8_t ShiftLeftValveStartValue_High;      //左移比例阀启动值高字节
    uint8_t ShiftRightValveStartValue_Low;      //右移比例阀启动值低字节
    uint8_t ShiftRightValveStartValue_High;     //右移比例阀启动值高字节
    uint8_t res0;                               //预留
    uint8_t res1;                               //预留
} RegDef_PartsAndValve2Param;
extern RegDef_PartsAndValve2Param RegReal_PartsAndValve2Param;      //寄存器0x501

struct Reg_TravelMode_Bits
{
//    uint8_t AllowedAutoMode               : 1; //是否允许进入自动模式状态：1允许，0不允许
    uint8_t res                             : 1; //预留
    uint8_t ForkLiftMode                    : 1; //整车模式状态：1自动，0手动
    uint8_t ServiceBrakeMode                : 1; //行车制动状态：1制动，0释放
    uint8_t ParkingBrakeMode                : 1; //驻车制动状态：1释放，0制动
    uint8_t res0                            : 1; //预留
    uint8_t res1                            : 1; //预留
    uint8_t res2                            : 1; //预留
    uint8_t res3                            : 1; //预留
};
typedef union
{
    uint8_t all;
    struct Reg_TravelMode_Bits bit;
} Reg_TravelMode_Byte0;
struct Reg_CANConState_Bits
{
    uint8_t SmartDrivingState               : 1; //VCU连接智驾CAN连接状态：1连接，0未连接
    uint8_t ChassisMotorCtrlState           : 1; //VCU连接行走电控CAN连接状态：1连接，0未连接
    uint8_t SteerMotorCtrlState             : 1; //VCU连接转向电控CAN连接状态：1连接，0未连接
    uint8_t PumpMotorCtrlState              : 1; //VCU连接油泵电控CAN连接状态：1连接，0未连接
    uint8_t AbsoluteEncoderState            : 1; //VCU连接转向绝对值编码器CAN连接状态：1连接，0未连接
    uint8_t BMSState                        : 1; //VCU连接BMS CAN连接状态：1连接，0未连接
    uint8_t ElectricActuatorState           : 1; //VCU连接电动电磁阀CAN连接状态：1连接，0未连接
    uint8_t res0                            : 1; //预留
};
typedef union
{
    uint8_t all;
    struct Reg_CANConState_Bits bit;
} Reg_CANConState_Byte1;
struct Reg_BoundaryState_Bits
{
    uint8_t ForkLimitSignal                 : 1; //货叉上限位信号：1有，0无
    uint8_t LeftForkBarrierState            : 1; //左货叉障碍检测信号状态：1有效，0无效
    uint8_t RightForkBarrierState           : 1; //右货叉障碍检测信号状态：1有效，0无效
    uint8_t CargoHeartbeatState             : 1; //货物检测信号状态：1有效，0无效
    uint8_t EmergencyState                  : 1; //急停开关状态：1整车可以动作，0整车不可以动作
    uint8_t TouchedgeSignal                 : 1; //触边信号：1有，0无
    uint8_t res0                            : 1; //预留
    uint8_t res1                            : 1; //预留
};
typedef union
{
    uint8_t all;
    struct Reg_BoundaryState_Bits bit;
} Reg_BoundaryState_Byte2;
typedef struct
{
    Reg_TravelMode_Byte0 u8TravelMode;          //行驶模式
    Reg_CANConState_Byte1 u8CANConState;        //CAN连接状态
    Reg_BoundaryState_Byte2 u8BoundaryState;    //边界检测状态
    uint8_t u8ServiceBrakeFeedback;             //行车制动率百分比反馈：0-255对应0-100%
    uint8_t u8TiltedAngleFeedback_Low;          //倾斜电位器角度检测反馈0-5000对应0-5V 低字节
    uint8_t u8TiltedAngleFeedback_High;         //倾斜电位器角度检测反馈0-5000对应0-5V 高字节
    uint8_t u8SideshiftAngleFeedback_Low;       //侧移电位器角度检测反馈0-5000对应0-5V 低字节
    uint8_t u8SideshiftAngleFeedback_High;      //侧移电位器角度检测反馈0-5000对应0-5V 高字节
} RegDef_ForkLiftState1;
extern RegDef_ForkLiftState1 RegReal_ForkLiftState1;  //寄存器0x181

typedef struct
{
    uint8_t u8SteerAngleFeedback_Low;            //转向角度反馈-7000 ~ 7000 对应 -70°~70° 低字节
    uint8_t u8SteerAngleFeedback_High;           //转向角度反馈-7000 ~ 7000 对应 -70°~70° 高字节
    uint8_t res0;                               //预留
    uint8_t res1;                               //预留
    uint8_t u8ChassisMotorCtrlErr_Low;          //行走电控故障代码反馈 低字节
    uint8_t u8ChassisMotorCtrlErr_High;         //行走电控故障代码反馈 高字节
    uint8_t u8SteerMotorCtrlErr_Low;            //转向电控故障代码反馈 低字节
    uint8_t u8SteerMotorCtrlErr_High;           //转向电控故障代码反馈 高字节
} RegDef_ForkLiftState2;
extern RegDef_ForkLiftState2 RegReal_ForkLiftState2;  //寄存器0x281

typedef struct  //测试用int型，实际用uint(数据采集过程中发现问题，在数据处理脚本中打补丁，数据采集完之后修改)
{
    int8_t u8ChassisMotorSpd_Low;              //行走电机转速反馈低字节 -5000 ~ 5000 对应-5000 ~ 5000 rpm
    int8_t u8ChassisMotorSpd_High;             //行走电机转速反馈高字节 -5000 ~ 5000 对应-5000 ~ 5000 rpm
    int8_t u8ChassisMotorCurrent_Low;          //行走电机电流反馈低字节 –10000 – 10000对应 –1000 – 1000 A
    int8_t u8ChassisMotorCurrent_High;         //行走电机电流反馈高字节 –10000 – 10000对应 –1000 – 1000 A
    int8_t u8ChassisMotorTemp_Low;             //行走电机温度反馈低字节 –1000 – 3000对应 –100 – 300°C
    int8_t u8ChassisMotorTemp_High;            //行走电机温度反馈高字节 –1000 – 3000对应 –100 – 300°C
    int8_t u8ChassisMotorCtrlTemp_Low;         //行走电控温度反馈低字节 –1000 – 3000对应 –100 – 300°C
    int8_t u8ChassisMotorCtrlTemp_High;        //行走电控温度反馈高字节 –1000 – 3000对应 –100 – 300°C
} RegDef_ForkLiftState3;
extern RegDef_ForkLiftState3 RegReal_ForkLiftState3;  //寄存器0x381

typedef struct
{
    uint8_t u8PumpMotorSpd_Low;                 //油泵电机转速反馈低字节 -5000 ~ 5000 对应-5000 ~ 5000 rpm
    uint8_t u8PumpMotorSpd_High;                //油泵电机转速反馈高字节 -5000 ~ 5000 对应-5000 ~ 5000 rpm
    uint8_t u8PumpMotorCurrent_Low;             //油泵电机电流反馈低字节 –10000 – 10000对应 –1000 – 1000 A
    uint8_t u8PumpMotorCurrent_High;            //油泵电机电流反馈高字节 –10000 – 10000对应 –1000 – 1000 A
    uint8_t u8PumpMotorTemp_Low;                //油泵电机温度反馈低字节 –1000 – 3000对应 –100 – 300°C
    uint8_t u8PumpMotorTemp_High;               //油泵电机温度反馈高字节 –1000 – 3000对应 –100 – 300°C
    uint8_t u8PumpMotorCtrlTemp_Low;            //油泵电控温度反馈低字节 –1000 – 3000对应 –100 – 300°C
    uint8_t u8PumpMotorCtrlTemp_High;           //油泵电控温度反馈高字节 –1000 – 3000对应 –100 – 300°C
} RegDef_ForkLiftState4;
extern RegDef_ForkLiftState4 RegReal_ForkLiftState4;  //寄存器0x481

typedef struct
{
    uint8_t u8PumpMotorCtrlErr_Low;             //油泵电控故障代码反馈 低字节
    uint8_t u8PumpMotorCtrlErr_High;            //油泵电控故障代码反馈 高字节
    uint8_t res0;                               //预留
    uint8_t res1;                               //预留
    uint8_t u8VCUErrFeedback;                   //VCU系统故障反馈
    uint8_t res2;                               //预留
    uint8_t res3;                               //预留
    uint8_t res4;                               //预留
} RegDef_ForkLiftState5;
extern RegDef_ForkLiftState5 RegReal_ForkLiftState5;  //寄存器0x182

struct Reg_BatteryError1_Bits
{
    uint8_t OverVoltage                     : 1; //总压过高：0正常，1总压过高
    uint8_t OverRelease                     : 1; //单体过放：0正常，1单体过放
    uint8_t CommunicationLost               : 1; //通讯中断：0正常，1中断
    uint8_t UnderVoltage                    : 1; //单体欠压：0正常，1单体欠压
    uint8_t OverCurrent                     : 1; //过电流：0正常，1过电流
    uint8_t OverTemperature                 : 1; //过温保护：0正常，1过温保护
    uint8_t TempProtection                  : 1; //温度保护：0正常，1温度保护
    uint8_t ChargingConnection              : 1; //充电连接：0正常，1充电连接
};
typedef union
{
    uint8_t all;
    struct Reg_BatteryError1_Bits bit;
} Reg_BatteryError_Byte6;
struct Reg_BatteryError2_Bits
{
    uint8_t ForcedFullCharge                : 1; //强制满充：0正常，1车辆需求充电
    uint8_t OffPowerProtection              : 1; //断功率保护：0正常，1电池停止放电
    uint8_t res0                            : 1; //预留
    uint8_t res1                            : 1; //预留
    uint8_t res2                            : 1; //预留
    uint8_t res3                            : 1; //预留
    uint8_t res4                            : 1; //预留
    uint8_t res5                            : 1; //预留
};
typedef union
{
    uint8_t all;
    struct Reg_BatteryError2_Bits bit;
} Reg_BatteryError_Byte7;
typedef struct
{
    uint8_t u8BatteryVoltage_Low;                //电池总电压低字节 范围：0 ~10000 比例因子：0.1V/bit 实际量程：0～1000 V
    uint8_t u8BatteryVoltage_High;               //电池总电压高字节 范围：0 ~10000 比例因子：0.1V/bit 实际量程：0～1000 V
    uint8_t u8BatteryCurrent_Low;                //电池总电流低字节 范围：0 ~ 65535 偏移量：-32000 比例因子：0.1A/bit 实际量程：-3200A～3353.5 A
    uint8_t u8BatteryCurrent_High;               //电池总电流高字节 范围：0 ~ 65535 偏移量：-32000 比例因子：0.1A/bit 实际量程：-3200A～3353.5 A
    uint8_t u8SOC;                               //SOC 范围：0 ~ 250 比例因子：0.4%/bit 实际量程：0～100%
    uint8_t u8BatteryCapacity;                   //电池容量 范围：0 ~ 250 比例因子：5Ah/bit 实际量程：0～1250Ah
    Reg_BatteryError_Byte6 u8BatteryError1;      //电池故障信息
    Reg_BatteryError_Byte7 u8BatteryError2;      //电池故障信息
} RegDef_ForkLiftState6;
extern RegDef_ForkLiftState6 RegReal_ForkLiftState6;  //寄存器0x19E

typedef struct
{
    uint8_t u8HARTCMD_Forkside;                  //侧移指令，0x01:左移，0xFF:右移
    uint8_t u8HARTCMD_DriveMode;                   //行进指令，0x01：后退，0xFF：前进
    uint8_t u8HARTCMD_SteerAngle;                //角度指令，0x00-0xFF，舵轮转向角度（-90°-90°）
    uint8_t u8HARTCMD_EmergencyStop;                  //模式，bit1：急停开关
    uint8_t u8HARTCMD_ForkLift;                  //起升/下降，0x01：货叉下降，0xFF：货叉上升
    uint8_t u8HARTCMD_ForkClaw;                 //开合，0x01：开，0xFF：合
    uint8_t u8HARTCMD_ForkRotate;               //旋转，0x01：去180°，0xFF：回0°
    uint8_t u8HARTCMD_ForkBowrise;                //俯仰，0x01：货叉后移，0xFF：货叉前移
} RegDef_HARTCMD;
extern RegDef_HARTCMD RegReal_HARTCMD;                //寄存器0x1A3，手操器命令结构体

typedef struct
{
    uint8_t u8ForkLiftMode;                      //整车模式状态：1自动，0手动
    uint8_t u8ServiceBrakeMode;                  //行车制动状态：1制动，0释放
    uint8_t u8ParkingBrakeMode;                  //驻车制动状态：1释放，0制动
    uint8_t u8SmartDrivingState;                 //VCU连接智驾CAN连接状态：1连接，0未连接
    uint8_t u8ChassisMotorCtrlState;             //VCU连接行走电控CAN连接状态：1连接，0未连接
    uint8_t u8SteerMotorCtrlState;               //VCU连接转向电控CAN连接状态：1连接，0未连接
    uint8_t u8PumpMotorCtrlState;                //VCU连接油泵电控CAN连接状态：1连接，0未连接
    uint8_t u8AbsoluteEncoderState;              //VCU连接转向绝对值编码器CAN连接状态：1连接，0未连接
    uint8_t u8BMSState;                          //VCU连接BMS CAN连接状态：1连接，0未连接
    uint8_t u8ElectricActuatorState;             //VCU连接液压驱动板CAN连接状态：1连接，0未连接
    uint8_t u8ForkLimitSignal;                   //货叉上限位信号：1有，0无
    uint8_t u8LeftForkBarrierState;              //左货叉障碍检测信号状态：1有效，0无效
    uint8_t u8RightForkBarrierState;             //右货叉障碍检测信号状态：1有效，0无效
    uint8_t u8CargoHeartbeatState;               //货物检测信号状态：1有效，0无效
    uint8_t u8EmergencyState;                    //急停开关状态：1整车可以动作，0整车不可以动作
    uint8_t u8TouchedgeSignal;                   //触边信号：1有，0无
    uint8_t u8BrakeFeedback;                     //行车制动率百分比反馈：0-255对应0-100%
    uint16_t u16TiltedAngleFeedback;             //倾斜电位器角度检测反馈0-5000对应0-5V，单位mV
    uint16_t u16SideshiftAngleFeedback;          //侧移电位器角度检测反馈0-5000对应0-5V，单位mV    以上为0x181寄存器数据解析参数
    
    int16_t s16SteerAngleFeedback;               //转向角度反馈-7000 ~ 7000 对应 -70°~70°，单位0.01°
    uint16_t u16ChassisMotorCtrlErr;             //行走电控故障代码反馈
    uint16_t u16SteerMotorCtrlErr;               //转向电控故障代码反馈    以上为0x281寄存器数据解析参数
    
    int16_t s16ChassisMotorSpd;                  //行走电机转速反馈 -5000 ~ 5000 对应-5000 ~ 5000 rpm
    int16_t s16ChassisMotorCurrent;              //行走电机电流反馈 –10000 – 10000对应 –1000 – 1000 A，单位0.1A
    int16_t s16ChassisMotorTemp;                 //行走电机温度反馈 –1000 – 3000对应 –100 – 300°C，单位0.1℃
    int16_t s16ChassisMotorCtrlTemp;             //行走电控温度反馈 –1000 – 3000对应 –100 – 300°C，单位0.1℃    以上为0x381寄存器数据解析参数
    
    int16_t s16PumpMotorSpd;                     //油泵电机转速反馈 -5000 ~ 5000 对应-5000 ~ 5000 rpm
    int16_t s16PumpMotorCurrent;                 //油泵电机电流反馈 –10000 – 10000对应 –1000 – 1000 A，单位0.1A
    int16_t s16PumpMotorTemp;                    //油泵电机温度反馈 –1000 – 3000对应 –100 – 300°C，单位0.1℃
    int16_t s16PumpMotorCtrlTemp;                //油泵电控温度反馈 –1000 – 3000对应 –100 – 300°C，单位0.1℃    以上为0x481寄存器数据解析参数
    
    uint16_t u16PumpMotorCtrlErr;                //油泵电控故障代码反馈
    uint8_t u8VCUSysErr;                         //VCU系统故障反馈    以上为0x182寄存器数据解析参数
    
    uint16_t u16BatteryVoltage;                  //电池总电压 范围：0 ~10000 比例因子：0.1V/bit 实际量程：0～1000 V
    int16_t s16BatteryCurrent;                   //电池总电流 范围：0 ~ 65535 偏移量：-32000 比例因子：0.1A/bit 实际量程：-3200A～3353.5 A
    uint8_t u8BatterySOC;                        //SOC 范围：0 ~ 250 比例因子：0.4%/bit 实际量程：0～100%
    uint8_t u8BatteryCapacity;                   //电池容量 范围：0 ~ 250 比例因子：5Ah/bit 实际量程：0～1250Ah
    uint8_t u8OverVoltage;                       //总压过高：0正常，1总压过高
    uint8_t u8OverRelease;                       //单体过放：0正常，1单体过放
    uint8_t u8CommunicationLost;                 //通讯中断：0正常，1中断
    uint8_t u8UnderVoltage;                      //单体欠压：0正常，1单体欠压
    uint8_t u8OverCurrent;                       //过电流：0正常，1过电流
    uint8_t u8OverTemperature;                   //过温保护：0正常，1过温保护
    uint8_t u8TempProtection;                    //温度保护：0正常，1温度保护
    uint8_t u8ChargingConnection;                //充电连接：0正常，1充电连接
    uint8_t u8ForcedFullCharge;                  //强制满充：0正常，1车辆需求充电
    uint8_t u8OffPowerProtection;                //断功率保护：0正常，1电池停止放电    以上为0x19E寄存器数据解析参数
} VCU_ParamStruct;
extern VCU_ParamStruct VCU_ParamHandle;          //VCU数据解析结构体

typedef struct
{
    uint8_t u8StartOfTransfer;                   //输出传输开始标志
    uint8_t u8EndOfTransfer;                     //输出传输结束标志
    uint8_t u8Toggle;                            //输出传输切换位标志
    uint8_t u8TransferID;                        //数据传输当前包数
} CanTransferTypeDef;

typedef struct
{
    _iq LiftPos;
    _iq ClawPos;
    _iq BowRisePos;
    _iq RotatePos;
    _iq SidePos;
    uint8_t LiftState;
    uint8_t ClawState;
    uint8_t RowRiseState;
    uint8_t RotateState;
    uint8_t SideState;
    
}ForkLiftPos;

typedef struct
{
    uint16_t SideMoveRightCnt;
    uint16_t SideMoveLeftCnt;
    uint16_t LiftUpCnt;
    uint16_t LiftDownCnt;
    uint16_t ClawOpenCnt;
    uint16_t ClawClosCnt;
    uint16_t BowRiseForCnt;
    uint16_t BowRiseRevCnt;
    uint16_t Rotate_0_Cnt;
    uint16_t Rotate_180_Cnt;
}RCCnt;
/*textile*/
struct AGVCtrlCmd1_Byte5
{
    uint8_t up                              : 1; //1:上升
//    uint8_t forward                         : 1; //1:前移
//    uint8_t backward                        : 1; //1:后移
    uint8_t res1                            : 7; //预留
};
struct TurnStaWord
{
    uint16_t ReadyPowerOn           : 1; //准备上电
    uint16_t PowerOn                : 1; //已上电
    uint16_t MotorEN                : 1; //使能
    uint16_t MotorErr               : 1; //故障
    
    uint16_t ForbiddenVoltOut       : 1; //禁止输出电压
    uint16_t RapidStop              : 1; //快速停止
    uint16_t ForbiddenPowerOn       : 1;//上电禁止
    uint16_t MotorWarning           : 1;//警告
    
    uint16_t res                    : 1;//保留
    uint16_t RemoteCtrl             : 1;//远程控制
    uint16_t PosReached             : 1;//目标到位
    uint16_t InterLimitActivation   : 1;//内部限位激活
    
    uint16_t ImpulseResponse        : 1;//脉冲响应
    uint16_t TrackingErr            : 1;//跟随误差
    uint16_t ElecmagExcitation      : 1;//找到电磁励磁
    uint16_t OriginLocated          : 1;//找到原点
};
typedef union
{
    uint16_t     all;
    struct      TurnStaWord   bit;
} TurnStaWord_REG;

typedef union
{
    uint8_t all;
    struct AGVCtrlCmd1_Byte5 bit;
} Reg_AGVCtrlCmd1_Byte5;

typedef struct
{
    int16_t s16WalkSpd;         //行走速度： 1=1rpm,-32767-32767
    uint8_t u8Res3;                    //预留1
    uint8_t u8Res4;                    //预留2
    uint8_t EMVCtrl;                   //电磁阀控制
    uint8_t u8WalkDnPV;                //行走下降比例阀---0~200：0-2000mA
    uint8_t u8Res7;                    //预留3
    uint8_t u8Res8;                    //预留4
} RegDef_Reg_AGVCtrlCmd1;
extern RegDef_Reg_AGVCtrlCmd1 Reg_AGVCtrlCmd1;
typedef struct
{
    uint8_t u8PumpSpd;                 //泵速度： 1-255对应0-100%
    uint8_t u8Res2;                    //预留1
    uint8_t u8Res3;                    //预留2
    uint8_t u8Res4;                    //预留3
    uint8_t u8Res5;                    //预留4
    uint8_t u8Res6;                    //预留5
    uint8_t u8WalkAccTime;             //行走加速时间：0-255 对应0-25.5S 
    uint8_t u8WalkDecTime;             //行走减速时间：0-255 对应0-25.5S 
} RegDef_Reg_AGVCtrlCmd2;
extern RegDef_Reg_AGVCtrlCmd2 Reg_AGVCtrlCmd2;
typedef struct
{
    uint8_t u8AGVSta;                  //Agv状态：0-静置；1-放电；2-充电
    uint8_t u8InquiryBMSSta;           //查询位
    uint8_t u8ChargerCtrl;             //充电机控制位: 0XA1-自动充电BMS关闭输出，0XA2-自动充电BMS开启充电，0XA3-手动充电BMS开启充电
    uint8_t u8Res4;                    //预留1
    uint8_t u8Res5;                    //预留2
    uint8_t u8Res6;                    //预留3
    uint8_t u8Res7;                    //预留4
    uint8_t u8Res8;                    //预留5
}RegDef_AGVBMSCtrlCmd;
extern RegDef_AGVBMSCtrlCmd Reg_AGVBMSCtrlCmd;

typedef struct
{
    uint8_t Staflag;
    uint8_t Soc;
    uint8_t CycleTimes[2];                  // 
    uint8_t CurrSum[2];                     //-30000~35535(偏移)  *0.1A               
    uint8_t MaxTemp;                        //最高温度  -40-215
    uint8_t u8FaultCode;                    //故障码
}RegDef_AGVBMSStaOrigDataFb;
extern RegDef_AGVBMSStaOrigDataFb Reg_AGVBMSStaOrigDataFb;

typedef struct
{
    int16_t s16WalkSpd;         //行走速度： 1=1rpm,-32767-32767
    uint8_t u8Res3;                    //预留1
    uint8_t u8Res4;                    //预留2
    uint8_t Soc;
    uint8_t u8WalkFaultCode;           //行走故障码
    uint8_t u8SteeringFaultCode;       //转向故障码
    uint8_t SignalFb;                  //信号反馈   
}RegDef_AGVDataFb1;
extern RegDef_AGVDataFb1 Reg_AGVDataFb1;
typedef struct
{
    int32_t s32WalkCnt;             //行走编码器计数,正转：+，反转：-
    uint8_t SignalFb;                  //信号反馈，bit0：1触发 T22前进开关，bit1：1触发T33后退开关
    uint8_t u8Res6;                    //预留1
    uint8_t u8Res7;                    //预留2
    uint8_t u8Res8;                    //预留3
}RegDef_AGVDataFb2;
extern RegDef_AGVDataFb2 Reg_AGVDataFb2;

typedef struct
{
    int16_t s16PumpSpd;         //泵速度： 1=1rpm,0-32767
    uint8_t u8WalkFaultCode;           //泵控故障码
    int16_t s16WalkCurrFb;      //行走控制器电流反馈 1=1A,0~32767
    int16_t s16PumpCurrFb;      //泵控控制器电流反馈 1=1A,0~32767
    uint8_t u8Res8;                    //预留1
}RegDef_AGVPumpCtrFb;
extern RegDef_AGVPumpCtrFb Reg_AGVPumpCtrFb;
typedef struct
{
    S32WordDef s32MotorPosCmd;//
    uint8_t u8WorkMode;     
    U16HalfWordDef u16CtrlWord; 
}RegDef_Reg_AGVTurnCtrlCmd1;

typedef struct
{
    int32_t s32SpdLpSpdCmd;
    int32_t s32PosLpSpdCmd;
}RegDef_AGVTurnCtrlCmd2;

typedef struct
{
    int32_t s32MotorPosFb;
    TurnStaWord_REG u16StaWord;
}RegDef_AGVTurnFb;
typedef struct
{
    uint8_t u8SwCmd;
    uint8_t u8Data[7];
}RegDef_AGVChargeSWCmd;

typedef struct
{
    uint8_t u8SwStaFb;
    uint8_t u8Data[7];
}RegDef_AGVChargeSWFb;

typedef struct
{
    uint8_t u8WalkCmd;         
    uint8_t u8TurnCmd;                    //
    uint8_t u8LiftCmd;                    //
    uint8_t u8RotatingCmd;
    uint8_t u8TelescopicCmd;            //
    uint8_t u8HuggingCmd;               //
    uint8_t u8GlandCmd;                  //  
    uint8_t u8EmergencyStopCmd;
}RegDef_RCCtrlCmdFb;

typedef struct
{
    uint8_t FaultCode;
    uint8_t res1;
    uint8_t res2;
    uint8_t res3;
    uint8_t res4;
    uint8_t res5;
    uint8_t res6;
    uint8_t res7;
}RegDef_RCWarningFb;

extern  RegDef_RCCtrlCmdFb Reg_RCCtrlCmdFb;
extern  RegDef_RCWarningFb Reg_RCWarningFb;

extern RegDef_AGVChargeSWFb Reg_AGVChargeSWFb;
extern RegDef_AGVPumpCtrFb Reg_AGVPumpCtrFb;
extern RegDef_Reg_AGVTurnCtrlCmd1 Reg_AGVTurnCtrlCmd1;
extern RegDef_AGVTurnCtrlCmd2 Reg_AGVTurnCtrlCmd2;
extern RegDef_AGVChargeSWCmd Reg_AGVChargeSWCmd;
extern uint8_t AGVTurnCtrlCmdSendFlag1, AGVTurnCtrlCmdSendFlag2;
extern RegDef_AGVTurnFb Reg_AGVTurnFb;
/**/
extern CanTransferTypeDef CanTransferStruct;    //CAN传输辅助字段
extern RegDef_HARTCMD RegReal_HARTCMD;                //寄存器0x1A3，手操器指令
extern int16_t VCULife;
void Can_VCU_Receive ( void );
void CanTaskProcess ( void );
void Hart_SolenoidCmd ( void);     //遥控液压驱动板指令
void Hart_SteerCmd ( void);         //遥控转向指令读取
int int32DataSat(int data,int Max,int Min);
void Hart_SolenoidCmdInit ( void);     //遥控液压驱动板指令初始化

#endif

    






