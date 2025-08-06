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
#ifndef _DataProcess__H_
#define _DataProcess__H_
//-------------------- include files ----------------------------------------
#include "Eeprom.h"
#include "MODBusCRC.h"
#include "Dido.h"
//-------------------- public definitions -----------------------------------
//typedef struct {
////-------------------- 310-313寄存器，版本相关参数 --------------------------
//    uint16_t Reg_Tab_Ver;                   //寄存器表版本
//    uint16_t Reg_Tab_Len;                   //寄存器表长度(参数个数)
//    uint16_t Reg_Dev_Ver;                   //设备类型编码
//    uint16_t Reg_HW_Ver;                    //硬件版本，形式：V1.0

//    uint32_t Reg_SN;                        //序列号
//    uint32_t Reg_SW_Ver;                    //当前软件版本

//    uint32_t Reg_SW_NewApp;                 //NewApp软件版本
//    uint32_t Reg_SW_Factory;                //出厂软件版本

//    uint32_t Reg_SW_Boot;                   //boot软件版本
//    uint32_t Reg_Password;                  //写保护密码(固化到代码中)

////-------------------- 320-322寄存器，电机控制相关参数（不可修改） --------------------------
//    uint16_t Reg_PolePairs;                 //磁极对数
//    uint16_t Reg_RatedCurrent;              //额定电流,单位:0.01A
//    uint16_t Reg_NoLoadSpeed;               //空载转速,单位:1rpm
//    uint16_t Reg_RatedVoltage;              //额定电压,单位0.01V

//    uint16_t Reg_MotorAxleOffset;           //电机出轴编码器OFFSET,第一次上动力电标定
//    int16_t Reg_ReverseFlag;                //正反转标志,第一次上动力电标定,1:电角度与码盘旋转方向相同,-1:电角度与码盘旋转方向相反,0:未标定
//    uint16_t u16MotorRes2;                  //预留
//    uint16_t u16MotorRes3;                  //预留

//    int32_t Reg_SampleMaxCur;               //AD采样最大电流,根据硬件电路而定，_IQ(1)表示1A
//    uint16_t u16MotorRes4;                  //预留
//    uint16_t u16MotorRes5;                  //预留

////-------------------- 330-333寄存器，电机控制相关参数（可修改） --------------------------
//    uint32_t Reg_PI_POS_KP;                 //位置环KP
//    uint32_t Reg_PI_POS_KI;                 //位置环KI

//    uint32_t Reg_PI_SPD_KP;                 //速度环KP
//    uint32_t Reg_PI_SPD_KI;                 //速度环KI

//    uint16_t Reg_PI_ID_KP;                  //电流环(ID)KP
//    uint16_t Reg_PI_ID_KI;                  //电流环(ID)KI
//    uint16_t Reg_PI_IQ_KP;                  //电流环(IQ)KP
//    uint16_t Reg_PI_IQ_KI;                  //电流环(IQ)KI

//    uint16_t Reg_AngleMax;                  //位置环输入最大值
//    uint16_t Reg_SpeedMax;                  //速度环输入最大值(位置环输出最大值),单位:RPM
//    uint16_t Reg_CurrentMax;                //电流环输入最大值(速度环输出最大值),单位:_IQ(1):1A
//    uint16_t Reg_IdIqMax;                   //Id/Iq输入最大值(电流环输出最大值)

////-------------------- 340寄存器，其他参数（可修改） --------------------------
//    uint32_t Reg_FaultCheckEnable;          //故障检测使能标志,0:禁使能,1:使能
//    uint16_t u16MotorRes6;                  //预留
//    uint16_t u16MotorRes7;                  //预留

////-------------------- 350-253寄存器，其他相关参数 -------------------------
//    uint16_t Reg_Res0;                      //预留0
//    uint16_t Reg_Res1;                      //预留1
//    uint16_t Reg_Res2;                      //预留2
//    uint16_t Reg_Res3;                      //预留3

//    uint16_t Reg_Res4;                      //预留4
//    uint16_t Reg_Res5;                      //预留5
//    uint16_t Reg_Res6;                      //预留6
//    uint16_t Reg_Res7;                      //预留7

//    uint16_t Reg_Res8;                      //预留8
//    uint16_t Reg_Res9;                      //预留9
//    uint16_t Reg_Res10;                     //预留10
//    uint16_t Reg_Res11;                     //预留11

//    uint16_t Reg_Res12;                     //预留12
//    uint16_t Reg_Res13;                     //预留13
//    uint16_t Reg_Res14;                     //预留14
////    uint16_t Reg_Res15;                     //预留15

//    uint16_t Reg_CRC ;                      //CRC校验
//} ParamStruct_Steer;
//extern ParamStruct_Steer ParamHandle_Steer;
//typedef struct {
//    uint16_t Reg_Tab_Ver ;                  //寄存器表版本
//    uint16_t Reg_Tab_Len ;                  //寄存器表长度(变量个数)
//    uint16_t Reg_Dev_Ver ;                  //设备类型编码
//    uint16_t Reg_HW_Ver ;                   //硬件版本
//    uint32_t Reg_SN ;                       //序列号
//    uint32_t Reg_SW_Ver ;                   //
//    uint32_t Reg_SW_NewApp;                 //NewApp软件版本
//    uint32_t Reg_SW_Factory;                //出厂软件版本
//    uint32_t Reg_SW_Boot;                   //boot软件版本
//    uint32_t Reg_Password ;                 //
//    uint16_t Reg_Mode ;                     //工作模式：0:车上    3:夹子上
//    uint16_t Reg_CRC ;                      //
//} ParamStruct_Hub485;

//typedef struct {
////-------------------- 210-213寄存器，版本相关参数 --------------------------
//    uint16_t Reg_Tab_Ver;                   //寄存器表版本
//    uint16_t Reg_Tab_Len;                   //寄存器表长度(参数个数)
//    uint16_t Reg_Dev_Ver;                   //设备类型编码
//    uint16_t Reg_HW_Ver;                    //硬件版本，形式：V1.0

//    uint32_t Reg_SN;                        //序列号
//    uint32_t Reg_SW_Ver;                    //当前软件版本

//    uint32_t Reg_SW_NewApp;                 //NewApp软件版本
//    uint32_t Reg_SW_Factory;                //出厂软件版本

//    uint32_t Reg_SW_Boot;                   //boot软件版本
//    uint32_t Reg_Password;                  //写保护密码(固化到代码中)

////-------------------- 220-225寄存器，升降相关参数 --------------------------
//    uint32_t Reg_Lift_SoftMax;              //升降高度软件最大值，65535代表6米
//    uint16_t Reg_Lift_SoftMin;              //升降高度软件最小值
//    uint16_t Reg_LiftPosOffSet;             //升降offset，最低位置0对应的拉线数据

//    uint32_t Reg_Lift_MechMax;              //升降位置机械最大值
//    uint16_t Reg_Lift_MechMin;              //升降位置机械最小值
//    uint16_t u16LiftRes0;                   //升降预留0

//    uint16_t Reg_Lift_PosAllowErr;          //升降位置死区
//    uint16_t Reg_Lift_Pressure;             //升降油压检测阈值
//    uint16_t u16LiftRes1;                   //升降预留1
//    uint16_t u16LiftRes2;                   //升降预留2

//    uint32_t Reg_Lift_PosKp;                //升降位置环PID比例系数
//    uint16_t Reg_Lift_PosKi;                //升降位置环PID积分系数
//    uint16_t Reg_Lift_PosKd;                //升降位置环PID微分系数

//    uint16_t Reg_Lift_PosUmax;              //升降位置环PID最大值限幅
//    uint16_t Reg_Lift_PosUmin;              //升降位置环PID最小值限幅
//    uint16_t Reg_Lift_SpdKp;                //升降速度环PID比例系数
//    uint16_t Reg_Lift_SpdKi;                //升降速度环PID积分系数

//    uint16_t Reg_Lift_SpdKd;                //升降速度环PID微分系数
//    uint16_t Reg_Lift_SpdUmax;              //升降速度环PID最大值限幅
//    uint16_t Reg_Lift_SpdUmin;              //升降速度环PID最小值限幅
//    uint16_t u16LiftRes3;                   //升降预留3

////-------------------- 230-233寄存器，开合相关参数 --------------------------
//    uint16_t Reg_Claw_SoftMax;              //开合角度软件最大值，32767代表360°
//    uint16_t Reg_Claw_SoftMin;              //开合角度软件最小值
//    uint16_t Reg_Claw_MechMax;              //开合角度机械最大值
//    uint16_t Reg_Claw_MechMin;              //开合角度机械最小值

//    uint16_t Reg_Claw_PosAllowErr;          //抱夹位置死区
//    uint16_t Reg_Claw_PressureErr;          //抱夹油压检测阈值
//    uint16_t u16ClawRes0;                   //升降预留0
//    uint16_t u16ClawRes1;                   //升降预留1

//    uint32_t Reg_Claw_Kp;                    //抱夹PID比例系数
//    uint16_t Reg_Claw_Ki;                   //抱夹PID积分系数
//    uint16_t Reg_Claw_Kd;                   //抱夹PID微分系数

//    uint16_t Reg_Claw_Umax;                 //抱夹位置环PID最大值限幅
//    uint16_t Reg_Claw_Umin;                 //抱夹位置环PID最小值限幅
//    uint16_t u16ClawRes2;                   //升降预留2
//    uint16_t u16ClawRes3;                   //升降预留3

////-------------------- 240-243寄存器，俯仰相关参数 --------------------------
//    uint16_t Reg_BowRise_SoftMax;           //俯仰长度软件最大值，65535代表1米
//    uint16_t Reg_BowRise_SoftMin;           //俯仰长度软件最小值
//    uint16_t Reg_BowRise_MechMax;           //俯仰长度机械最大值
//    uint16_t Reg_BowRise_MechMin;           //俯仰长度机械最小值

//    uint16_t Reg_BowRise_PosAllowErr;       //俯仰位置死区
//    uint16_t u16BowRiseRes0;                //俯仰预留0
//    uint16_t u16BowRiseRes1;                //俯仰预留1
//    uint16_t u16BowRiseRes2;                //俯仰预留2

//    uint32_t Reg_BowRise_Kp;                 //俯仰PID比例系数
//    uint16_t Reg_BowRise_Ki;                //俯仰PID积分系数
//    uint16_t Reg_BowRise_Kd;                //俯仰PID微分系数 

//    uint16_t Reg_BowRise_Umax;              //俯仰PID输出最大限幅
//    uint16_t Reg_BowRise_Umin;              //俯仰PID输出最小限幅
//    uint16_t u16BowRiseRes3;                //俯仰预留3
//    uint16_t u16BowRiseRes4;                //俯仰预留4

////-------------------- 250-254寄存器，旋转相关参数 --------------------------
//    uint32_t Reg_Rotate_SoftMax;             //旋转角度软件最大值，32767代表360°
//    uint32_t Reg_Rotate_SoftMin;             //旋转角度软件最小值

//    uint32_t Reg_Rotate_MechMax;             //旋转角度机械最大值
//    uint32_t Reg_Rotate_MechMin;             //旋转角度机械最小值

//    uint32_t Reg_RotatePosOffSet;            //旋转offset
//    uint16_t Reg_Rotate_PosAllowErr;        //旋转位置死区
//    uint16_t u16RotateRes0;                 //旋转预留0

//    uint32_t Reg_Rotate_Kp;                  //旋转PID比例系数
//    uint16_t Reg_Rotate_Ki;                 //旋转PID积分系数
//    uint16_t Reg_Rotate_Kd;                 //旋转PID微分系数

//    uint32_t Reg_Rotate_Umax;               //旋转位置环最大值
//    uint32_t Reg_Rotate_Umin;               //旋转位置环最小值
//    uint16_t u16RotateRes1;                 //旋转预留1
//    uint16_t u16RotateRes2;                 //旋转预留2

////--------------------- 260寄存器，电磁阀相关参数 ---------------------------
//    uint32_t Reg_ValveIp_Kp;                 //2~4号推杆电机位置环PID比例系数
//    uint32_t Reg_ValveIp_Ki;                 //2~4号推杆电机位置环PID比例系数

////--------------------- 270寄存器，其他相关参数 ---------------------------
//    uint32_t ValveErrEnable;               //液压驱动板故障码使能

////------------------- 280-283寄存器，其他相关参数 -------------------------
//    uint16_t Reg_Res0;                      //预留0
//    uint16_t Reg_Res1;                      //预留1
//    uint16_t Reg_Res2;                      //预留2
//    uint16_t Reg_Res3;                      //预留3

//    uint16_t Reg_Res4;                      //预留4
//    uint16_t Reg_Res5;                      //预留5
//    uint16_t Reg_Res6;                      //预留6
//    uint16_t Reg_Res7;                      //预留7

//    uint16_t Reg_Res8;                      //预留8
//    uint16_t Reg_Res9;                      //预留9
//    uint16_t Reg_Res10;                     //预留10
//    uint16_t Reg_Res11;                     //预留11

//    uint16_t Reg_Res12;                      //预留12
//    uint16_t Reg_Res13;                      //预留13
//    uint16_t Reg_Res14;                      //预留14
////    uint16_t Reg_Res15;                      //预留15

////    uint16_t Reg_Res16;                      //预留16
////    uint16_t Reg_Res17;                      //预留17
////    uint16_t Reg_Res18;                      //预留18
////    uint16_t Reg_Res19;                      //预留19
////    
////    uint16_t Reg_Res20;                      //预留20
////    uint16_t Reg_Res21;                      //预留21
////    uint16_t Reg_Res22;                      //预留22
////    uint16_t Reg_Res23;                      //预留23
////    
////    uint16_t Reg_Res24;                      //预留24
////    uint16_t Reg_Res25;                      //预留25
////    uint16_t Reg_Res26;                      //预留26
////    uint16_t Reg_Res27;                      //预留27
////    
////    uint16_t Reg_Res28;                      //预留28
////    uint16_t Reg_Res29;                      //预留29
////    uint16_t Reg_Res30;                      //预留30
////    uint16_t Reg_Res31;                      //预留31

//    uint16_t Reg_CRC;                       //参数表校验
//} ParamStruct_Solenoid;

typedef struct {
    uint16_t Reg_Tab_Ver ;                  //寄存器表版本
    uint16_t Reg_Tab_Len ;                  //寄存器表长度(变量个数)
    uint16_t Reg_Dev_Ver ;                  //设备类型编码
    uint16_t Reg_HW_Ver ;                   //硬件版本
    uint32_t Reg_SN ;                       //序列号
    uint32_t Reg_SW_Ver ;                   //
    uint32_t Reg_SW_NewApp;                 //NewApp软件版本
    uint32_t Reg_SW_Factory;                //出厂软件版本
    uint32_t Reg_SW_Boot;                   //boot软件版本
    uint32_t Reg_Password ;                 //
    uint32_t Reg_WirelessBDSN ;             //无线控制板序列号
    uint16_t Reg_PairingFlag;               //无线控制板配对标志
    uint16_t Reg_WLSpdLimit;                //无线控制板速度限制
    uint16_t Reg_PCDogTimeOut ;             //上位机心跳超时
    uint16_t Reg_BD1DogTimeOut ;            //旋转驱动板心跳超时
    uint16_t Reg_BD2DogTimeOut ;            //伸缩驱动板心跳超时
    uint16_t Reg_BD3DogTimeOut ;            //环抱驱动板心跳超时
    uint16_t Reg_BD4DogTimeOut ;            //抬压驱动板心跳超时
    uint16_t Reg_VCUDogTimeOut ;            //VCU心跳超时
    uint16_t Reg_RadarDogTimeOut ;            //雷达心跳超时
    uint16_t Reg_PCWalkAccTime ;            //PC配置行走加速时间
    uint16_t Reg_PCWalkDecTime ;            //PC配置行走减速时间
    uint16_t Reg_WLWalkAccTime ;            //WL配置行走加速时间
    uint16_t Reg_WLWalkDecTime ;            //WL配置行走减速时间
    uint16_t Reg_DrawWireDogTimeOut;        //拉线心跳超时
    uint16_t Reg_OverHeat;                  //
    uint16_t res;         //拉线心跳超时
    uint16_t Reg_GasSpeedBound;             //高低速度油门给定分界值，_IQ表示
    uint16_t Reg_HighSpdGasMax;             //高速油门给定量，_IQ表示
    uint16_t Reg_LowSpdGasMax;              //低速油门给定量，_IQ表示
    uint16_t Reg_BreakSpeedBound;           //高低速度刹车给定分界值，_IQ表示
    uint16_t Reg_HighSpdBreakRange;         //高速定点刹车刹车距离，_IQ表示
    uint16_t Reg_LowSpdBreakRange;          //低速定点刹车刹车距离，_IQ表示
    uint16_t Reg_LoadBreakAdd;              //负载定点刹车增加距离，_IQ表示
    uint16_t Reg_RC_LiftPos;                 //遥控升降给定增量
    uint16_t Reg_RC_ClawPos;                 //遥控抱夹给定增量
    uint16_t Reg_RC_BowRisePos;              //遥控俯仰给定增量
    uint16_t Reg_RC_RotatePos;               //遥控旋转给定增量
    uint16_t Reg_RC_Speed;                   //遥控速度给定量
    uint16_t Reg_GasBias;                   //油门偏置
    uint16_t Reg_CRC ;                      //
} ParamStruct_Control;

typedef enum{
    Dev_ForlLift_Control = 0,
    Dev_ForkLift_Solenoid,
    Dev_ForkLift_485Hub,
    
    Dev_Inspection_Control = 10,
    Dev_Inspection_Lift,
    
    Dev_Textile_Control = 21,
}Dev_Ver_List;


#define Tab_Ver         1
#define Tab_Len         37
#define Dev_Ver         Dev_Textile_Control


#define ParamDefault    { \
    Tab_Ver,/* Reg_Tab_Ver*/\
    Tab_Len,/* Reg_Tab_Len*/\
    Dev_Ver,/* Reg_Dev_Ver*/\
    0,/* Reg_HW_Ver*/\
    0,/* Reg_SN*/\
    0,/* Reg_SW_Ver*/\
    0,/*Reg_SW_NewApp*/\
    0,/*Reg_SW_Factory*/\
    0,/*Reg_SW_Boot*/\
    0,/* Reg_Password*/\
    0,/* Reg_WirelessBDSN*/\
    0,/* Reg_PairingFlag*/\
    1000,/* Reg_WLSpdLimit*/\
    1000,/* Reg_PCDogTimeOut*/\
    1000,/* Reg_BD1DogTimeOut*/\
    1000,/* Reg_BD2ogTimeOut*/\
    1000,/* Reg_BD3DogTimeOut*/\
    1000,/* Reg_BD4DogTimeOut*/\
    1000,/* Reg_VCUDogTimeOut*/\
    1000,/* Reg_RadarDogTimeOut*/\
    20,/* Reg_PCWalkAccTime*/\
    10,/* Reg_PCWalkDecTime*/\
    50,/* Reg_WLWalkAccTime*/\
    10,/* Reg_WLWalkDecTime*/\
    1000,/* Reg_TelescopicMBDRelayRunTime*/\
    52428,/* Reg_OverHeat*/\
    9363,/* Reg_MileFormula*/\
    6553,/* Reg_SpeedBoundary*/\
    52428,/* Reg_HighSpdGasMax*/\
    32767,/* Reg_LowSpdGasMax*/\
    1310,/* Reg_BreakSpeedBound*/\
    170,/* Reg_HighSpdBreakRange*/\
    300,/* Reg_LowSpdBreakRange*/\
    30,/* Reg_LoadBreakAdd*/\
    3276,/* Reg_RC_LiftPos*/\
    3276,/* Reg_RC_ClawPos*/\
    500,/* Reg_RC_BowRisePos*/\
    3276,/* Reg_RC_RotatePos*/\
    500,/* Reg_RC_Speed*/\
    19660,/* Reg_GasBias*/\
    0,/* Reg_CRC*/\
};

#define OS_YEAR     ((((__DATE__ [7] - '0') * 10 + (__DATE__ [8] - '0')) * 10 + (__DATE__ [9] - '0')) * 10 + (__DATE__ [10] - '0'))
#define OS_MONTH    (__DATE__ [2] == 'n' ? (__DATE__ [1] == 'a' ? 1 : 6) \
                                 : __DATE__ [2] == 'b' ? 2 \
                                 : __DATE__ [2] == 'r' ? (__DATE__ [0] == 'M' ? 3 : 4) \
                                 : __DATE__ [2] == 'y' ? 5 \
                                 : __DATE__ [2] == 'l' ? 7 \
                                 : __DATE__ [2] == 'g' ? 8 \
                                 : __DATE__ [2] == 'p' ? 9 \
                                 : __DATE__ [2] == 't' ? 10 \
                                 : __DATE__ [2] == 'v' ? 11 : 12)
#define OS_DAY      ((__DATE__ [4] == ' ' ? 0 : __DATE__ [4] - '0') * 10  + (__DATE__ [5] - '0'))
#define OS_HOUR     ((__TIME__ [0] - '0') * 10 + (__TIME__ [1] - '0'))
#define OS_MINUTE   ((__TIME__ [3] - '0') * 10 + (__TIME__ [4] - '0'))
#define OS_SECOND   ((__TIME__ [6] - '0') * 10 + (__TIME__ [7] - '0'))

#define ADDRESS_EepromParamSaving_FLAG          ADDR_FLASH_SECTOR_2    //Eeprom参数保存标志位
#define EepromParamSaving_ControlResult         0x1111
#define EepromParamSaving_SolenoidResult        0x2222
#define EepromParamSaving_SteerResult           0x3333

#define SoftVer_A                               1  //范围为0~255，A为大版本，一般为接口的变化，大版本变化后与原有框架不兼容
#define SoftVer_B                               11  //范围为0~255，B为功能版本，增加功能需变化B项
#define SoftVer_C                               1  //范围为0~255，C为bug修改，需更改C项

//-------------------- public data ------------------------------------------
extern ParamStruct_Control ParamHandle_Control;
//extern ParamStruct_Solenoid ParamHandle_Solenoid;
//extern ParamStruct_Steer ParamHandle_Steer;
//extern ParamStruct_Hub485 ParamHandle_Hub485_Fork;
//extern ParamStruct_Hub485 ParamHandle_Hub485_Body;
//-------------------- public functions -------------------------------------
void DataBaseInit(void);
void CtrlDataBaseInit ( void );
int8_t Param2Eeprom ( void );

#endif // _DataProcess__H_

//-----------------------End of file------------------------------------------
/** @}*/
