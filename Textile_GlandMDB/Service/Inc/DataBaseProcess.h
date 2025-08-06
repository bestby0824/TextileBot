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
#include "StFlash.h"

//-------------------- public definitions -----------------------------------
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
#define EepromParamSaving_ValveResult           0x2222
#define EepromParamSaving_SteerResult           0x3333

#define SoftVer_A                               1  //范围为0~255，A为大版本，一般为接口的变化，大版本变化后与原有框架不兼容
#define SoftVer_B                               0  //范围为0~255，B为功能版本，增加功能需变化B项
#define SoftVer_C                               6  //范围为0~255，C为bug修改，需更改C项

#define HardVerCrc(x)                           x*12/4096

//-------------------- public data ------------------------------------------
typedef struct {
//-------------------- 310-313寄存器，版本相关参数 --------------------------
    uint16_t Reg_Tab_Ver;                   //0:寄存器表版本
    uint16_t Reg_Tab_Len;                   //1:寄存器表长度(参数个数)
    uint16_t Reg_Dev_Ver;                   //2:设备类型编码
    uint16_t Reg_HW_Ver;                    //3:硬件版本，形式：V1.0
    
    uint32_t Reg_SN;                        //4-5:序列号
    uint32_t Reg_SW_Ver;                    //6-7:当前软件版本
    
    uint32_t Reg_SW_NewApp;                 //8-9:NewApp软件版本
    uint32_t Reg_SW_Factory;                //10-11出厂软件版本
    
    uint32_t Reg_SW_Boot;                   //12-13:boot软件版本
    uint32_t Reg_Password;                  //14-15:写保护密码(固化到代码中)

//-------------------- 320-322寄存器，电机控制相关参数（不可修改） --------------------------
    uint16_t Reg_PolePairs;                 //16:磁极对数
    uint16_t Reg_RatedCurrent;              //17:额定电流,单位:0.01A
    uint16_t Reg_NoLoadSpeed;               //18:空载转速,单位:1rpm
    uint16_t Reg_RatedVoltage;              //19:额定电压,单位0.01V
    
    uint16_t Reg_MotorAxleOffset;           //20:电机出轴编码器OFFSET,第一次上动力电标定
    int16_t Reg_ReverseFlag;                //21:正反转标志,第一次上动力电标定,1:电角度与码盘旋转方向相同,-1:电角度与码盘旋转方向相反,0:未标定
    uint16_t Reg_ReductionRatio;            //22:电机减速比
    uint16_t u16MotorRes3;                  //23:预留
    
    int32_t Reg_SampleMaxCur;               //24-25:AD采样最大电流,根据硬件电路而定，_IQ(1)表示1A---55A
    uint16_t u16MotorRes4;                  //26:预留
    uint16_t u16MotorRes5;                  //27:预留
    
//-------------------- 330-333寄存器，电机控制相关参数（可修改） --------------------------
    uint32_t Reg_PI_POS_KP;                 //28-29:位置环KP
    uint32_t Reg_PI_POS_KI;                 //30-31:位置环KI
    
    uint32_t Reg_PI_SPD_KP;                 //32-33:速度环KP
    uint32_t Reg_PI_SPD_KI;                 //34-35:速度环KI
    
    uint16_t Reg_PI_ID_KP;                  //36:电流环(ID)KP
    uint16_t Reg_PI_ID_KI;                  //37:电流环(ID)KI
    uint16_t Reg_PI_IQ_KP;                  //38:电流环(IQ)KP
    uint16_t Reg_PI_IQ_KI;                  //39:电流环(IQ)KI
    
    uint16_t Reg_AngleMax;                  //40:位置环输入最大值
    uint16_t Reg_SpeedMax;                  //41:速度环输入最大值(位置环输出最大值),单位:RPM
    uint16_t Reg_CurrentMax;                //42:电流环输入最大值(速度环输出最大值),单位:_IQ(1):1A
    uint16_t Reg_IdIqMax;                   //43:Id/Iq输入最大值(电流环输出最大值)
    
//-------------------- 340寄存器，其他参数（可修改） --------------------------
    uint32_t Reg_FaultCheckEnable;          //44-45:故障检测使能标志,0:禁使能,1:使能
    uint16_t Reg_CtrlDeadZone;              //46:指令死区
    uint16_t Reg_PosDeadZone;               //47:到位死区
    
    uint16_t Reg_PosMax;                    //48:位置最大限幅
    uint16_t Reg_PosMin;                    //49:位置最小限幅
    uint16_t Reg_TorqueMax;                 //50:转矩最大限幅
    uint16_t Reg_TorqueMin;                 //51:转矩最小限幅
    
    uint16_t Reg_CaliTimeOut;               //52:标定超时时间
    uint16_t Reg_RunTimeOut;                //53:运动超时时间
    uint16_t Reg_RC_Spdmax;                 //54:遥控模式最大速度
    uint16_t Reg_PromEnd;                  //55:参数结束
    
//-------------------- 350-253寄存器，其他相关参数 -------------------------
    uint16_t Reg_Spd_Slow;                  //低速配置，65535代表8.37m/S，低速用于找0点等，有强烈碰撞风险（非过坎），故限制最大1m/S
    uint16_t Reg_Spd_Normal;                //中速配置，65535代表8.37m/S
    uint16_t Reg_Spd_Fast;                  //高速配置，65535代表8.37m/S
    uint16_t Reg_Pos_Max;                   //终点位置，1代表0.1m
    
    uint16_t Reg_DogTimeOut;                 //上位机心跳超时
    uint16_t Reg_IniSta;                     //初始化状态字
    uint16_t Reg_SocLim;                     //充电阈值
    uint16_t ChargDistance;                  //单位0.01m
    
    uint16_t CardDistance;                   //单位0.01m
    uint16_t EndDistance;                    //单位0.01m
    uint32_t Reg_SW_Ver_usr;                 //预留10
    
    int32_t Reg_Tag0;                       //标志牌位置
    int32_t Reg_Tag1;                       //标志牌位置
    
    int32_t Reg_Tag2;                       //标志牌位置
    int32_t Reg_Tag3;                       //标志牌位置
    
    int32_t Reg_Tag4;                       //标志牌位置
    int32_t Reg_Tag5;                       //标志牌位置
    
    int32_t Reg_Tag6;                       //标志牌位置
    int32_t Reg_Tag7;                       //标志牌位置
    
    int32_t Reg_Tag8;                       //标志牌位置
    int32_t Reg_Tag9;                       //标志牌位置
    
    int32_t Reg_Tag10;                       //标志牌位置
    int32_t Reg_Tag11;                       //标志牌位置
    
    int32_t Reg_Tag12;                       //标志牌位置
    int32_t Reg_Tag13;                       //标志牌位置
    
    int32_t Reg_Tag14;                       //标志牌位置
    int32_t Reg_Tag15;                       //标志牌位置
    
    int32_t Reg_Tag16;                       //标志牌位置
    int32_t Reg_Tag17;                       //标志牌位置
    
    int32_t Reg_Tag18;                       //标志牌位置
    int32_t Reg_Tag19;                       //标志牌位置
    
    int32_t Reg_Tag20;                       //标志牌位置
    int32_t Reg_Tag21;                       //标志牌位置
    
    int32_t Reg_Tag22;                       //标志牌位置
    int32_t Reg_Tag23;                       //标志牌位置
    
    int32_t Reg_Tag24;                       //标志牌位置
    int32_t Reg_Tag25;                       //标志牌位置
    
    int32_t Reg_Tag26;                       //标志牌位置
    int32_t Reg_Tag27;                       //标志牌位置
    
    int32_t Reg_Tag28;                       //标志牌位置
    int32_t Reg_Tag29;                       //标志牌位置
    
    int32_t Reg_Tag30;                       //标志牌位置
    int32_t Reg_Tag31;                       //74-75:标志牌位置
    
    uint16_t Reg_AccSpd;                     //76：加速加速度，65535代表1m/S^2
    uint16_t Reg_DecSpd;                     //77：减速加速度，65535代表1m/S^2
    uint16_t Reg_OverLoadLim;                //78：过载时间，单位mS，2号车30000，其他车10000
//    uint16_t Reg_Res15;                     //预留15z
    
    uint16_t Reg_CRC ;                      //79:CRC校验
} ParamStruct_Steer;
extern ParamStruct_Steer ParamHandle_Steer;

typedef enum{
    Dev_Textile_Control = 21,
    Dev_Textile_Rotate,
    Dev_Textile_Telescopic,
    Dev_Textile_Hug,
    Dev_Textile_Gland,
}Dev_Ver_List;

#define Tab_Ver         1
#define Tab_Len         sizeof( ParamStruct_Steer )
#define Dev_Ver         Dev_Textile_Gland

#define ParamDefault    { \
    Tab_Ver,/* Reg_Tab_Ver*/\
    Tab_Len,/* Reg_Tab_Len*/\
    Dev_Ver,/* Reg_Dev_Ver*/\
    0200,/* Reg_HW_Ver*/\
    0xFFFFFFFF,/* Reg_SN*/\
    001000001,/* Reg_SW_Ver*/\
    0xFFFFFFFF,/*Reg_SW_NewApp*/\
    001000001,/*Reg_SW_Factory*/\
    001000001,/*Reg_SW_Boot*/\
    202504,/* Reg_Password*/\
    \
    4,/* Reg_PolePairs*/\
    200,/* Reg_RatedCurrent*/\
    3000,/* Reg_NoLoadSpeed*/\
    2400,/* Reg_RatedVoltage*/\
    0,/* Reg_MotorAxleOffset*/\
    0,/* Reg_ReverseFlag*/\
    0,/* Reg_ReductionRatio*/\
    0,/* u16MotorRes3*/\
    _IQ(27),/* Reg_SampleMaxCur*/\
    0,/* u16MotorRes4*/\
    0,/* u16MotorRes5*/\
    \
    _IQ ( 24 ),/* Reg_PI_POS_KP*/\
    _IQ ( 0 ),/* Reg_PI_POS_KI*/\
    _IQ ( 20 ),/* Reg_PI_SPD_KP*/\
    _IQ ( 0.002 ),/* Reg_PI_SPD_KI*/\
    _IQ ( 0.085 ),/* Reg_PI_ID_KP*/\
    _IQ ( 0.075 ),/* Reg_PI_ID_KI*/\
    _IQ ( 0.085 ),/* Reg_PI_IQ_KP*/\
    _IQ ( 0.075 ),/* Reg_PI_IQ_KI*/\
    _IQ ( 0.1667 ),/* Reg_AngleMax*/\
    3000,/* Reg_SpeedMax*/\
    300,/* Reg_CurrentMax*/\
    _IQ ( 0.5 ),/* Reg_IdIqMax*/\
    \
    0xFFFFFFFF,/* Reg_FaultCheckEnable*/\
    1,/* Reg_CtrlDeadZone*/\
    1,/* Reg_PosDeadZone*/\
    \
    320,/* Reg_PosMax*/\
    8,/* Reg_PosMin*/\
    50,/* Reg_TorqueMax*/\
    30,/* Reg_TorqueMin*/\
    20000,/* Reg_CaliTimeOut*/\
    20000,/* Reg_RunTimeOut*/\
    _IQ(0.25),/* Reg_RC_Spdmax*/\
    0,/* 参数结束*/\
    \
    3900,/* Reg_Spd_Slow*/\
    15600,/* Reg_Spd_Normal*/\
    62600,/* Reg_Spd_Fast*/\
    1000,/* Reg_Pos_Max*/\
    6000,/* Reg_DogTimeOut*/\
    0,/* Reg_IniSta*/\
    50,/* Reg_SocLim*/\
    100,/* ChargDistance*/\
    4800,/* CardDistance*/\
    100,/* EndDistance*/\
    0,/* SWVer*/\
    _IQ(0),/*Reg_Tag0*/\
    _IQ(48),/*Reg_Tag1*/\
    _IQ(96),/*Reg_Tag2*/\
    _IQ(144),/*Reg_Tag3*/\
    _IQ(192),/*Reg_Tag4*/\
    _IQ(240),/*Reg_Tag5*/\
    _IQ(288),/*Reg_Tag6*/\
    _IQ(336),/*Reg_Tag7*/\
    _IQ(384),/*Reg_Tag8*/\
    _IQ(432),/*Reg_Tag9*/\
    _IQ(480),/*Reg_Tag10*/\
    _IQ(528),/*Reg_Tag11*/\
    _IQ(576),/*Reg_Tag12*/\
    _IQ(624),/*Reg_Tag13*/\
    _IQ(672),/*Reg_Tag14*/\
    _IQ(720),/*Reg_Tag15*/\
    _IQ(768),/*Reg_Tag16*/\
    _IQ(816),/*Reg_Tag17*/\
    _IQ(864),/*Reg_Tag18*/\
    _IQ(912),/*Reg_Tag19*/\
    _IQ(960),/*Reg_Tag20*/\
    _IQ(1008),/*Reg_Tag21*/\
    _IQ(1056),/*Reg_Tag22*/\
    _IQ(1104),/*Reg_Tag23*/\
    _IQ(1152),/*Reg_Tag24*/\
    _IQ(1200),/*Reg_Tag25*/\
    _IQ(1248),/*Reg_Tag26*/\
    _IQ(1296),/*Reg_Tag27*/\
    _IQ(1344),/*Reg_Tag28*/\
    _IQ(1392),/*Reg_Tag29*/\
    _IQ(1440),/*Reg_Tag30*/\
    _IQ(1488),/*Reg_Tag31*/\
    _IQ(0.01),/* Reg_AccSpd*/\
    _IQ(0.01),/* Reg_DecSpd*/\
    5000,/* Reg_OverLoadLim*/\
    0,/* Reg_CRC*/\
};

//-------------------- public functions -------------------------------------
void DataBaseInit(void);
int8_t Param2Eeprom ( ParamStruct_Steer Param );
int DataUpParameter ( void );

#endif // _DataProcess__H_

//-----------------------End of file------------------------------------------
/** @}*/
