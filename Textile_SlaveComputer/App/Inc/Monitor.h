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
#ifndef _Monitor__H_
#define _Monitor__H_
//-------------------- include files ----------------------------------------
#include "Driver.h"
#include "Oled_com.h"
#include "AngleMath.h"
//#include "SteeringWheelProcess.h"
#include "NodeProcess.h"

//-------------------- public definitions -----------------------------------
#define R1                  (30000)                                             // 分压电阻R1，30kΩ
#define R2                  (2200)                                              // 分压电阻R2，2.2kΩ
#define ADC_MAX             (40950)                                              // ADC最大值，12位ADC
#define REF_VOLTAGE         (33)                                                 // 参考电压，3.3V
#define VREF_PER_ADC        _IQdiv(REF_VOLTAGE, ADC_MAX)                        // 参考电压与ADC最大值的比值
#define RESISTOR_RATIO      _IQdiv((R1 + R2), R2)                               // 分压比：R2 / (R1 + R2)
#define POWERCURR_RATIO     _IQ ( -10 )                                          //电源电流系数10A/V
#define NoFault                            0
#define Fault                              1
#define HighTempValue                      7000
//struct Error_BITS_Steer              //转向驱动板故障码
//{
//    uint32_t    uE2promErr                     : 1; //E2p故障
//    uint32_t    uDBVersion                     : 1; //数据库版本不一致
//    uint32_t    uLackUVW                       : 1; //缺相
//    uint32_t    uReangleLack                   : 1; //电机轴编码器离线
//    
//    uint32_t    uCoderTama1_Lack               : 1; //出轴1号编码器离线
//    uint32_t    uCoderTama2_Lack               : 1; //出轴1号编码器离线
//    uint32_t    uCmdDogLose                    : 1; //心跳超时
//    uint32_t    uSpdCircle                     : 1; //速度跟踪严重不良
//    
//    uint32_t    uLodeOver                      : 1; //过载
//    uint32_t    uTempOver                      : 1; //过温
//    uint32_t    uVolOver                       : 1; //过压
//    uint32_t    uBoardUpdata                   : 1; //存在板卡升级/参数修改
//    uint32_t    NO_AKM                          : 1; //不符合阿克曼模型

//    uint32_t    rsvd                           : 10;
//};
//typedef union
//{
//    uint32_t    all;
//    struct      Error_BITS_Steer   bit;
//} Fault_REG_Steer;

//struct Error_BITS_Solenoid              //液压驱动板故障码
//{
//    uint32_t    LiftSolenoidInitErr                     : 1; //电磁阀自检故障           
//    uint32_t    LiftSensorLinkLose                      : 1; //升降传感器失联          
//    uint32_t    LiftSensorDataError                     : 1; //升降拉线传感器数据异常（两个拉线数据不一致，需要注意区分升降触底检测）
//    uint32_t    LiftMotorPosLinkLose                    : 1; //电流环失效                
//    uint32_t    LiftMoveTimeOut                         : 1; //运动超时                     
//    uint32_t    LiftSolenoidRun_Err                     : 1; //电流环输出与实际运动不符       
//    
//    uint32_t    ClawSolenoidInitErr                     : 1; //电磁阀自检故障              
//    uint32_t    ClawSensorLinkLose                      : 1; //传感器失联                  
//    uint32_t    ClawSensorDataError                     : 1; //抱夹传感器数据异常（超限）  
//    uint32_t    ClawMotorPosLinkLose                    : 1; //电流环失效                
//    uint32_t    ClawMoveTimeOut                         : 1; //运动超时                       
//    uint32_t    ClawSolenoidRun_Err                     : 1; //电流环输出与实际运动不符       

//    uint32_t    BowRiseSolenoidInitErr                  : 1; //电磁阀自检故障          
//    uint32_t    BowRiseSensorLinkLose                   : 1; //传感器失联              
//    uint32_t    BowRiseSensorDataError                  : 1; //俯仰拉线传感器数据异常（超限）
//    uint32_t    BowRiseMotorPosLinkLose                 : 1; //电流环失效            
//    uint32_t    BowRiseMoveTimeOut                      : 1; //运动超时                 
//    uint32_t    BowRiseSolenoidRun_Err                  : 1; //电流环输出与实际运动不符      
//    
//    uint32_t    RotateSolenoidInitErr                   : 1; //电磁阀自检故障         
//    uint32_t    RotateSensorLinkLose                    : 1; //传感器失联  
//    uint32_t    RotateSensorDataError                   : 1; //旋转传感器数据异常（超限）    
//    uint32_t    RotateMotorPosLinkLose                  : 1; //电流环失效              
//    uint32_t    RotateMoveTimeOut                       : 1; //运动超时               
//    uint32_t    RotateSolenoidRun_Err                   : 1; //电流环输出与实际运动不符       

//    uint32_t    EepromError                             : 1; //Eeprom故障
//    uint32_t    LiftToBottomErr                         : 1; //一降到底指令错误
//    uint32_t    Claw_Rotate_Err                         : 1; //开合旋转指令冲突
//    uint32_t    Claw_Rotate_Switch_Err                  : 1; //开合旋转控制阀切换错误
//    uint32_t    SideSensorLinkLose                      : 1; //侧移传感器失联          
//    uint32_t    SideSensorErr                           : 1; //侧移传感器异常          
//    uint32_t    SideMoveTimeOut                         : 1; //侧移运动超时               
//    uint32_t    Claw_Lift_CmdErr                               : 1;  //报完纸卷禁止下降
//};

//typedef union
//{
//    uint32_t    all;
//    uint8_t     RevBuff[4];
//    struct      Error_BITS_Solenoid   bit;
//} Fault_REG_Solenoid;

struct Error_BITS_Control
{
    uint32_t    HostLinkLose            : 1; //上位机失联   
    uint32_t    VCULinkLose             : 1; //VCU心跳           
    uint32_t    RotatingLinkLose        : 1; //旋转电机失联                 
    uint32_t    TelescopicLinkLose      : 1; //伸缩电机失联             
    uint32_t    HuggingLinkLose         : 1; //环抱电机失联                   
    uint32_t    GlandLinkLose           : 1; //抬压电机失联     
    uint32_t    MOTOR1CurrOver          : 1; //MOTOR1过流             
    uint32_t    MOTOR2CurrOver          : 1; //MOTOR2过流 
    
    uint32_t    PCCurrOver              : 1; //PC过流       
    uint32_t    SensorCurrOver          : 1; //传感器过流                                        
    uint32_t    TurnError               : 1; //转向错误                 
    uint32_t    EmergencyStop           : 1; //0类急停触发           
    uint32_t    ExcessTemperature       : 1; //温度过高
    uint32_t    EepromReadErr           : 1; //Eeprom读错误
    uint32_t    EepromWriteErr          : 1; //Eeprom写入错误
    uint32_t    TelescopicMBDRelayErr   : 1; //伸出继电器故障
    
};
struct Warning_BITS
{
    uint32_t    PC_Power24V_Low             : 1; //PC24V电压低
    uint32_t    HandleBatteryLow            : 1; //手柄电池电压低
    uint32_t    BLDC1_Power24V_Low          : 1; //电机1动力24V电压低
    uint32_t    BLDC2_Power24V_Low          : 1; //电机2动力24V电压低
    uint32_t    Sensor_Power24V_Low         : 1; //传感器板电压低
    uint32_t    TurnWarning                 : 1; //旋向警告
    uint32_t    IMULinkLose                 : 1; //IMU失联
    uint32_t    RadarLinkLose                 : 1; //雷达失联
    
    uint32_t    DrawWireLinkLose            : 1;
    uint32_t    VolLow            : 1;
    uint32_t    MotorNoSt            : 1;
    uint32_t    IMUInitErr            : 1;//IMU初始化失败
    uint32_t    Res2            : 1;
    uint32_t    Res3            : 1;
    uint32_t    Res4            : 1;
    uint32_t    Res5            : 1;
    
    uint32_t    Res31           : 1; //车载80V电压低
    uint32_t    Res32       : 1; //备用电池电压低
    uint32_t    Res33              : 1; //IMU初始化失败
    uint32_t    Res34             : 1; //IMU失联
    uint32_t    Res35      : 1; //液压板动力24V电压低
    uint32_t    Res36       : 1; //转向板动力24V电压低
    uint32_t    Res37           : 1; //旋转指令错误（抱夹高度过低，禁止旋转）
    uint32_t    Res38           : 1;
    
    uint32_t    Res41           : 1; //车载80V电压低
    uint32_t    Res42           : 1; //备用电池电压低
};
//struct Solenoid_Warning_BITS
//{
//    uint32_t    SolenoidResWarning                  : 1; //电磁阀线圈阻抗异常
//    uint32_t    HighTempWarning                     : 1; //温度过高
//    uint32_t    HighVoltageWarning                  : 1; //高电压
//    uint32_t    LowVoltageWarning                   : 1; //低电压
//    uint32_t    PowerLossWarning                    : 1; //低电压
//    uint32_t    Lift2SensorLinkWarning              : 1; //升降油缸线传感器失联
//    uint32_t    Lift2SensorDataWarning              : 1; //升降油缸线传感器数值异常
//    uint32_t    RotateSNError                       : 1; //旋转指令错误（抱夹高度过低，禁止旋转）
//    uint32_t    SideShiftSNWarning                  : 1; //侧移指令错误
//};
struct Light_BITS
{
    uint8_t    HighLight                    : 1; 
    uint8_t    LowLight                     : 1; 
    uint8_t    MidLight                     : 1; 
    uint32_t   Res1                         : 5;
};

//typedef union
//{
//    uint32_t    all;
//    struct      Solenoid_Warning_BITS   bit;
//} Solenoid_Warning_REG;
typedef union
{
    uint32_t    all;
    struct      Warning_BITS   bit;
} Warning_REG;

typedef union
{
    uint32_t    all;
    struct      Error_BITS_Control   bit;
} Fault_REG;
typedef union
{
    uint8_t    all;
    struct      Light_BITS   bit;
} Light_REG;

//typedef struct
//{
 extern  Fault_REG FaultCtrl;
//    uint8_t PumpMotorFault;
////    Fault_REG_Solenoid Fault_Solenoid;
////    Fault_REG_Steer   Fault_Steer;
    
//} Fault_Handle ;//故障码汇总结构体
extern Warning_REG Warning_Report;
typedef enum{
    OLED_Start = 0,
    OLED_Logo,
    OLED_CopInfo,
    OLED_CtrlVer,
    OLED_SolenoidVer,
    OLED_SteerVer,
    OLED_BoardInfo,
    OLED_ErrCode,
    OLED_LiftSta,
    OLED_ClawSta,
    OLED_DriveSta,
    OLED_SolenoidInfo,
}OLED_Display;

typedef struct 
{
    _iq U;
    _iq I;
    _iq P;
    _iq T;
    _iq W_Step;
    _iq W_Sum;    
    _iq ADC_V;
    _iq ADC_I;
}Energy_Struct;         //功耗计算结构体

typedef enum
{
    RadarOut_Null = 0,
    RadarOut1_SlowDown,
    RadarOut2_SoftStop,
    RadarOut3_EmerStop,
} RadarOut;


#define BIT(n)         (0x01<<n)
//-------------------- public data ------------------------------------------
//extern Fault_Handle Fault_Handle1;
extern int16_t  m_nSolenoidLife;                   //推杆电机板通信心跳
extern int16_t  m_nSteeringLife;                   //转向电机板通信心跳
extern  int16_t  IMULife ;                        //IMU通信心跳
extern int16_t  m_nHostLife;
extern int16_t  HostSpdLife ;                  //上位机速度更新计数
extern uint16_t LightState;     
extern int16_t  HARTCMDLife ;//遥控器心跳
extern int16_t Temperature;        //CPU温度
extern int16_t BLDC_24V, VALVE_24V;
extern uint16_t  HostFaultClear;
//extern Solenoid_Warning_REG Solenoid_Warning;
extern uint16_t Radar_Zone_Left,Radar_Zone_Right;
extern uint8_t Stop_1A_State_Flag;               //检测急停标志
extern uint8_t   HostShutDowncmd;
extern Light_REG Light_State;                                     //补光灯控制
extern RadarOut RadarOut_R, RadarOut_L, RadarResult;
extern Energy_Struct Energy_PC, Energy_SENSOR, Energy_MOTOR1, Energy_MOTOR2, Energy_ElecMag;
//-------------------- public functions -------------------------------------
extern uint8_t ElecMagSta;
uint16_t FaultNum_Get ( void );
void Monitordisplay ( void );
void Stop_1A_State ( void );
void Buzzer_Warning ( void );
void HeartBeat_Check (void);         //心跳状态检测
void Motor_24Check(void) ;           //动力24V检测
void MonitorProcess(void);
void BumperStop(void);
void LightStateCheck(void);        //灯光状态 0是绿灯常亮，1是绿灯闪烁，2是黄灯常亮，3是黄红闪烁，4是红灯常亮
uint16_t SelectLeftRadarZone (void);
uint16_t SelectRightRadarZone (void);
void Radar_Process ( void ) ;
void RGBCtrl();
void Energy_Consumption(void);
#endif // _XXX_H_

//-----------------------End of file------------------------------------------
/** @}*/
