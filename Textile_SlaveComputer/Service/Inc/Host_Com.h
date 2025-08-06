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
#ifndef _HOST_COM__H_
#define _HOST_COM__H_
//-------------------- include files ----------------------------------------
#include"Driver.h"
#include "AngleMath.h"
#include "StateCtrl.h"
#include "ErrorCode.h"
#include "Uart_Dbug.h"
#include "Uart_Host.h"
#include "IMU_Com.h"
#include "string.h"
#include "MODBusCRC.h"
#include "EncoderProcess.h"
#include "Spd_Ctrl.h"
#include "Accelerator.h"
#include "pullrodProcess.h"
#include "Dido.h"
#include "Monitor.h"
#include "pullrod_Ctrl.h"
#include "Monitor.h"
#include "Xint.h"
#include "main.h"
#include "VCUProcess.h"
#include "Spd_Ctrl.h"
#include "SteeringWheelProcess.h"
#include "VCU_Ctrl.h"
#include "Electromagnet_Ctrl.h"
//-------------------- public definitions -----------------------------------

#define MessageID_SteerCmd          1       //转向指令
#define MessageID_GasCmd            2       //车速指令
#define MessageID_LiftCmd           3       //升降指令
#define MessageID_HuggingCmd        4       //环抱指令
#define MessageID_AutoCharCmd       5       //自动充电
#define MessageID_GlandCmd          6       //抬压指令
#define MessageID_TelescopicCmd     7       //伸缩指令
#define MessageID_RotateCmd         8       //旋转指令
#define MessageID_BrakeCmd          9       //紧急刹车（定点停车）
#define MessageID_RadarCmd          0x0A    //避障雷达开关
#define MessageID_FaultClearCmd     0x0B    //故障清除
#define MessageID_BreakCanelCmd     0x0C    //导航刹车
#define MessageID_ParamMngCmd       0x0D    //参数管理
#define MessageID_RaderLiftCmd      0x0E    //雷达升降机构
#define MessageID_SlowSpdCmd        0x0F    //车辆小位置缓行（蠕动模式）
#define MessageID_ElectronMagCmd    0x10    //电磁铁控制
#define MessageID_Reset             0x11    //故障恢复、屏蔽
#define MessageID_EncoderCalaCmd    0x12    //码盘安装后校准功能
#define MessageID_SteerCalaCmd      0x13    //转向电机码盘0点标定功能
#define MessageID_PumpEnableCmd     0x14    //泵电机控制（预留）
#define MessageID_HandAskCmd        0x15    //协议版本握手
#define MessageID_PowerSwCmd        0x16    //电源开关控制
#define MessageID_TelescopicMBDRelayCmd        0x17    //电源开关控制
#define MessageID_WatchDogCmd       0x63    //看门狗

#define MessageID_BootLinkCmd       0x30    //升级连接指令ID
#define ProtocolVer_1               2
#define ProtocolVer_2               1
#define TurnDegThreshold            2000
//#define   PCDataToAgvWorkSpd(Data)    _IQdiv(Data, _IQ(7.084))  //(2m/s-4625.5rpm))
//#define   AgvWorkSpdToPCData(Data)    _IQdiv(Data, _IQ(0.14116121))  //(2m/s-4625.5rpm))   
#define   PCDataToAgvWorkSpd(Data)    _IQdiv(Data, _IQ(9.480))//(2m/s-4625.5rpm))
#define   AgvWorkSpdToPCData(Data)    _IQrmpy(Data, _IQ(9.480)) //(2m/s-4625.5rpm))   
#define AGVReadErrMaxCnt        1000
// 预计算 1500/1024 的 Q15 值
#define CONSTANT_1500_DIV_1024 _IQ(1500.0/1024.0)  // ≈ 1.46484375
#define LiftPosBias     2600
//---------------------------------------------------------------------------------------
//-------------------- public data ------------------------------------------
/*textile*/
typedef enum {
    SlaveBoard = 1,
    RotatingBoard,    /*旋转电机*/
    TelescopicBoard, /*伸缩电机*/
    HuggingBoard,   /*环抱电机*/
    GlandBoard      /*抬压电机*/
} BoardIdDef;
typedef struct  //1
{
    uint16_t u16SN;                        //命令序号，ack回应使用
    uint16_t u16CtrlMode;                    //控制模式
    int16_t  s16Pos;                    //steer位置，码盘位置
} SteerCommand;

typedef struct //2
{
    uint16_t u16SN;                       //命令序号，ack回应使用
    int16_t  s16Spd;                   //轮子角速度，码盘速度
} GasCommand;

typedef struct //3
{
    uint16_t u16SN;                    //命令序号，ack回应使用
    uint16_t CtrlMode;                    //控制模式
    uint16_t unLiftPos;                  //目标Lift高度 32767 = 6m
    uint16_t LiftMode;                   //升降模式：0是位置伺服，1是下降到底（给定位置大于目标位置15cm），2是上升不需要精度（必须要达到对应高度，可以多不可以少）
} LiftCommand;

typedef struct//4
{
    uint16_t u16SN;                     //命令序号，ack回应使用
    uint16_t u16CtrlMode;                    //控制模式
    uint16_t u16OpenDegree;                //开度，32767=1m
} HuggingCommand;

typedef struct //5
{
    uint16_t u16SN;                        //命令序号，ack回应使用
    uint16_t u16SW;                        //开关
    uint16_t res;                      //
} AutoCharCommand;

typedef struct//6
{
    uint16_t u16SN;                    //命令序号，ack回应使用
    uint16_t u16CtrlMode;                    //控制模式
    uint16_t u16Pos;                  //目标Lift高度 32767 = 6m
    uint16_t res;                   //升降模式：0是位置伺服，1是下降到底（给定位置大于目标位置15cm），2是上升不需要精度（必须要达到对应高度，可以多不可以少）
} GlandCommand;

typedef struct//7
{
    uint16_t u16SN;                    //命令序号，ack回应使用
    uint16_t u16CtrlMode;                    //控制模式
    uint16_t u16Pos;            //开度，32767=1m
} TelescopicCommand;

typedef struct//8
{
    uint16_t u16SN;                  //命令序号，ack回应使用
    uint16_t u16CmdMode;              //见SolenoidPos_ 宏
    uint16_t u16RotateDegree;           //开度，32767=1m
} RotateCommand;

typedef struct //9
{
    uint16_t u16SN;                    //命令序号，ack回应使用
    int16_t  CarWheelPos;               //见SolenoidPos_ 宏
} WheelPosMode;

typedef struct {
    uint16_t u16SN;                        //命令序号，ack回应使用
    RunStates unDriverMode;                  //见RunStates 宏
} SetModeCommand;
typedef struct {
    uint16_t unSyncWord;    //0xfee1
    uint16_t unMsgID;
    uint16_t unTimeStamp;    //单位1ms
    uint16_t unDataLen;      //单位byte

} HostProtocalHeader;
typedef struct//0xE
{
    HostProtocalHeader ParamMngHeader;
    uint16_t u16SN;
    uint16_t u16CmdAck;
    uint16_t u16BoardIDAck;
    uint16_t u16AddrAck;
    uint16_t u16ValueAck;
    uint16_t u16Crc;
} RegDef_ParamMngCmdAck;
typedef struct {
    uint16_t unSteerAck;//转向Ack
    uint16_t unGasAck;//行进Ack
    uint16_t unCurLiftAck;//升降ack
    uint16_t unHuggingAck;//环抱ack
    uint16_t unAutoCharAck;//自动充电
    uint16_t unGlandAck;//抬压ack
    uint16_t unTelescopicAck;//伸缩ack
    uint16_t unRotatingAck;//旋转ack
    uint16_t unBrakeAck;//刹车
    uint16_t unBrakeCanelAck;//导航刹车
    uint16_t unSetModeAck;
    uint16_t unTelescopicRelayAck;
    uint16_t unPowerSwAck;
    uint16_t unRaderAck;
    uint16_t unFaultClearAck;
    uint16_t unResetAck;//复位
    uint16_t unMotorCalaAck;//
    uint16_t unElectronMagAck;//
    uint16_t unSideShiftAck;
    uint16_t unRadLiftAck;
    uint16_t unSlowSpdAck;
    uint16_t unSoundLightAck;
    uint16_t unFault_DisableAck;
    uint16_t unEncoderCalaAck;
    uint16_t unSteerCalaAck;
    uint16_t unPumpEnableAck;
    uint16_t unHandAck;
} CommandAck;                               //用于转存上位机发来的SN

typedef struct //10
{
    uint16_t u16SN;                        //命令序号，ack回应使用
    int16_t  Radar_Switch;                    //开关信号
} RadarCommand;                             //避障雷达开关信号

typedef struct//0x0B
{
    uint16_t u16SN;                        //命令序号，ack回应使用
    uint16_t u16BoardID;                   //清除信号类型 1下位机 2液压板 3转向板 4全部
    uint16_t u16Mod;
} FaultClear;                               //故障清除指令

typedef struct //0x0C
{
    uint16_t u16SN;                        //命令序号，ack回应使用
    uint16_t BreakTime;                     //刹车时间
} BreakCanel;                               //导航刹车指令

typedef struct 
{
    uint16_t u16SN;                       //命令序号，ack回应使用
    uint16_t Weight;                        //纸卷重量(KG)
    uint16_t CtrlMode;                      //控制模式
    int16_t  unSideShiftDegree;             //见SolenoidPos_ 宏
} SideShiftCommand;

typedef struct {
    uint16_t u16SN;                        //命令序号，ack回应使用
    uint16_t unCRes;                        //命令序号，ack回应使用
    uint16_t  RadLiftPos;                    //雷达升降位置
} RadLiftCmd;                               //雷达升降指令

typedef struct //0x0F
{
    uint16_t u16SN;                    //命令序号，ack回应使用
    uint16_t PosDir;                    //方向
    uint16_t CarWheelPos;                //距离
} Slow_Spd_Mode;                          //缓行模式（蠕动）

typedef struct //0x10
{
    uint16_t u16CmdSN;                        //命令序号，ack回应使用
    uint16_t u16SW;                        //开关
    uint16_t res;                      //
} ElectronMagCommand;

typedef struct {
    uint16_t u16SN;                    //命令序号，ack回应使用
    uint16_t Ctrl_ID;                    //控制对象
    uint16_t Switch;                     //开关
} Sound_Lignt_Ctrl;                      //声光控制

typedef struct {
    uint16_t u16SN;                    //命令序号，ack回应使用
    uint16_t Board_ID;                   //板卡ID
    uint32_t Fault_Num;                  //故障详细码
} Fault_Disable;                         //故障屏蔽

typedef struct {
    uint16_t u16SN;                    //命令序号，ack回应使用
    uint16_t Res;                       //预留
    uint16_t ID;                        //校准ID
} EncoderCala;                         //码盘校准

typedef struct {
    uint16_t u16SN;                    //命令序号，ack回应使用
    uint16_t Switch;                     //开关
} Steer_EncoderCala;                     //转向码盘标定

typedef struct {
    uint16_t u16SN;                    //命令序号，ack回应使用
    uint16_t ID;                        //控制对象
    uint16_t Switch;                   //开关
} PumpEnable;                         //泵电机使能控制

typedef struct {
    uint16_t u16SN;                    //命令序号，ack回应使用
    uint16_t Version_1;                 //大版本
    uint16_t Version_2;                 //小版本
} HandAsk;                              //协议握手
typedef struct//0x11
{
    uint16_t u16SN;                    //命令序号，ack回应使用
    uint16_t u16BoardID;                   //板卡ID
    uint16_t u16Delay;                   //延时复位，单位10mS
} RegDef_Reset;
extern RegDef_Reset m_Reset;

typedef struct//0x63
{
    uint16_t WDCnt;
    uint16_t res;
} RegDef_WatchDogCmd;

typedef struct//0x13
{
    uint16_t u16SN;                    //命令序号，ack回应使用
    uint16_t u16Cmd;
    uint16_t u16BoardID;                   //板卡ID
} RegDef_MotorCala;
extern RegDef_MotorCala m_MotorCala;

typedef struct//0x0D
{
    uint16_t u16SN;                    //命令序号，ack回应使用
    uint16_t u16Cmd;
    uint16_t u16BoardID;                   //板卡ID
    uint16_t u16Addr;                     //地址
    uint16_t u16Value;                   //寄存器值
} RegDef_ParamMngCmd;
extern RegDef_ParamMngCmdAck m_ParamMngCmdAck;
typedef struct//0x30
{
    uint16_t u16Cmd;
    uint16_t u16BoardID;                   //板卡ID
} RegDef_BootLinkCmd;
extern void HostCom_ParamSendReport ( RegDef_ParamMngCmdAck* pParamMngCmdAck );

struct PowerSw_BITS
{
    uint8_t    PcPowerSw                            : 1; 
    uint8_t    Motor1PowerSw                        : 1; 
    uint8_t    Motor2PowerSw                        : 1; 
    uint8_t    SensorPowerSw                        : 1; 
    uint8_t    TimerOut                             : 1; 
    uint8_t    Res1                                 : 3;
};

typedef union
{
    uint8_t     all;
    struct      PowerSw_BITS   bit;
} PowerSw_REG;

typedef struct//0x16
{
    uint16_t u16SN;                    //命令序号，ack回应使用
    PowerSw_REG PowerSwOpt;             //电源开关选择
    uint16_t u16Delay;                   //延时，单位10mS
} RegDef_PowerSwCmd;
typedef struct//0x17
{
    uint16_t u16SN;                    //命令序号，ack回应使用
    uint16_t u16Cmd;
    uint16_t u16Delay;
}HostCom_TelescopicMBDRelayCmd;
extern RegDef_PowerSwCmd m_PowerSwCmd;
//-------------------- public functions -------------------------------------
//获取俯仰指令，非0为无指令
int8_t HostCom_ReadBowRiseCommand ( void );
//获取旋转指令，非0为无指令
int8_t HostCom_ReadRotateCommand ( void );
//获取旋转指令，非0为无指令
int8_t HostCom_ReadSideShiftCommand ( void );
//获取刹车指令，非0为无指令
int8_t HostCom_ReadBrakeCommand ( int16_t* pnSolenoidPos, uint16_t* punRotateDegree );

//获取抱夹指令，非0为无指令
int8_t HostCom_ReadClampCommand ( void );
//获取升降指令，非0为无指令
int8_t HostCom_ReadLiftCommand ( void );
//获取转向指令，非0为无指令
int8_t HostCom_ReadSteerCommand ( int32_t* pnSteerPos );
//获取速度指令，非0为无指令
int8_t HostCom_ReadGasCommand ( int16_t* pnWheelSpeed );
//获取驾驶模式，非0为无指令
int8_t HostCom_ReadDrvModeCommand ( int16_t* pnDrvMode );
//初始化
int8_t HostCom_Init();
//处理接收数据
int8_t HostCom_ProcRecvData();
//发送上报数据
int8_t HostCom_SendReport();
//读取避障雷达数据
int8_t  HostCom_RadarCommand ( uint16_t* Radar_Switch ) ;

//雷达升降机构
int8_t HostCom_RadLiftCommand ( void );
int8_t HostCom_SlowSpdCommand ( uint16_t*Dir, uint16_t* Pos );      //低速缓行位置伺服
int8_t Sound_LigntCommand ( void );     //声光控制
int8_t HostCom_Fault_DisableCommand ( void );     //故障禁能（屏蔽）
int8_t EncoderCalaCommand ( void );     //码盘位置标定初始化
int8_t SteerCalaCommand ( void );     //转向码盘标定
int8_t PumpEnableCommand ( void );     //泵电机使能
int8_t HandAskCommand ( void );     //协议版本握手
int8_t HostCom_AutoCharCommand ( void );



void HostCom_Test();

extern int8_t HostCom_ResetCommand ( void );
//void Fault_SendReport() ;         //上报故障信息
void SolenoidInit ( void ) ;      //推杆指令初始化，防止刚上电，推杆运动
void Hart_DriveCmd ( void )  ; //读取遥控器行进指令
int8_t  BreakCanelFun ( uint8_t* Break_Switch, uint16_t* Break_Time );     //导航刹车
extern CommandAck   m_sCommandAck;
extern uint32_t g_unHostTimeTick;
extern uint16_t g_u16ParamStep;
extern RegDef_ParamMngCmd m_ParamMngCmd;
extern uint8_t LiftCmdFlag, ClawCmdFlag, BowRiseCmdFlag, RotateCmdFlag, SideCmdFlag;
extern GasCommand m_sCurGasCmd;
extern GlandCommand m_GlandCmd;
extern SteerCommand m_sCurSteerCmd;
extern HuggingCommand m_HuggingCmd;
extern SideShiftCommand m_sCurSideShiftCmd;
extern TelescopicCommand m_TelescopicCmd;
extern RotateCommand m_sCurRotateCmd;
extern RadarCommand  m_sRadarCmd;
extern Sound_Lignt_Ctrl m_sSound_LigntCmd;
extern uint16_t SideCtrl_Flag;
extern uint16_t RaderReport;
extern uint16_t ReSet_Flag;
extern uint8_t AutoCharFlag;
extern uint16_t PC_Rader_Stop;
extern uint16_t RotatePumpMotorSpd;
extern uint16_t Ctrl_OffLine_Flag;
extern uint16_t Steer_OffSet;
extern FaultClear     m_sFaultClear;                               //故障清除
extern uint16_t AGVReadErrCnt;
extern _iq iqVCUTurnDegFB;
extern void HostCom_ClearCommand();
extern ElectronMagCommand  m_sElectronMagCmd;                  //电磁铁控制
int8_t  HostCom_PowerSwCommand ( void );
int8_t HostCom_ReadHuggingCommand ( void );
int8_t HostCom_ReadGlandCommand ( void );
int8_t HostCom_ReadTelescopicCommand ( void );
int8_t HostCom_ReadRotatingCommand ( void );
int8_t HostCom_ReadElectronmagCommand ( void );
int8_t HostCom_ReadTelescopicMBDRelayCommand ( void );
extern void BootProcess ( void );
extern uint8_t u8BootLinkFlag;
#endif // _HOST_COM__H_

//-----------------------End of file------------------------------------------
/** @}*/
