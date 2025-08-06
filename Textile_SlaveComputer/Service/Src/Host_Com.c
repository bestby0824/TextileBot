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

//-------------------- include files ----------------------------------------
#include "Host_Com.h"


//-------------------- local definitions ------------------------------------
//0.2m俯仰处于中间位置


#define MessageID_Polling               99
#define MessageID_StatusReport          101
#define MessageID_ErrorReport           102
#define MessageID_IMUData               104

#define ProtocalHeader_Length 8
#define ProtocalTail_Length 2

uint8_t LiftCmdFlag, ClawCmdFlag, BowRiseCmdFlag, RotateCmdFlag, SideCmdFlag;
uint16_t Ctrl_OffLine_Flag = 1;
uint16_t RaderReport = 4;
#define ReportBuffer_Length 255
#define MaxRecvDataLength   255

uint32_t hostCnt1 = 0, hostCnt2 = 0, hostCnt3 = 0;
uint16_t PC_Rader_Stop = 0;
uint32_t HostTextCnt = 0, TextCnt = 0 ;
uint16_t  Sidecnt = 0;
uint16_t Steer_OffSet = 0;    //546
uint16_t AGVReadErrCnt = 0;

typedef struct {
    HostProtocalHeader sHeader;             // 8字节
    uint16_t  Ver_1;                         //协议大版本号
    uint16_t  Ver_2;                         //协议小版本号
    uint32_t SlaveFaultNum;                  //下位机故障码
    uint32_t SlaveWarning;                    //下位机警告
    uint32_t RotatingFaultNum;                  //旋转故障码 16字节
    uint32_t RotatingWarning;                    //旋转警告
    uint32_t TelescopicFaultNum;                  //伸缩故障码
    uint32_t TelescopicWarning;                    //伸缩警告
    uint32_t HuggingFaultNum;                  //环抱故障码 16字节
    uint32_t HuggingWarning;                    //环抱警告
    uint32_t GlandFaultNum;                  //压盖故障码 16字节
    uint32_t GlandWarning;                    //压盖警告
    uint8_t AGVErr1;                   //AGV
    uint8_t AGVErr2;                  //AGV
    uint16_t AGVLiftPosFB;                  //升降高度
//    uint8_t AGVPropDscRunFB;
    int16_t AGVWorkSpdFB;                   //AGV行走速度
    uint16_t RotatingPosFB;                //旋转0-90度
    uint16_t TelescopicPosFB;                //伸缩
    uint16_t HuggingPosFB;                //环抱
    uint16_t GlandPosFB;                //盖板
    uint8_t BMSFlag;                    //电池状态标志位
    uint8_t BMSSoc;                     //电池SOC
    uint16_t BMSCycle;                //电池完整循环次数
    uint16_t BMSCurr;                //电池电流
    int8_t BMSMaxTmp;                //电池最高温度
    uint8_t BMSErr;                //电池故障码
    int16_t VCUTurnDegFB;       //VCU转向角度
    int16_t nImuRZ;
    int16_t nImuRY;
    int16_t nImuRX;
    int16_t nImuRZSpd;
    int16_t nImuRYSpd;
    int16_t nImuRXSpd;
    int16_t nImuRZAcc;
    int16_t nImuRYAcc;
    int16_t nImuRXAcc;
    uint8_t u8EMSta;          //电磁铁状态
    uint8_t u8PowerSwSta;  //电源状态
    uint8_t u8AutoChargeSta;  //自动充电状态
    uint8_t u8CurMode;         //当前模式
    uint16_t u16PowerSwAck;  //电源状态ack
    uint16_t AGVLiftAck;         //升降ack
    uint16_t AGVWalkAck;         //行进ack
    uint16_t RotatingAck;        //旋转ack
    uint16_t TelescopicAck;      //伸缩ack
    uint16_t HuggingAck;                //环抱ack
    uint16_t GlandAck;                //抬压ack
    uint16_t VCUTurnAck;                //转向ack
    uint16_t FixedPointParkingAck;        //定点停车ack
    uint16_t ParkingNavigationAck;       //导航停车ack
    uint16_t RadarAck;                   //避障ack
    uint16_t FaultClearAck;              //故障清除ack
    uint16_t ResetAck;                   //复位ack
    uint16_t TelescopicMBDRelayAck;      //伸缩继电器ack
    uint16_t u16AutoChargeAck;            //自动充电ack
    uint16_t u16ElectronMagAck;           //电磁铁开关ack
    uint16_t GlandSta;                //抬压电机状态
    uint16_t HuggingSta;                //环抱电机状态
    uint16_t TelescopicSta;              //伸缩电机状态
    uint16_t RotatingSta;                //旋转电机状态
    uint16_t RadarSta;                //避障雷达状态反馈
    uint16_t unBrakeSta;                //定点刹车状态
    uint16_t TelescopicMBDRelaySta;           //伸缩继电器状态
    uint16_t unSlowSpdSta;              //蠕动状态
    uint16_t AGVLiftPosSta;             //升降状态
    int16_t s16BrakePosFB;              //定点停车位置反馈
    uint16_t unSlowSpdPosFB;           //蠕动位置
    uint16_t unSlowSpdAck;            //蠕动ack
    uint16_t u16AgvWalkDeg;             //行进码盘反馈
    uint16_t u16RotatingEnc;            //旋转码盘反馈
    uint16_t TelescopicMBDNPNSta;       //接触开关状态
    uint16_t res2;
    uint16_t res3;
    uint16_t res4;
    uint16_t res5;
    uint16_t res6;
    uint16_t res7;
    uint16_t res8;
    uint16_t res9;
    uint16_t res10;
//    uint16_t res11;
//    uint16_t res12;
//    uint16_t res13;
//    uint16_t res14;
//    uint16_t res15;
//    uint16_t res16;
//    uint16_t res17;
//    uint16_t res18;
//    uint16_t res19;
//    uint16_t res20;
//    uint16_t res21;
//    uint16_t res22;
//    uint16_t res23;
//    uint16_t res24;
//    uint16_t res25;
//    uint16_t res26;
//    uint16_t res27;
//    uint16_t res28;
    uint16_t u16Crc;
} StatusReport;

typedef struct
{
    HostProtocalHeader sHeader;

    uint8_t SolenoidLift_Mode,
            SolenoidClaw_Mode,
            SolenoidBowRise_Mode,
            SolenoidRotate_Mode;

    uint16_t SolenoidLift_TarPos,
             SolenoidClaw_TarPos,
             SolenoidBowRise_TarPos,
             SolenoidRotate_TarPos;

    uint8_t Lift_Mode,
            Claw_Mode,
            BowRise_Mode,
            Rotate_Mode;
    uint16_t Lift_TarTarPos,
             Claw_TarPos,
             BowRise_TarPos,
             Rotate_TarPos;
} LogReport;



SolenoidCommand SolenoidCtrlCmd;

extern uint8_t  Status_Text;
extern int16_t PosCtrl_InitFlag;    //位置控制初始化

int16_t  m_nHostLife = -1;                  //上位机心跳计数
int16_t  HostSpdLife = -1;                  //上位机速度更新计数
int32_t Com_success_Cnt = 0;               //用于计数与上位机通信成功的数量
int32_t Com_success_Cnt_1 = 0;               //用于计数与上位机通信成功的数量
uint8_t AutoCharFlag  = 0;                   //自动充电标志
uint16_t SideCtrl_Flag = 0;
uint8_t u8BootLinkFlag = 0;
//-------------------- private data -----------------------------------------

//-------------------- private functions declare ----------------------------

void HostCom_ProcCommand ( uint8_t* pbData, uint16_t unDataLen );
void HostCom_ResolveCommand ( uint8_t* pbData, uint16_t unDataLen );
int8_t HostCom_CheckCrc ( uint8_t* pbData, uint16_t unPackLen, uint16_t unDatakLen );

//-------------------- public data ------------------------------------------
//SN若为0则代表无新指令

#define HostCom_FullLife  500                               //上位机发送心跳
SteerCommand m_sCurSteerCmd;                                //转向指令
GasCommand m_sCurGasCmd;                                    //VGA指令
LiftCommand m_sCurLiftCmd;                                  //升降指令
GlandCommand m_GlandCmd;                                  //抬压指令
HuggingCommand m_HuggingCmd;                                //开合指令
AutoCharCommand m_sAutoCharCmd;                             //自动充电
SetModeCommand m_sCurModeCmd;                               //切换驾驶模式（挂挡）
TelescopicCommand m_TelescopicCmd;                          //伸缩指令
RotateCommand m_sCurRotateCmd;                              //旋转指令
WheelPosMode m_sCurBrakeCmd;                                //刹车指令
RadarCommand   m_sRadarCmd;                                 //避障雷达开关
FaultClear     m_sFaultClear;                               //故障清除
BreakCanel     m_sBreakCanel;                               //导航刹车
SideShiftCommand m_sCurSideShiftCmd;                        //侧移控制
RadLiftCmd     m_sRadLift;                                  //雷达升降机构
Slow_Spd_Mode m_sSlow_SpdCmd;                               //车辆小位置缓行（蠕动模式）
ElectronMagCommand  m_sElectronMagCmd;                  //电磁铁控制
Sound_Lignt_Ctrl m_sSound_LigntCmd;                         //声光控制
Fault_Disable  m_sFault_DisableCmd;                         //故障恢复、屏蔽
EncoderCala  m_sEncoderCalaCmd;                             //码盘安装后校准功能
Steer_EncoderCala  m_sSteerCalaCmd;                         //转向电机码盘0点标定功能
PumpEnable    m_sPumpEnableCmd;                             //泵电机控制（预留）
HandAsk        m_sHandAskCmd;                               //协议版本握手
RegDef_PowerSwCmd m_PowerSwCmd = {0,15,0};
CommandAck   m_sCommandAck;

/*Textile*/
uint16_t g_u16ParamStep = 0;
RegDef_WatchDogCmd m_WatchDogCmd;
RegDef_Reset m_Reset;
RegDef_MotorCala m_MotorCala;
RegDef_ParamMngCmd m_ParamMngCmd;
RegDef_BootLinkCmd m_BootLinkCmd;
RegDef_ParamMngCmdAck m_ParamMngCmdAck;
HostCom_TelescopicMBDRelayCmd m_TelescopicMBDRelayCmd;
uint32_t g_u32ParamMngTimeTick = 0;

uint16_t m_unRecvLen = 0;
uint8_t m_bReportBuf[ReportBuffer_Length] = {0};
uint8_t m_bRecvBuf[MaxRecvDataLength] = {0};
//StatusReport* m_psStatusRepot = ( StatusReport* ) m_bReportBuf;
StatusReport* psStatus = ( StatusReport* ) m_bReportBuf;
uint32_t g_unHostTimeTick = 0;
//-------------------- public functions -------------------------------------
uint8_t m_bInitFlag = 0;
uint32_t unStartMSec = 0;
uint32_t unCurMSec = 0;
uint16_t ReSet_Flag = 0;
_iq LastSpeed = 0;
extern uint16_t Break_RangeFlag;
extern uint8_t LiftToBottom;
/*
* @brief       函数
* @param       无
* @retval      无
*
*/
void HostCom_Test()
{
    uint32_t unDurationMSec = 0;
    unCurMSec = 0;
    if ( RunSta_Reset2Idle == GetRUNSta() )
        return;
    if ( 0 == m_bInitFlag )
    {
        m_bInitFlag = 1;
        unStartMSec = g_unHostTimeTick;
    }
    unCurMSec = unStartMSec;
    unDurationMSec = 1000 / 20;
    if ( g_unHostTimeTick > unCurMSec && g_unHostTimeTick < ( unCurMSec + unDurationMSec ) ) //前进挡3s
    {
        m_sCurModeCmd.u16SN = 1;
        m_sCurModeCmd.unDriverMode = RunSta_Run_D;
    }
    unCurMSec += unDurationMSec;
}
/*
* @brief       液压控制初始化函数---没用
* @param       无
* @retval      无
*
*/
void SolenoidInit ( void )
{
    SolenoidCtrlCmd.Lift.EN = 0;
    SolenoidCtrlCmd.Claw.EN = 0;
    SolenoidCtrlCmd.Rotate.EN = 0;
    SolenoidCtrlCmd.BowRise.EN = 0;
    SolenoidCtrlCmd.SideShift.EN = 0;
//    m_sCurLiftCmd.unLiftPos = g_sSolenoid_Info1[RodID_Lift].Pos;
//    m_sCurClampCmd.unOpenDegree = g_sSolenoid_Info1[RodID_Claw].Pos;
//    m_sCurBowRiseCmd.unBowRiseDegree = g_sSolenoid_Info1[RodID_BowRise].Pos;
//    m_sCurRotateCmd.unRotateDegree = g_sSolenoid_Info1[RodID_Rotate].Pos;
//    m_sCurSideShiftCmd.unSideShiftDegree = g_sSolenoid_Info1[RodID_SideShift].Pos;
    LiftCmdFlag = 0;
    ClawCmdFlag = 0;
    BowRiseCmdFlag = 0;
    RotateCmdFlag = 0;
    SideCmdFlag = 0;
}
/*
* @brief       初始化函数---没用
* @param       无
* @retval      无
*
*/
int8_t HostCom_Init()
{
    m_nHostLife = -1;
    HostSpdLife = -1;
    HostCom_ClearCommand();
    g_unHostTimeTick = 0;
    return Err_None;
}
/*
* @brief       转向接收上位机命令函数--0x01
* @param       无
* @retval      无
*
*/
int8_t HostCom_ReadSteerCommand ( int32_t* pnSteerPos )
{
    _iq IQTurnPos;

    if ( 0 == m_sCurSteerCmd.u16SN )
        return Err_NotData;
    IQTurnPos = _IQsat ( m_sCurSteerCmd.s16Pos,  _IQ ( 0.2639 ), - _IQ ( 0.2639 ) );
    *pnSteerPos = Reduction_Ratio ( IQTurnPos ); //
    if ( ( m_sCommandAck.unSteerAck != m_sCurSteerCmd.u16SN ) && ( m_eCtrlMode == PC_MODE ) ) //有新指令，开泵
    {
//        RegReal_ChassisParam.u8ChassisEnabled.bit.SteeringEnabled = 1;//VCU转向使能
        StreeCnt = 0;
    }
    m_sCommandAck.unSteerAck = m_sCurSteerCmd.u16SN;//反馈SN
    m_sCurSteerCmd.u16SN = 0;//清除标记位
    Com_success_Cnt++;
    return Err_None;
}

/*
* @brief       获取上位机行进车速指令函数1kHz--0x02
* @param       无
* @retval      无
*
*/
int8_t HostCom_ReadGasCommand ( int16_t* pnWheelSpeed )
{
    if ( 0 == m_sCurGasCmd.u16SN )
    {
        AGVReadErrCnt++;
        return Err_NotData;
    }
    *pnWheelSpeed =  PCDataToAgvWorkSpd ( m_sCurGasCmd.s16Spd );  //m_sCurGasCmd.nWheelSpeed 32768表示2m/s
    LastSpeed = *pnWheelSpeed;
    m_sCommandAck.unGasAck = m_sCurGasCmd.u16SN;//反馈SN
    m_sCurGasCmd.u16SN = 0;//清除标记位
    Break_RangeFlag = 0;
    AGVReadErrCnt = 0;
    HostSpdLife = 2000;         //两秒没有收到上位机速度指令，需要对速度清零
    Com_success_Cnt++;
    return Err_None;
}
/*
* @brief       升降电机控制数据传送函数---0x03
* @param       无
* @retval      无
*
*/
int8_t HostCom_ReadLiftCommand ( void )
{
    uint16_t u16LiftPos;
//    u16LiftPos = m_sCurLiftCmd.unLiftPos;
//    SolenoidCtrlCmd.Lift.unPos = _IQdiv(u16LiftPos, CONSTANT_1500_DIV_1024);	
    if ( 0 == m_sCurLiftCmd.u16SN )
        return Err_NotData;
    m_sCommandAck.unCurLiftAck = m_sCurLiftCmd.u16SN;
    SolenoidCtrlCmd.Lift.EN = 1;//m_sCurLiftCmd.CtrlMode;
    u16LiftPos = m_sCurLiftCmd.unLiftPos;
    SolenoidCtrlCmd.Lift.CtrlMode = m_sCurLiftCmd.LiftMode;
    if ( u16LiftPos > LiftPosMax )
    {
        u16LiftPos = LiftPosMax;
    }
    SolenoidCtrlCmd.Lift.unPos = _IQdiv(u16LiftPos, CONSTANT_1500_DIV_1024);
    SolenoidCtrlCmd.Lift.unSN = m_sCurLiftCmd.u16SN;
    m_sCurLiftCmd.u16SN = 0;//清除标记位
    Com_success_Cnt++;
    return Err_None;
}
/*
* @brief       获取环抱指令函数---0x04
* @param       无
* @retval      无
*
*/
int8_t HostCom_ReadHuggingCommand ( void )
{
    if ( 0 == m_HuggingCmd.u16SN )
        return Err_NotData;
    m_sCommandAck.unHuggingAck = m_HuggingCmd.u16SN;//反馈SN
    RegReal_HuggingMBDCmd.u16Cmd = m_HuggingCmd.u16CtrlMode;
    RegReal_HuggingMBDCmd.u16SN = m_HuggingCmd.u16SN;
    RegReal_HuggingMBDCmd.u16Ref = m_HuggingCmd.u16OpenDegree;
    RegReal_HuggingMBDCmd.u16EN = 1;
    m_HuggingCmd.u16SN = 0;//清除标记位
    Break_RangeFlag = 0;
    HostSpdLife = 2000;         //两秒没有收到上位机速度指令，需要对速度清零
    Com_success_Cnt++;
    return Err_None;
}
/*
* @brief       自动充电ack函数---0x05
* @param       无
* @retval      无
*
*/

int8_t HostCom_AutoCharCommand ( void )     //自动充电
{
    if ( 0 == m_sAutoCharCmd.u16SN )
        return Err_NotData;
    m_sCommandAck.unAutoCharAck = m_sAutoCharCmd.u16SN;
    AutoCharFlag = m_sAutoCharCmd.u16SW;
    m_sAutoCharCmd.u16SN = 0;
    return Err_None;
}

/*
* @brief       获取抬压指令函数---0x06
* @param       无
* @retval      无
*
*/
int8_t HostCom_ReadGlandCommand ( void )
{
    if ( 0 == m_GlandCmd.u16SN )
        return Err_NotData;
    m_sCommandAck.unGlandAck = m_GlandCmd.u16SN;//反馈SN
    RegReal_GlandMBDCmd.u16Cmd = m_GlandCmd.u16CtrlMode;
    RegReal_GlandMBDCmd.u16SN = m_GlandCmd.u16SN;
    RegReal_GlandMBDCmd.u16Ref = m_GlandCmd.u16Pos;
    RegReal_GlandMBDCmd.u16EN = 1;
    m_GlandCmd.u16SN = 0;//清除标记位
    Break_RangeFlag = 0;
    HostSpdLife = 2000;         //
    Com_success_Cnt++;
    return Err_None;
}

/*
* @brief       获取伸缩指令函数---0x07
* @param       无
* @retval      无
*
*/
int8_t HostCom_ReadTelescopicCommand ( void )
{
    if ( 0 == m_TelescopicCmd.u16SN )
        return Err_NotData;
    m_sCommandAck.unTelescopicAck = m_TelescopicCmd.u16SN;//反馈SN
    RegReal_TelescopicMBDCmd.u16Cmd = m_TelescopicCmd.u16CtrlMode;
    RegReal_TelescopicMBDCmd.u16SN = m_TelescopicCmd.u16SN;
    RegReal_TelescopicMBDRelayCmd.u16SN = m_TelescopicCmd.u16SN;
    RegReal_TelescopicMBDCmd.u16Ref = m_TelescopicCmd.u16Pos;
    RegReal_TelescopicMBDCmd.u16EN = 1;
    m_TelescopicCmd.u16SN = 0;//清除标记位
    Break_RangeFlag = 0;
    HostSpdLife = 2000;         //两秒没有收到上位机速度指令，需要对速度清零
    Com_success_Cnt++;
    return Err_None;
}
/*
* @brief       获取旋转指令函数---0x08
* @param       无
* @retval      无
*
*/
int8_t HostCom_ReadRotatingCommand ( void )
{
    if ( 0 == m_sCurRotateCmd.u16SN )
        return Err_NotData;
    m_sCommandAck.unRotatingAck = m_sCurRotateCmd.u16SN;//反馈SN
    RegReal_RotatingMBDCmd.u16Cmd = m_sCurRotateCmd.u16CmdMode;
    RegReal_RotatingMBDCmd.u16SN = m_sCurRotateCmd.u16SN;
    RegReal_RotatingMBDCmd.u16Ref = m_sCurRotateCmd.u16RotateDegree;
    RegReal_RotatingMBDCmd.u16EN = 1;
    m_sCurRotateCmd.u16SN = 0;//清除标记位
    Break_RangeFlag = 0;
    HostSpdLife = 2000;         //两秒没有收到上位机速度指令，需要对速度清零
    Com_success_Cnt++;
    return Err_None;
}

/*
* @brief       刹车控制模式获取上位机更新位置信息函数--0x09
* @param       无
* @retval      1kHZ
*
*/
//int8_t HostCom_ReadBrakeCmd ( int16_t* PostoStopPos)
//{
//    if ( 0 == m_sCurBrakeCmd.u16SN )
//        return Err_NotData;
//    *PostoStopPos = m_sCurBrakeCmd.CarWheelPos;      //最后运动距离
//    return Err_None;
//}
int8_t HostCom_ReadBrakeCommand ( int16_t* PostoStopPos, uint16_t* PostoStopFlag )
{
    if ( 0 == m_sCurBrakeCmd.u16SN )
        return Err_NotData;
    m_sCommandAck.unBrakeAck = m_sCurBrakeCmd.u16SN;//反馈SN
    *PostoStopPos = m_sCurBrakeCmd.CarWheelPos;      //最后运动距离
    *PostoStopFlag = 1;       //位置模式Flag

    SolenoidCtrlCmd.CmdSet.bit.Brake = 1;
    m_sCurBrakeCmd.u16SN = 0;//清除标记位
    iq_PosAddCircles = 0;//清除当前实际位置
    Com_success_Cnt++;
    return Err_None;
}
/*
* @brief       避障雷达开关接收函数---0x0A
* @param       无
* @retval      无
*
*/
int8_t  HostCom_RadarCommand ( uint16_t* Radar_Switch )      //避障雷达开关接收
{
    if ( 0 == m_sRadarCmd.u16SN )
        return Err_NotData;
    *Radar_Switch =  m_sRadarCmd.Radar_Switch;
    m_sCommandAck.unRaderAck = m_sRadarCmd.u16SN;
    m_sRadarCmd.u16SN = 0;
    return Err_None;
}
/*
* @brief       故障禁能（屏蔽）ack函数---0x0B
* @param       无
* @retval      无
*
*/
int8_t HostCom_Fault_DisableCommand ( void )     //故障禁能（屏蔽）
{
    if ( 0 == m_sFaultClear.u16SN )
        return Err_NotData;
    HostFaultClear = 1;
    TextCnt++;//无用
    m_sCommandAck.unFaultClearAck = m_sFaultClear.u16SN;
    m_sFaultClear.u16SN = 0;
    return Err_None;
}
/*
* @brief       导航刹车信息获取函数---0x0C
* @param       无
* @retval      无
*
*/
int8_t  BreakCanelFun ( uint8_t* Break_Switch, uint16_t* Break_Time )     //导航刹车
{
    if ( 0 == m_sBreakCanel.u16SN )
        return Err_NotData;
    //
    m_sCommandAck.unBrakeCanelAck = m_sBreakCanel.u16SN;
    *Break_Switch = m_sBreakCanel.BreakTime;

    m_sBreakCanel.u16SN = 0;
    return Err_None;
}
/*
* @brief       车辆小位置缓行（蠕动模式）获取上位机控制参数函数--0x0F
* @param       无
* @retval      无--1KHz
*
*/
int8_t HostCom_SlowSpdCommand ( uint16_t*Dir, uint16_t* Pos )      //低速缓行位置伺服
{
    if ( 0 == m_sSlow_SpdCmd.u16SN )
        return Err_NotData;

    *Dir =  m_sSlow_SpdCmd.PosDir;
    * Pos = m_sSlow_SpdCmd.CarWheelPos;
    iq_PosAddCircles = 0;//清除当前实际位置

    SlowModeFlag = 1;
    SlowCtrlOverTimecnt = 0;
    m_sCommandAck.unSlowSpdAck = m_sSlow_SpdCmd.u16SN;//反馈SN
    m_sSlow_SpdCmd.u16SN = 0;

    return Err_None;
}

/*
* @brief       获取电磁铁控制指令函数---0x10
* @param       无
* @retval      无
*
*/
int8_t HostCom_ReadElectronmagCommand ( void )
{
    if ( 0 == m_sElectronMagCmd.u16CmdSN )
        return Err_NotData;
    m_sCommandAck.unElectronMagAck =  m_sElectronMagCmd.u16CmdSN;
    EMCtrlON ( m_sElectronMagCmd.u16SW );
    m_sElectronMagCmd.u16CmdSN  = 0;//清除标记位
    Break_RangeFlag = 0;
    HostSpdLife = 2000;         //两秒没有收到上位机速度指令，需要对速度清零
    Com_success_Cnt++;
    return Err_None;
}


/*
* @brief       复位接收函数---0x11
* @param       无
* @retval      无
*
*/
int8_t  HostCom_ResetCommand ( void )      //
{
    if ( 0 == m_Reset.u16SN )
        return Err_NotData;
    m_sCommandAck.unResetAck = m_Reset.u16SN;
    if ( m_Reset.u16Delay < 10 )
    {
        m_Reset.u16Delay = 10;
    }
    m_Reset.u16SN = 0;
    return Err_None;
}


/*
* @brief       电源开关接收函数---0x16
* @param       无
* @retval      无
*
*/

int8_t  HostCom_PowerSwCommand ( void )      //
{
    if ( 0 == m_PowerSwCmd.u16SN )
        return Err_NotData;
    m_sCommandAck.unPowerSwAck = m_PowerSwCmd.u16SN;
    if ( m_PowerSwCmd.u16Delay < 10 )
    {
        m_PowerSwCmd.u16Delay = 10;
    }
    
    m_PowerSwCmd.u16SN = 0;
    return Err_None;
}
int8_t HostCom_ReadTelescopicMBDRelayCommand ( void )
{
     if ( 0 == m_TelescopicMBDRelayCmd.u16SN )
        return Err_NotData;
    m_sCommandAck.unTelescopicRelayAck = m_TelescopicMBDRelayCmd.u16SN; 
    RegReal_TelescopicMBDRelayCmd.u16EN = 1;
    RegReal_TelescopicMBDRelayCmd.u16SN = m_TelescopicMBDRelayCmd.u16SN;
    RegReal_TelescopicMBDRelayCmd.u16Cmd = m_TelescopicMBDRelayCmd.u16Cmd;
//    RegReal_TelescopicMBDRelayCmd.u16RunTime = m_TelescopicMBDRelayCmd.u16Delay;
    m_TelescopicMBDRelayCmd.u16SN = 0;
    return Err_None;
}
/*
* @brief       开合电机控制数据传送函数
* @param       无
* @retval      无
*
*/
//int8_t HostCom_ReadClampCommand ( void )
//{
////    if ( 0 == m_sCurClampCmd.u16SN )
////        return Err_NotData;
////    SolenoidCtrlCmd.Claw.CtrlMode = m_sCurClampCmd.CtrlMode;
////    SolenoidCtrlCmd.Claw.unPos = m_sCurClampCmd.unOpenDegree;
////    SolenoidCtrlCmd.Claw.unSN = m_sCurClampCmd.u16SN;
////    SolenoidCtrlCmd.Claw.EN = 1;
////    SolenoidCtrlCmd.CmdSet.bit.Claw = 1;
////    SolenoidCmddown.ClawEndOfCtrl = 1;       //收到升降指令后默认将EndOfCtrl置1，且需要50ms不变，待液压板上报EndOfCtrl为0时判定为运动结束
////    ClawCmdCnt = 0;
////    m_sCurClampCmd.u16SN = 0;//清除标记位
//    Com_success_Cnt++;
//    return Err_None;
//}

/*
* @brief       反馈模式及SN传送函数
* @param       无
* @retval      无
*
*/
int8_t HostCom_ReadDrvModeCommand ( int16_t* pnDrvMode )
{
    if ( 0 == m_sCurModeCmd.u16SN )
        return Err_NotData;
    *pnDrvMode = m_sCurModeCmd.unDriverMode;
    m_sCommandAck.unSetModeAck = m_sCurModeCmd.u16SN;
    m_sCurModeCmd.u16SN = 0;//清除标记位
    Com_success_Cnt++;
    return Err_None;
}
/*
* @brief       俯仰电机控制数据传送函数
* @param       无
* @retval      无
*
*/
int8_t HostCom_ReadBowRiseCommand ( void )
{
//    if ( 0 == m_sCurBowRiseCmd.u16SN )
//        return Err_NotData;
//    SolenoidCtrlCmd.BowRise.CtrlMode = m_sCurBowRiseCmd.CtrlMode;
//    SolenoidCtrlCmd.BowRise.unPos = m_sCurBowRiseCmd.unBowRiseDegree;
//    SolenoidCtrlCmd.BowRise.unSN = m_sCurBowRiseCmd.u16SN;
//    SolenoidCtrlCmd.BowRise.EN = 1;
//    SolenoidCmddown.BowRiseEndOfCtrl = 1;      //收到升降指令后默认将EndOfCtrl置1，且需要50ms不变，待液压板上报EndOfCtrl为0时判定为运动结束
//    BowRiseCmdCnt = 0;
//    SolenoidCtrlCmd.CmdSet.bit.BowRise = 1;
//    m_sCurBowRiseCmd.u16SN = 0;//清除标记位
    Com_success_Cnt_1++;
    return Err_None;
}
/**
 * @brief       Boot模式，
 * @param       无
 * @retval      无
 */
void BootProcess ( void )
{
    Set_BootFlg();
    __disable_irq();
    __NVIC_SystemReset();
}
/*
* @brief       旋转电机控制数据传送函数
* @param       无
* @retval      无
*
*/
//int8_t HostCom_ReadRotateCommand ( void )
//{
//    if ( 0 == m_sCurRotateCmd.u16SN )
//        return Err_NotData;
////    SolenoidCtrlCmd.Rotate.unPos = m_sCurRotateCmd.unRotateDegree;
////    SolenoidCtrlCmd.Rotate.unSN = m_sCurRotateCmd.u16SN;
//    SolenoidCtrlCmd.Rotate.EN = 1;
//    SolenoidCmddown.RotateEndOfCtrl = 1;      //收到升降指令后默认将EndOfCtrl置1，且需要50ms不变，待液压板上报EndOfCtrl为0时判定为运动结束
//    RotateCmdCnt = 0;
//    SolenoidCtrlCmd.CmdSet.bit.Rotate = 1;
//    m_sCurRotateCmd.u16SN = 0;//清除标记位
////    if ( abs ( m_sCurRotateCmd.unRotateDegree - SolenoidReport.unRotateDegree ) < 910 ) //小于5°
////    {
////        RotatePumpMotorSpd = 700;
////    }
////    else
////    {
////        RotatePumpMotorSpd = 1000;
////    }
//    return Err_None;
//}

/*
* @brief       转向码盘标定ack函数
* @param       无
* @retval      无
*
*/
int8_t SteerCalaCommand ( void )     //转向码盘标定
{
    if ( 0 == m_sSteerCalaCmd.u16SN )
        return Err_NotData;
    m_sCommandAck.unSteerCalaAck = m_sSteerCalaCmd.u16SN;
    m_sSteerCalaCmd.u16SN = 0;
    return Err_None;
}

/*
* @brief       函数
* @param       无
* @retval      无
*
*/
int8_t HostCom_ReadSideShiftCommand ( void )
{
    if ( 0 == m_sCurSideShiftCmd.u16SN )
        return Err_NotData;
//    SolenoidCtrlCmd.SideShift.CtrlMode = m_sCurSideShiftCmd.CtrlMode;
    SolenoidCtrlCmd.SideShift.unPos = m_sCurSideShiftCmd.unSideShiftDegree;
    SolenoidCtrlCmd.SideShift.unSN = m_sCurSideShiftCmd.u16SN;
    SolenoidCtrlCmd.SideShift.EN = 1;
    SolenoidCmddown.SideShiftEndOfCtrl = 1;      //收到升降指令后默认将EndOfCtrl置1，且需要50ms不变，待液压板上报EndOfCtrl为0时判定为运动结束
    SideShiftCmdCnt = 0;
    SolenoidCtrlCmd.Weight = m_sCurSideShiftCmd.Weight;
    SolenoidCtrlCmd.CmdSet.bit.SideShift = 1;
    m_sCommandAck.unSideShiftAck = m_sCurSideShiftCmd.u16SN;
    m_sCurSideShiftCmd.u16SN = 0;//清除标记位
    return Err_None;
}
/*
* @brief       更新函数
* @param       无
* @retval      无
*
*/
int8_t HostCom_RadLiftCommand ( void )
{
    if ( 0 == m_sRadLift.u16SN )
        return Err_NotData;
    m_sCommandAck.unRadLiftAck = m_sRadLift.u16SN;
    SolenoidCtrlCmd.RadLift.unSN = m_sRadLift.u16SN;
    SolenoidCtrlCmd.RadLift.unPos = m_sRadLift.RadLiftPos;
    SolenoidCtrlCmd.RadLift.EN = 1;
    SolenoidCmddown.Rad_Lift_EndOfCtrl = 1;     //收到指令后，将EndOfCtrl置1
    SolenoidCtrlCmd.CmdSet.bit.RadLift = 1;
    m_sRadLift.u16SN = 0;//清除标记位
    LiftRaderCmdCnt = 0;
    Com_success_Cnt++;
    return Err_None;
}

/*
* @brief       灯光控制函数--无用
* @param       无
* @retval      无
*
*/
int8_t Sound_LigntCommand ( void )     //声光控制
{
    if ( 0 == m_sSound_LigntCmd.u16SN )
        return Err_NotData;
    if ( m_sSound_LigntCmd.Ctrl_ID == 2 )
    {
        PC_Rader_Stop = m_sSound_LigntCmd.Switch;//上位机灯光控制指令
    }
    m_sCommandAck.unSoundLightAck = m_sSound_LigntCmd.u16SN;
    m_sSound_LigntCmd.u16SN = 0;

    return Err_None;
}

/*
* @brief       码盘位置标定初始化ack函数
* @param       无
* @retval      无
*
*/
int8_t EncoderCalaCommand ( void )     //码盘位置标定初始化
{
    if ( 0 == m_sEncoderCalaCmd.u16SN )
        return Err_NotData;
    m_sCommandAck.unEncoderCalaAck = m_sEncoderCalaCmd.u16SN;
    m_sEncoderCalaCmd.u16SN = 0;
    return Err_None;
}

/*
* @brief       泵电机ack使能函数
* @param       无
* @retval      无
*
*/
//int8_t PumpEnableCommand ( void )     //泵电机使能
//{
//    if ( 0 == m_sPumpEnableCmd.u16SN )
//        return Err_NotData;
//    m_sCommandAck.unPumpEnableAck = m_sPumpEnableCmd.u16SN;

//    m_sPumpEnableCmd.u16SN = 0;
//    return Err_None;
//}
/*
* @brief       协议版本握手ack函数
* @param       无
* @retval      无
*
*/
//int8_t HandAskCommand ( void )     //协议版本握手
//{
//    if ( 0 == m_sHandAskCmd.u16SN )
//        return Err_NotData;
//    m_sCommandAck.unHandAck = m_sHandAskCmd.u16SN;
//    if ( ( m_sHandAskCmd.Version_1 != ProtocolVer_1 ) || ( m_sHandAskCmd.Version_2 != ProtocolVer_2 ) )
//    {
//        FaultCtrl.bit.Protocol_VerErr = 1;            //协议版本错误 不可自恢复
//    }
//    m_sHandAskCmd.u16SN = 0;
//    return Err_None;
//}
/*
* @brief       上位机串口接收函数
* @param       无
* @retval      无
*
*/
void HostCom_ResolveCommand ( uint8_t* pbData, uint16_t unDataLen )
{
    uint16_t unCurPos = 0;
    HostProtocalHeader* psHeader = NULL;
    while ( unCurPos < MaxRecvDataLength ) //255
    {
        if ( ( unCurPos + ProtocalHeader_Length + ProtocalTail_Length ) > unDataLen ) //剩余数据不足一包头，认为已处理完
            break;
        psHeader = ( HostProtocalHeader* ) ( pbData + unCurPos );//帧头位置
        if ( 0xfee1 != psHeader->unSyncWord ) //查找协议头
        {
            unCurPos++;
            continue;
        }
        hostCnt2++;//？
        if ( ( unDataLen - unCurPos ) < ( psHeader->unDataLen + ProtocalTail_Length ) ) //不完整包，跳过
            break;

        if ( 1 == HostCom_CheckCrc ( pbData + unCurPos, psHeader->unDataLen, unDataLen - unCurPos ) )
        {
            hostCnt3++;
            HostCom_ProcCommand ( pbData + unCurPos, psHeader->unDataLen );//接收解析
            unCurPos += psHeader->unDataLen + ProtocalTail_Length;//
        }
        else
        {
            unCurPos++;
        }
    }
    return;
}
/*
* @brief       CRC校验函数
* @param       无
* @retval      无
*
*/
int8_t HostCom_CheckCrc ( uint8_t* pbData, uint16_t unPackLen, uint16_t unDatakLen )
{
    if ( unDatakLen < ( unPackLen + ProtocalTail_Length ) )
        return 0;
    if ( * ( uint16_t* ) ( pbData + unPackLen ) == Modbus_GetCRC16 ( pbData, unPackLen ) )
        return 1;
    else
        return 0;
}
/*
* @brief       上位机串口接收解析函数
* @param       无
* @retval      无
*
*/
void HostCom_ProcCommand ( uint8_t* pbData, uint16_t unDataLen )
{
    RunStates nDriveMode = RunSta_Idle;
    HostProtocalHeader* psHeader = ( HostProtocalHeader* ) pbData;
    //CRC
    pbData += sizeof ( HostProtocalHeader );//剔除帧头/时间戳/数据包长度

    m_nHostLife = ParamHandle_Control.Reg_PCDogTimeOut;//500
    FaultCtrl.bit.HostLinkLose = 0;//自动清除
    switch ( psHeader->unMsgID )
    {
    case MessageID_SteerCmd://转向指令--1
        memcpy ( &m_sCurSteerCmd, pbData, sizeof ( SteerCommand ) );
        break;
    case MessageID_GasCmd://VAG控制指令--2
        memcpy ( &m_sCurGasCmd, pbData, sizeof ( GasCommand ) );
        break;
    case MessageID_LiftCmd://升降指令---3
        memcpy ( &m_sCurLiftCmd, pbData, sizeof ( m_sCurLiftCmd ) );
        break;
    case MessageID_HuggingCmd://环抱指令---4
        memcpy ( &m_HuggingCmd, pbData, sizeof ( m_HuggingCmd ) );
        break;
    case MessageID_AutoCharCmd://自动充电---5
        memcpy ( &m_sAutoCharCmd, pbData, sizeof ( AutoCharCommand ) );
        break;
    case MessageID_GlandCmd://抬压指令---6
        memcpy ( &m_GlandCmd, pbData, sizeof ( m_GlandCmd ) );
        break;
    case MessageID_TelescopicCmd://伸缩指令----7
        memcpy ( &m_TelescopicCmd, pbData, sizeof ( m_TelescopicCmd ) );
        break;
    case MessageID_RotateCmd://旋转指令---8
        memcpy ( &m_sCurRotateCmd, pbData, sizeof ( m_sCurRotateCmd ) );
        break;
    case MessageID_BrakeCmd://刹车指令----9
        memcpy ( &m_sCurBrakeCmd, pbData, sizeof ( m_sCurBrakeCmd ) );
        break;
    case MessageID_RadarCmd://避障雷达开关---10
        memcpy ( &m_sRadarCmd, pbData, sizeof ( m_sRadarCmd ) );
        break;
    case MessageID_FaultClearCmd://故障清除---11
        memcpy ( &m_sFaultClear, pbData, sizeof ( m_sFaultClear ) );
        m_sCommandAck.unFaultClearAck = m_sFaultClear.u16SN;
        break;
    case MessageID_BreakCanelCmd://导航刹车---12
        memcpy ( &m_sBreakCanel, pbData, sizeof ( m_sBreakCanel ) );
        break;
    case MessageID_SlowSpdCmd://车辆小位置缓行（蠕动模式）--15
        memcpy ( &m_sSlow_SpdCmd, pbData, sizeof ( m_sSlow_SpdCmd ) );
        break;
    case MessageID_ElectronMagCmd://--16
        memcpy ( &m_sElectronMagCmd, pbData, sizeof ( m_sElectronMagCmd ) );
        break;
    case MessageID_WatchDogCmd://心跳--
        memcpy ( &m_WatchDogCmd, pbData, sizeof ( m_WatchDogCmd ) );
        break;
    case MessageID_Reset://复位--0x11
    {
        memcpy ( &m_Reset, pbData, sizeof ( m_Reset ) );
    }
    break;
    case MessageID_SteerCalaCmd://标定  --0x13
        memcpy ( &m_MotorCala, pbData, sizeof ( m_MotorCala ) );
        break;
    case MessageID_ParamMngCmd://参数管理 --0x0D
    {
        memcpy ( &m_ParamMngCmd, pbData, sizeof ( m_ParamMngCmd ) );
        g_u16ParamStep = 1;
    }
    break;
    case MessageID_BootLinkCmd://升级请求
    {
        memcpy ( &m_BootLinkCmd, pbData, sizeof ( m_BootLinkCmd ) );
        u8BootLinkFlag = 1;
//        Set_BootFlg();
//        __disable_irq();
//        __NVIC_SystemReset();
    }
    break;
    case MessageID_PowerSwCmd://电源开关--0x16
    {
        memcpy ( &m_PowerSwCmd, pbData, sizeof ( m_PowerSwCmd ) );
    }
    case MessageID_TelescopicMBDRelayCmd://伸缩继电器开关--0x17
    {
        memcpy ( &m_TelescopicMBDRelayCmd, pbData, sizeof ( m_TelescopicMBDRelayCmd ) );
    }
    }
    return ;
}
/*
* @brief       故障后SN清除函数（没用）
* @param       无
* @retval      无
*
*/
void HostCom_ClearCommand()
{
    m_sCurGasCmd.u16SN = 0;//行进
    m_sCurLiftCmd.u16SN = 0;//升降
    m_sCurSteerCmd.u16SN = 0;//转向
    m_HuggingCmd.u16SN = 0;//环抱
    m_sAutoCharCmd.u16SN = 0;//充电
    m_GlandCmd.u16SN = 0;//抬压
    m_TelescopicCmd.u16SN = 0;//伸缩
    m_sCurRotateCmd.u16SN = 0;//旋转
    m_sSlow_SpdCmd.u16SN = 0;//蠕动
    m_sCurBrakeCmd.u16SN = 0;//刹车
    m_sBreakCanel.u16SN = 0;//导航刹车
    m_TelescopicMBDRelayCmd.u16SN = 0;
//    m_sCurClampCmd.u16SN = 0;
//    m_sCurBowRiseCmd.u16SN = 0;
//    m_sCurRotateCmd.u16SN = 0;
//    m_sCurBrakeCmd.u16SN = 0;
//    m_sCurModeCmd.u16SN = 0;
}
/*
* @brief       复位函数（没用）
* @param       无
* @retval      无
*
*/
//void HostCom_ResetCommand ( void )
//{
//    m_sCurGasCmd.u16SN = 0;
//    m_sCurGasCmd.nWheelSpeed = 0;

//    m_sCurLiftCmd.u16SN = 0;

//    m_sCurClampCmd.u16SN = 0;

//    m_sCurBowRiseCmd.u16SN = 0;

//    m_sCurRotateCmd.u16SN = 0;

//    m_sCurSteerCmd.u16SN = 0;
//    m_sCurSteerCmd.nSteerPos = _IQsat ( g_u16SteeringWheelAbsPos, 32767, 0 );//当前位置
//}
/*
* @brief       上位机数据接收函数
* @param       无
* @retval      无
*
*/
//1000hz接收，100hz处理
int8_t HostCom_ProcRecvData()
{
    static uint16_t nTimeCounter = 0;
    m_unRecvLen = MaxRecvDataLength;

    if ( 1 != Uart_Host_Receive ( m_bRecvBuf, &m_unRecvLen ) )
    {
        m_unRecvLen = 0;
    }
#ifdef EmbDebug
    HostCom_Test();
#else
    if ( 0 != m_unRecvLen )
    {
        hostCnt1 ++;
        HostCom_ResolveCommand ( m_bRecvBuf, m_unRecvLen );//上位机接收函数
        m_unRecvLen = 0;
        memset ( m_bRecvBuf, 0, MaxRecvDataLength );
    }
#endif

    nTimeCounter++;
    if ( nTimeCounter <= 10 )
        return Err_None;

    nTimeCounter = 0;
    return Err_None;
}
/*
* @brief       发送参数管理ACK给上位机函数
* @param       无
* @retval      无
*
*/
void HostCom_ParamSendReport ( RegDef_ParamMngCmdAck* pParamMngCmdAck )
{
    g_u32ParamMngTimeTick++;
    pParamMngCmdAck->ParamMngHeader.unSyncWord = 0xfee1;
    pParamMngCmdAck->ParamMngHeader.unMsgID = 0x0E;
    pParamMngCmdAck->ParamMngHeader.unTimeStamp = g_u32ParamMngTimeTick;
    pParamMngCmdAck->ParamMngHeader.unDataLen = sizeof ( RegDef_ParamMngCmdAck ) - ProtocalTail_Length;
    pParamMngCmdAck->u16Crc = Modbus_GetCRC16 ( ( uint8_t * ) pParamMngCmdAck, sizeof ( RegDef_ParamMngCmdAck ) - ProtocalTail_Length );
    Uart_Host_Send ( ( uint8_t * ) pParamMngCmdAck, sizeof ( RegDef_ParamMngCmdAck ) );
}

static uint16_t HostPowSendConv(_iq data)
{
    return (((data) * 10) >> 16); 
}
/*
* @brief       发送ACK给上位机函数  --50hz
* @param       无
* @retval      无
*
*/
//由于码盘更新频率为50Hz，更改上报频率为50Hz
_iq iqVCUTurnDegFB,iqAGVLiftPos;
int16_t s16VCUTurnDegFB,s16VCUWalkSpdFB;
int32_t s32VCUWalkSpdFB;
uint16_t u16AGVWalkPosFb, u16SENSORVlot;

int8_t HostCom_SendReport()
{
    uint8_t nRetCode = 0;
    uint16_t u16Tem;
    
    RunStates nDriveMode = RunSta_Idle;

    g_unHostTimeTick++;

    nDriveMode = GetRUNSta();    //从状态机取当前状态

    psStatus->sHeader.unSyncWord = 0xfee1;
    psStatus->sHeader.unDataLen = sizeof ( StatusReport );
    psStatus->sHeader.unMsgID = 101;
    psStatus->sHeader.unTimeStamp = g_unHostTimeTick;
    psStatus->Ver_1 = 0x01;
    psStatus->Ver_2 = 0x02;
//**********************************故障状态*********************************************//
    psStatus->SlaveFaultNum = FaultCtrl.all;
    psStatus->SlaveWarning = Warning_Report.all;
    psStatus->RotatingFaultNum = RegReal_RotatingMBDFaultStaFB.u32Fault;
    psStatus->RotatingWarning = RegReal_RotatingMBDFaultStaFB.u32Warning;
    
    psStatus->TelescopicFaultNum = RegReal_TelescopicMBDFaultStaFB.u32Fault;
    psStatus->TelescopicWarning = RegReal_TelescopicMBDFaultStaFB.u32Warning;
    psStatus->HuggingFaultNum = RegReal_HuggingMBDFaultStaFB.u32Fault;
    psStatus->HuggingWarning = RegReal_HuggingMBDFaultStaFB.u32Warning;
    
    psStatus->GlandFaultNum = RegReal_GlandMBDFaultStaFB.u32Fault;
    psStatus->GlandWarning = RegReal_GlandMBDFaultStaFB.u32Warning;
    psStatus->AGVErr1 =  Reg_AGVDataFb1.u8WalkFaultCode;
    psStatus->AGVErr2 =  Reg_AGVPumpCtrFb.u8WalkFaultCode;
    iqAGVLiftPos = _IQrmpy(u32LiftPos, CONSTANT_1500_DIV_1024);
//    iqAGVLiftPos = _IQrmpy(SolenoidCtrlCmd.Lift.unPos, CONSTANT_1500_DIV_1024);
    psStatus->AGVLiftPosFB = ( uint16_t ) iqAGVLiftPos; //升降高度
    s32VCUWalkSpdFB = -AgvWorkSpdToPCData ( Reg_AGVDataFb1.s16WalkSpd );
    if(s32VCUWalkSpdFB > 32767)  
    {
        s16VCUWalkSpdFB = 32767;
    }
    else if(s32VCUWalkSpdFB < -32768)
    {
        s16VCUWalkSpdFB = -32768;
    }
    else
    {
        s16VCUWalkSpdFB = (int16_t)s32VCUWalkSpdFB;
    }
    psStatus->AGVWorkSpdFB =  s16VCUWalkSpdFB;//

    psStatus->RotatingPosFB =  RegReal_RotatingMBDCtrlFB.u16Ack;//旋转位置反馈
    psStatus->TelescopicPosFB =  RegReal_TelescopicMBDCtrlFB.u16Ack;
    psStatus->HuggingPosFB =  RegReal_HuggingMBDCtrlFB.u16Ack;
    psStatus->GlandPosFB =  RegReal_GlandMBDCtrlFB.u16Ack;

    psStatus->BMSFlag = Reg_AGVBMSStaOrigDataFb.Staflag;
    psStatus->BMSSoc = Reg_AGVBMSStaOrigDataFb.Soc;
    psStatus->BMSCycle = * ( uint16_t * ) &Reg_AGVBMSStaOrigDataFb.CycleTimes;
    psStatus->BMSCurr = * ( uint16_t * ) &Reg_AGVBMSStaOrigDataFb.CurrSum;

    psStatus->BMSMaxTmp =  Reg_AGVBMSStaOrigDataFb.MaxTemp;
    psStatus->BMSErr =  Reg_AGVBMSStaOrigDataFb.u8FaultCode;
    iqVCUTurnDegFB = INVReduction_Ratio ( Reg_AGVTurnFb.s32MotorPosFb );//Reg_AGVTurnCtrlCmd1.s32MotorPosCmd.all 
    s16VCUTurnDegFB = (int16_t)iqVCUTurnDegFB;
    psStatus->VCUTurnDegFB = s16VCUTurnDegFB;//转向角度反馈
    
    psStatus->nImuRZ = g_sImuData.nImuRZ;
    psStatus->nImuRY = g_sImuData.nImuRY;
    psStatus->nImuRX = g_sImuData.nImuRX;
    psStatus->nImuRZSpd = g_sImuData.nImuRZSpeed;
    psStatus->nImuRYSpd = g_sImuData.nImuRYSpeed;
    psStatus->nImuRXSpd = g_sImuData.nImuRXSpeed;
    psStatus->nImuRZAcc = g_sImuData.nImuZAcc;
    psStatus->nImuRYAcc = g_sImuData.nImuYAcc;
    psStatus->nImuRXAcc = g_sImuData.nImuXAcc;
    psStatus->u8EMSta = ElecMagSta;//电磁铁状态
    psStatus->u8PowerSwSta = m_PowerSwCmd.PowerSwOpt.all;
    psStatus->u8AutoChargeSta = Reg_AGVChargeSWFb.u8SwStaFb;//自动充电继电器状态
    
    psStatus->u8CurMode = m_eCtrlMode;                       //当前模式 RC PC HAND
    psStatus->u16PowerSwAck = m_sCommandAck.unPowerSwAck;//电源开关ack
    psStatus->AGVLiftAck = m_sCommandAck.unCurLiftAck;
    psStatus->AGVWalkAck = m_sCommandAck.unGasAck;

    psStatus->RotatingAck =  m_sCommandAck.unRotatingAck;//RegReal_RotatingMBDCtrlFB.u16SN;//
    psStatus->TelescopicAck = m_sCommandAck.unTelescopicAck;//RegReal_TelescopicMBDCtrlFB.u16SN; //
    psStatus->HuggingAck =  m_sCommandAck.unHuggingAck;//RegReal_HuggingMBDCtrlFB.u16SN;//
    psStatus->GlandAck =  m_sCommandAck.unGlandAck;//RegReal_GlandMBDCtrlFB.u16SN;//

    psStatus->VCUTurnAck =  m_sCommandAck.unSteerAck;//转向
    psStatus->FixedPointParkingAck =  m_sCommandAck.unBrakeAck;
    psStatus->ParkingNavigationAck =  m_sCommandAck.unBrakeCanelAck;
    psStatus->RadarAck = m_sCommandAck.unRaderAck;
    psStatus->FaultClearAck =  m_sCommandAck.unFaultClearAck;

    psStatus->ResetAck =  m_sCommandAck.unResetAck;
//    psStatus->MotorCalaAck =  m_sCommandAck.unMotorCalaAck;
    psStatus->TelescopicMBDRelayAck = m_sCommandAck.unTelescopicRelayAck;
    psStatus->u16AutoChargeAck = m_sCommandAck.unAutoCharAck;
    psStatus->u16ElectronMagAck = m_sCommandAck.unElectronMagAck;

    psStatus->GlandSta =  RegReal_GlandMBDCtrlFB.u16Sta;
    psStatus->HuggingSta =  RegReal_HuggingMBDCtrlFB.u16Sta;
    psStatus->TelescopicSta =  RegReal_TelescopicMBDCtrlFB.u16Sta;
    psStatus->RotatingSta =  RegReal_RotatingMBDCtrlFB.u16Sta;
    
    psStatus->RadarSta = RaderReport;
    psStatus->unBrakeSta = SolenoidCmddown.Brake_EndOfCtrl;
//    psStatus->unBrakeCanelSta = SolenoidCmddown.BreakCanel_EndOfCtrl;
    psStatus ->TelescopicMBDRelaySta = RegReal_TelescopicMBDRelayCmdFB.u16RelayCmdFb;
    psStatus->unSlowSpdSta = SolenoidCmddown.SlowSpd_EndOfCtrl;
    
    psStatus->AGVLiftPosSta = SolenoidCmddown.liftEndOfCtrl;
    psStatus->s16BrakePosFB = s16AGVWalkBrakePosFB;
    psStatus->unSlowSpdPosFB = AGVSlowSpdPosFB;
    psStatus->unSlowSpdAck = m_sCommandAck.unSlowSpdAck;
    
    
    u16AGVWalkPosFb = 65535 - iqAGVWalkPosFb;
    psStatus->u16AgvWalkDeg = u16AGVWalkPosFb;
    psStatus->u16RotatingEnc = RegReal_RotatingMBDRunningSta2FB.u16RunningSta1;
    psStatus ->TelescopicMBDNPNSta = RegReal_TelescopicMBDRelayCmdFB.u16TouchSwSta;
    u16SENSORVlot = HostPowSendConv(Energy_SENSOR.U);
    psStatus->res2 = u16SENSORVlot;
    
    psStatus->res3 = HostPowSendConv(Energy_PC.P);
    psStatus->res4 = HostPowSendConv(Energy_SENSOR.P);
    psStatus->res5 = HostPowSendConv(Energy_MOTOR1.P + Energy_MOTOR2.P);
    
    psStatus->res6 = HostPowSendConv(Energy_PC.W_Sum);
    psStatus->res7 = HostPowSendConv(Energy_SENSOR.W_Sum);
    psStatus->res8 = HostPowSendConv(Energy_MOTOR1.W_Sum + Energy_MOTOR2.W_Sum);
    

    * ( uint16_t* ) ( m_bReportBuf + sizeof ( StatusReport ) ) = Modbus_GetCRC16 ( m_bReportBuf, sizeof ( StatusReport ) );
    nRetCode = Uart_Host_Send ( m_bReportBuf, sizeof ( StatusReport ) + ProtocalTail_Length );
    HostTextCnt = sizeof ( StatusReport );
    memset ( m_bReportBuf, 0, sizeof ( StatusReport ) );
    return Err_None;
}




//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


