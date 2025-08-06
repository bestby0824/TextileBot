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
#include "VCUProcess.h"
#include "CAN_VCU.h"
#include "string.h"
#include "IMU_Com.h"
#include "main.h"
#include "VCU_Ctrl.h"
#include "StateCtrl.h"
#include "Host_Com.h"
#include "NodeProcess.h"
#include "Xint.h"
#include "SteeringWheelProcess.h"


//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
uint8_t u8TransferMode = 0;

//-------------------- private functions declare ----------------------------

//-------------------- public data ------------------------------------------
UserFrame_t Tx_frame;
RegDef_ChassisParam RegReal_ChassisParam;  //寄存器0x201
RegDef_ForkParam RegReal_ForkParam;  //寄存器0x301
RegDef_PartsAndValve1Param RegReal_PartsAndValve1Param;  //寄存器0x401
RegDef_PartsAndValve2Param RegReal_PartsAndValve2Param;  //寄存器0x501
RegDef_ForkLiftState1 RegReal_ForkLiftState1;  //寄存器0x181
RegDef_ForkLiftState2 RegReal_ForkLiftState2;  //寄存器0x281
RegDef_ForkLiftState3 RegReal_ForkLiftState3;  //寄存器0x381
RegDef_ForkLiftState4 RegReal_ForkLiftState4;  //寄存器0x481
RegDef_ForkLiftState5 RegReal_ForkLiftState5;  //寄存器0x182
RegDef_ForkLiftState6 RegReal_ForkLiftState6;  //寄存器0x19E
RegDef_HARTCMD RegReal_HARTCMD;                //寄存器0x1A3，手操器指令
VCU_ParamStruct VCU_ParamHandle;               //VCU数据解析结构体
VCU_ParamStruct VCU_ParamHandle;  //VCU数据解析结构体
CanTransferTypeDef CanTransferStruct;          //CAN传输辅助字段
ForkLiftPos ForkLiftAllPos;                     //解析液压板反馈的位置信息
RCCnt RC_Speed_Cnt;                             //遥控运动速度控制
int16_t VCULife = -1;                       //VCU心跳计数

/*textile*/
RegDef_Reg_AGVCtrlCmd1 Reg_AGVCtrlCmd1;
RegDef_Reg_AGVCtrlCmd2 Reg_AGVCtrlCmd2;
RegDef_AGVBMSCtrlCmd Reg_AGVBMSCtrlCmd;
RegDef_Reg_AGVTurnCtrlCmd1 Reg_AGVTurnCtrlCmd1;
RegDef_AGVTurnCtrlCmd2 Reg_AGVTurnCtrlCmd2;
RegDef_AGVChargeSWCmd Reg_AGVChargeSWCmd;

RegDef_AGVBMSStaOrigDataFb Reg_AGVBMSStaOrigDataFb;
RegDef_AGVDataFb1 Reg_AGVDataFb1;
RegDef_AGVDataFb2 Reg_AGVDataFb2;
RegDef_AGVPumpCtrFb Reg_AGVPumpCtrFb;
RegDef_AGVTurnFb Reg_AGVTurnFb;
RegDef_AGVChargeSWFb Reg_AGVChargeSWFb;
RegDef_RCCtrlCmdFb Reg_RCCtrlCmdFb;
RegDef_RCWarningFb Reg_RCWarningFb;
uint8_t AGVTurnCtrlCmdSendFlag2 = 0, AGVTurnCtrlCmdSendFlag1 = 1;
//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
//-------------------- private functions declare ----------------------------
static int CanTask_Send ( uint32_t Cmd_ID );
//static int HyDriveBoard_ProcRecvData( uint8_t *PayLoad );

//-------------------- public data ------------------------------------------
uint16_t RCSpeed = 4000;  //4000 - 0.66m/s
//uint16_t RCSpeed = 5500;  //4000 - 0.66m/s
//float SpeedKP = 1;
//-------------------- public functions -------------------------------------
void Can_VCU_Receive ( void );
void CanTaskProcess ( void );

/**
 * @brief       VCU对应寄存器状态发送static
 * @param       寄存器ID
 * @retval      无
 */
int CanTask_Send ( uint32_t Cmd_ID )
{
    static uint8_t res = 0;
    Tx_frame.ID = Cmd_ID;
    Tx_frame.Len = 8;

    switch ( Cmd_ID )
    {
    /*textile*/
    case Step_AGVCtrlCmd1:
    {
        memcpy ( ( uint8_t * ) &Tx_frame.Buff, ( uint8_t * ) &Reg_AGVCtrlCmd1, 8 );
    }
    break;

    case Step_AGVCtrlCmd2:
    {
        memcpy ( ( uint8_t * ) &Tx_frame.Buff, ( uint8_t * ) &Reg_AGVCtrlCmd2, 8 );
    }
    break;

    case Step_AGVBMSCtrlCmd:
    {
        memcpy ( ( uint8_t * ) &Tx_frame.Buff, ( uint8_t * ) &Reg_AGVBMSCtrlCmd, 8 );
    }
    break;
    
    case Step_AGVTurnCtrlCmd1:
    {
       
       // memcpy ( ( uint8_t * ) &Tx_frame.Buff, ( uint8_t * ) &Reg_AGVTurnCtrlCmd1, 7 );
        Tx_frame.Buff[0] = Reg_AGVTurnCtrlCmd1.s32MotorPosCmd.bit[0];
        Tx_frame.Buff[1] = Reg_AGVTurnCtrlCmd1.s32MotorPosCmd.bit[1];
        Tx_frame.Buff[2] = Reg_AGVTurnCtrlCmd1.s32MotorPosCmd.bit[2];
        Tx_frame.Buff[3] = Reg_AGVTurnCtrlCmd1.s32MotorPosCmd.bit[3];
        Tx_frame.Buff[4] = Reg_AGVTurnCtrlCmd1.u8WorkMode;
        Tx_frame.Buff[5] = Reg_AGVTurnCtrlCmd1.u16CtrlWord.bit[0];
        Tx_frame.Buff[6] = Reg_AGVTurnCtrlCmd1.u16CtrlWord.bit[1];
        Tx_frame.Len = 7;
    }
    break;
    
    case Step_AGVTurnCtrlCmd2:
    {
        memcpy ( ( uint8_t * ) &Tx_frame.Buff, ( uint8_t * ) &Reg_AGVTurnCtrlCmd2, 8 );
    }
    break;
    case Step_AGVChargeSW:
    {
       
//        Reg_AGVChargeSWCmd.u8Data[0] = 0xF0;
        memcpy ( ( uint8_t * ) &Tx_frame.Buff, ( uint8_t * ) &Reg_AGVChargeSWCmd, 8 );
    }
    break;
//    case Step_DrawWireSensor:
//    {
//         memcpy ( ( uint8_t * ) &Tx_frame.Buff, ( uint8_t * ) &Reg_DrawWireSensor, Reg_DrawWireSensor.u8DataLen );
//         Tx_frame.Len = Reg_DrawWireSensor.u8DataLen;
//    }
//    break;
    default:
        break;
    }

    res = Can_VCU_send_msg ( Tx_frame.ID, ( uint8_t * ) &Tx_frame.Buff, Tx_frame.Len );

    return res;
}
uint32_t Rx_ID;
/**
 * @brief       接收CAN1上数据并分类填充到VCU结构体
 * @param       无
 * @retval      无
 */
void Can_VCU_Receive ( void )
{
    
    static uint8_t Rx_Buffer[8];

    Rx_ID = Can_VCU_receive_msg ( Rx_Buffer );
    VCULife  = ParamHandle_Control.Reg_VCUDogTimeOut;

    switch ( Rx_ID )
    {
    case Reg_IMUState:
    {
        ;
    }
    break;

/*textile*/
    case Step_AGVBMSStaOrigDataFb:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &Reg_AGVBMSStaOrigDataFb, Rx_Buffer, 8 );
        __enable_irq();          // 开启总中断
    }
    break;

    case Step_AGVDataFb1:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &Reg_AGVDataFb1, Rx_Buffer, 8 );
        __enable_irq();          // 开启总中断
    }
    break;

    case Step_AGVDataFb2:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &Reg_AGVDataFb2, Rx_Buffer, 8 );
        __enable_irq();          // 开启总中断
    }
    break;

    case Step_AGVPumpCtrFb:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &Reg_AGVPumpCtrFb, Rx_Buffer, 8 );
        __enable_irq();          // 开启总中断
    }
    break;
    
    case Step_AGVTurnFb:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &Reg_AGVTurnFb, Rx_Buffer, 8 );
        __enable_irq();          // 开启总中断
    }
    break;
    case Step_AGVChargeSW:
    {
          __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &Reg_AGVChargeSWFb, Rx_Buffer, 8 );
        __enable_irq();          // 开启总中断
     }
     break;
    case Step_RCCtrlCmdFb:
    {
        __disable_irq();         //关闭总中断
        memcpy ( ( uint8_t * ) &Reg_RCCtrlCmdFb, Rx_Buffer, 8 );
        __enable_irq(); 
       
    }
     break;
    default:
        break;
    }
}
/**
 * @brief       VCU对应寄存器状态发送，20HZ处理
 * @param       无
 * @retval      1k
 */

void CanTaskProcess ( void )
{
    static uint8_t u8CanTaskStep = 0;
    int testflag;
    if ( u8CanTaskStep < 200 ) u8CanTaskStep++;
    else u8CanTaskStep = 0;

    switch ( u8CanTaskStep % 10)//100Hz
    {
    case 0:
    {
        testflag = CanTask_Send ( Step_AGVCtrlCmd1 );
    }

    case 1:
    {
        testflag = CanTask_Send ( Step_AGVCtrlCmd2 );
    }

    case 2:
    {
        testflag = CanTask_Send ( Step_AGVBMSCtrlCmd );
    }
    break;
    
     case 3:
    {

        testflag = CanTask_Send ( Step_AGVTurnCtrlCmd1 );
    }
    break;
    
     case 4:
    {
        testflag = CanTask_Send ( Step_AGVTurnCtrlCmd2 );
    }
    break;
    
    case 5:
    {
        testflag = CanTask_Send (Step_AGVChargeSW);
    }
    break;
    default:
        break;
    }
}

//**************************************遥控器控制接收处理逻辑*************************************************//
uint16_t LiftSN, ClawSN, BowRiseSN, RotateSN, SideSN, SteerSN; //自加SN
int RCLiftPos, RCClawPos, RCBowRisePos, RCRotatePos, RCSidePos; //位置指令中间值
#if 0
/*
* @brief       手操器函数
* @param       无
* @retval      无
*
*/
void Hart_SolenoidCmdInit ( void )    //遥控液压驱动板指令初始化
{
    RegReal_HARTCMD.u8HARTCMD_SteerAngle = 0;//角度指令归零
}
/*
* @brief       手操器控制液压驱动板函数 10hz
* @param       无
* @retval      无
*
*/

void Hart_SolenoidCmd ( void )    //遥控液压驱动板指令
{
    ForkLiftAllPos.LiftPos = g_sSolenoid_Info1[RodID_Lift].Pos;         //升降
    ForkLiftAllPos.ClawPos = g_sSolenoid_Info1[RodID_Claw].Pos;         //开合
    ForkLiftAllPos.BowRisePos = g_sSolenoid_Info1[RodID_BowRise].Pos;   //俯仰
    ForkLiftAllPos.RotatePos = g_sSolenoid_Info1[RodID_Rotate].Pos;     //旋转
    ForkLiftAllPos.SidePos = g_sSolenoid_Info1[RodID_SideShift].Pos;    //侧移
//侧移
    if ( RegReal_HARTCMD.u8HARTCMD_Forkside == 0xFF ) //侧移右移
    {
        RC_Speed_Cnt.SideMoveRightCnt++;
        if ( RC_Speed_Cnt.SideMoveRightCnt > 50 ) //最大5秒，系数为5
            RC_Speed_Cnt.SideMoveRightCnt = 50;

        RCSidePos = ForkLiftAllPos.SidePos - _IQ ( 0.01 ) * ( RC_Speed_Cnt.SideMoveRightCnt / 10 ); //ParamHandle_Control.Reg_RC_LiftPos
        m_sCurSideShiftCmd.u16SN = SideSN++;
        ForkLiftAllPos.SideState = 1;//
    }
    else if ( RegReal_HARTCMD.u8HARTCMD_Forkside == 0x01 ) //侧移左移
    {
        RC_Speed_Cnt.SideMoveLeftCnt++;
        if ( RC_Speed_Cnt.SideMoveLeftCnt > 50 ) //最大5秒，系数为5
            RC_Speed_Cnt.SideMoveLeftCnt = 50;
        RCSidePos = ForkLiftAllPos.SidePos + _IQ ( 0.01 ) * ( RC_Speed_Cnt.SideMoveRightCnt / 10 ); //ParamHandle_Control.Reg_RC_LiftPos
        m_sCurSideShiftCmd.u16SN = SideSN++;
        ForkLiftAllPos.SideState = 2;//
    }
    else
    {
        RC_Speed_Cnt.SideMoveRightCnt = 10;
        RC_Speed_Cnt.SideMoveLeftCnt = 10;

        if ( ForkLiftAllPos.SideState == 1 )
        {
            ForkLiftAllPos.SideState = 0;
            RCSidePos = ForkLiftAllPos.SidePos + _IQ ( 0.01 );//右移结束后回退
        }
        else if ( ForkLiftAllPos.SideState == 2 )
        {
            RCSidePos = ForkLiftAllPos.SidePos - _IQ ( 0.01 );//左移结束后回退
            ForkLiftAllPos.SideState = 0;
        }
    }
//升降    
    if ( RegReal_HARTCMD.u8HARTCMD_ForkLift == 0xFF ) //上升
    {
        RC_Speed_Cnt.LiftUpCnt++;
        if ( RC_Speed_Cnt.LiftUpCnt > 50 ) //最大5秒，系数为5
            RC_Speed_Cnt.LiftUpCnt = 50;

        RCLiftPos = ForkLiftAllPos.LiftPos + _IQ ( 0.01 ) * ( RC_Speed_Cnt.LiftUpCnt / 10 ); //ParamHandle_Control.Reg_RC_LiftPos
//        m_sCurLiftCmd.u16SN = LiftSN++;
        ForkLiftAllPos.LiftState = 1;
    }
    else if ( RegReal_HARTCMD.u8HARTCMD_ForkLift == 0x01 ) //下降
    {
        RC_Speed_Cnt.LiftDownCnt++;
        if ( RC_Speed_Cnt.LiftDownCnt > 50 ) //最大5秒，系数为5
            RC_Speed_Cnt.LiftDownCnt = 50;
        RCLiftPos = ForkLiftAllPos.LiftPos - _IQ ( 0.01 ) * ( RC_Speed_Cnt.LiftDownCnt / 10 ); //ParamHandle_Control.Reg_RC_LiftPos
//        m_sCurLiftCmd.u16SN = LiftSN++;
        ForkLiftAllPos.LiftState = 2;

    }
    else
    {
        RC_Speed_Cnt.LiftUpCnt = 10;
        RC_Speed_Cnt.LiftDownCnt = 10;

        if ( ForkLiftAllPos.LiftState == 1 )
        {
            ForkLiftAllPos.LiftState = 0;
            RCLiftPos = ForkLiftAllPos.LiftPos + _IQ ( 0.01 );//上升结束后多走点
        }
        else if ( ForkLiftAllPos.LiftState == 2 )
        {
            RCLiftPos = ForkLiftAllPos.LiftPos - _IQ ( 0.01 );//下降结束后多走点
            ForkLiftAllPos.LiftState = 0;
        }
    }
//开合
    if ( RegReal_HARTCMD.u8HARTCMD_ForkClaw == 0xFF ) //合
    {
        RC_Speed_Cnt.ClawClosCnt++;
        if ( RC_Speed_Cnt.ClawClosCnt > 50 ) //最大5秒，系数为5
            RC_Speed_Cnt.ClawClosCnt = 50;

        RCClawPos = ForkLiftAllPos.ClawPos + _IQ ( 0.01 ) * ( RC_Speed_Cnt.ClawClosCnt / 10 ); //ParamHandle_Control.Reg_RC_LiftPos
//        m_sCurClampCmd.u16SN = ClawSN++;
        ForkLiftAllPos.ClawState = 1;
    }
    else if ( RegReal_HARTCMD.u8HARTCMD_ForkClaw == 0x01 ) //开
    {
        RCClawPos = ForkLiftAllPos.ClawPos - _IQ ( 0.01 ); //ParamHandle_Control.Reg_RC_ClawPos
        RC_Speed_Cnt.ClawOpenCnt++;
        if ( RC_Speed_Cnt.ClawOpenCnt > 50 ) //最大5秒，系数为5
            RC_Speed_Cnt.ClawOpenCnt = 50;

        RCClawPos = ForkLiftAllPos.ClawPos - _IQ ( 0.01 ) * ( RC_Speed_Cnt.ClawOpenCnt / 10 ); //ParamHandle_Control.Reg_RC_LiftPos
//        m_sCurClampCmd.u16SN = ClawSN++;
        ForkLiftAllPos.ClawState = 2;
    }
    else
    {
        RC_Speed_Cnt.ClawClosCnt = 10;
        RC_Speed_Cnt.ClawOpenCnt = 10;

        if ( ForkLiftAllPos.ClawState == 1 )
        {
            ForkLiftAllPos.ClawState = 0;
            RCClawPos = ForkLiftAllPos.ClawPos + _IQ ( 0.005 );//环抱结束后再抱紧点
        }
        else if ( ForkLiftAllPos.ClawState == 2 )
        {
            RCClawPos = ForkLiftAllPos.ClawPos - _IQ ( 0.005 );//打开结束后再多打开些
            ForkLiftAllPos.ClawState = 0;
        }
    }
//俯仰    
    if ( RegReal_HARTCMD.u8HARTCMD_ForkBowrise == 0xFF ) //前倾
    {
        RC_Speed_Cnt.BowRiseForCnt++;
        if ( RC_Speed_Cnt.BowRiseForCnt > 50 ) //最大5秒，系数为5
            RC_Speed_Cnt.BowRiseForCnt = 50;

        RCBowRisePos = ForkLiftAllPos.BowRisePos +  _IQ ( 0.0169 ) * ( RC_Speed_Cnt.BowRiseForCnt / 10 ); //ParamHandle_Control.Reg_RC_LiftPos
        ForkLiftAllPos.RowRiseState = 1;
//        m_sCurBowRiseCmd.u16SN = BowRiseSN++;
    }
    else if ( RegReal_HARTCMD.u8HARTCMD_ForkBowrise == 0x01 ) //后仰
    {
        RC_Speed_Cnt.BowRiseRevCnt++;
        if ( RC_Speed_Cnt.BowRiseRevCnt > 50 ) //最大5秒，系数为5
            RC_Speed_Cnt.BowRiseRevCnt = 50;

        RCBowRisePos = ForkLiftAllPos.BowRisePos - _IQ ( 0.0169 ) * ( RC_Speed_Cnt.BowRiseRevCnt / 10 ); //ParamHandle_Control.Reg_RC_LiftPos
        ForkLiftAllPos.RowRiseState = 1;
//        m_sCurBowRiseCmd.u16SN = BowRiseSN++;

    }
    else
    {
        RC_Speed_Cnt.BowRiseForCnt = 10;
        RC_Speed_Cnt.BowRiseRevCnt = 10;

        if ( ForkLiftAllPos.RowRiseState == 1 )
        {
            ForkLiftAllPos.RowRiseState = 0;
            RCBowRisePos = ForkLiftAllPos.BowRisePos + _IQ ( 0.005 );//多前倾点
        }
        else if ( ForkLiftAllPos.RowRiseState == 2 )
        {
            RCBowRisePos = ForkLiftAllPos.BowRisePos - _IQ ( 0.005 );//多后仰点
            ForkLiftAllPos.RowRiseState = 0;
        }
    }
//旋转
    if ( RegReal_HARTCMD.u8HARTCMD_ForkRotate == 0xFF ) //回零
    {
        RC_Speed_Cnt.Rotate_0_Cnt++;
        if ( RC_Speed_Cnt.Rotate_0_Cnt > 50 ) //最大5秒，系数为5
            RC_Speed_Cnt.Rotate_0_Cnt = 50;

        RCRotatePos = ForkLiftAllPos.RotatePos + _IQ ( 0.0028 ) * ( RC_Speed_Cnt.Rotate_0_Cnt / 5 ); //ParamHandle_Control.Reg_RC_LiftPos
        ForkLiftAllPos.RotateState = 1;
        m_sCurRotateCmd.u16SN = RotateSN++;
    }
    else if ( RegReal_HARTCMD.u8HARTCMD_ForkRotate == 0x01 ) //去180°
    {
        RC_Speed_Cnt.Rotate_180_Cnt++;
        if ( RC_Speed_Cnt.Rotate_180_Cnt > 50 ) //最大5秒，系数为5
            RC_Speed_Cnt.Rotate_180_Cnt = 50;

        RCRotatePos = ForkLiftAllPos.RotatePos - _IQ ( 0.0028 ) * ( RC_Speed_Cnt.Rotate_180_Cnt / 5 ); //ParamHandle_Control.Reg_RC_LiftPos
        ForkLiftAllPos.RotateState = 2;
        m_sCurRotateCmd.u16SN = RotateSN++;
    }
    else
    {
        RC_Speed_Cnt.Rotate_0_Cnt = 10;
        RC_Speed_Cnt.Rotate_0_Cnt = 10;

        if ( ForkLiftAllPos.RotateState == 1 )
        {
            ForkLiftAllPos.RowRiseState = 0;
            RCRotatePos = ForkLiftAllPos.RotatePos;
        }
        else if ( ForkLiftAllPos.RowRiseState == 2 )
        {
            RCRotatePos = ForkLiftAllPos.RotatePos ;
            ForkLiftAllPos.RowRiseState = 0;
        }
    }
    
    //****************对夹具的位置指令限幅*************************//
    m_sCurSideShiftCmd.unSideShiftDegree =  int32DataSat ( RCSidePos, 65535, 0 );           //侧移中间变量
//    m_sCurLiftCmd.unLiftPos =  int32DataSat ( RCLiftPos, 65535, 0 );                        //升降
//    m_sCurClampCmd.unOpenDegree = int32DataSat ( RCClawPos, 65535, 0 );                     //开合
//    m_sCurBowRiseCmd.unBowRiseDegree = int32DataSat ( RCBowRisePos, 65535, 0 );             //俯仰 
//    m_sCurRotateCmd.unRotateDegree = int32DataSat ( RCRotatePos, 65535, 0 );                //旋转

//对下发结构体赋值并使能对应动作
    HostCom_ReadLiftCommand ();                                                             //升降传递值函数
//    HostCom_ReadClampCommand ();                                                            //开合
    HostCom_ReadBowRiseCommand ();                                                          //俯仰 
//    HostCom_ReadRotateCommand ();                                                           //旋转
    HostCom_ReadSideShiftCommand();                                                         //侧移

}

/*
* @brief       手操器控制VCU行进函数--1k
* @param       无
* @retval      无
*
*/
void Hart_DriveCmd ( void )
{
    static uint8_t u8LastDriveMode = 0;
    static uint16_t u16DelayTime = 0;

    if ( RegReal_HARTCMD.u8HARTCMD_DriveMode != u8LastDriveMode ) u16DelayTime = 0;//行进指令
    else
    {
        if ( u16DelayTime < 2000 ) u16DelayTime++;
    }

    if ( RegReal_HARTCMD.u8HARTCMD_DriveMode == 0xFF )  //行进指令--前进
    {
        RunState_RC = RunSta_Run_D;
        s16TargetSpeed = RCSpeed;//4000 - 0.66m/s
        m_eRunState = RunSta_Run_D;
        if ( u16DelayTime > 200 )//延时0.2s  ？
        {
            s16TargetSpeed = RCSpeed;
        }
        else
        {
            BreakPos = 0;//取消刹车
        }
    }
    else if ( RegReal_HARTCMD.u8HARTCMD_DriveMode == 0x01 ) //行进指令--后退
    {
        RunState_RC   = RunSta_Run_R;
        m_eRunState = RunSta_Run_R;
        s16TargetSpeed = RCSpeed;
        if ( u16DelayTime > 200 )
        {
            s16TargetSpeed = RCSpeed;
        }
        else
        {
            BreakPos = 0;
        }

    }
    else
    {
        m_eRunState = RunSta_Run_N;
        RunState_RC   = RunSta_Run_N;
        s16TargetSpeed = 0;
        if ( ( g_qCurWSpeed < 50 ) || ( u16DelayTime > 1500 ) )
        {
            BreakPos = BrakeDeep;//刹车
            BreakFlag = 1;
        }

    }
    u8LastDriveMode = RegReal_HARTCMD.u8HARTCMD_DriveMode;//行进指令
}
#endif
/*
* @brief       手操器模式转向控制函数
* @param       无
* @retval      无
*
*/
int16_t LastSteerPos, LastSteerPosCnt = 0, SteerPosCheakCnt = 0;
void Hart_SteerCmd ( void )
{
//    if ( SteerPosCheakCnt % 300 == 0 )
//    {
//        LastSteerPos =  m_sCurSteerCmd.nSteerPos;
//    }
//    m_sCurSteerCmd.nSteerPos = 16384 + ( 128 - RegReal_HARTCMD.u8HARTCMD_SteerAngle ) * 40;//手操器控制位置
//    m_sCurSteerCmd.u16SN = SteerSN++;
//    SteerPosCheakCnt++;
//    if ( ( LastSteerPos > ( m_sCurSteerCmd.nSteerPos + 120 ) ) || ( LastSteerPos < ( m_sCurSteerCmd.nSteerPos - 120 ) ) ) //遥控器指令300ms内有1.5°变化，开泵
//    {
//        RegReal_ChassisParam.u8ChassisEnabled.bit.SteeringEnabled = 1;//VCU转向使能
//        RegReal_SteerPosCtrlCmd.u16EN = ENABLE;

//        LastSteerPosCnt = 0;
//    }
//    else
//    {
//        if ( abs ( m_sCurSteerCmd.nSteerPos - g_u16SteeringWheelAbsPos ) < 320 ) //到达目标位置4°以内，1秒后关闭泵
//        {
//            LastSteerPosCnt++;
//            if ( LastSteerPosCnt > 1000 )
//            {
////                RegReal_SteerPosCtrlCmd.u16EN = DISABLE;
////                RegReal_ChassisParam.u8ChassisEnabled.bit.SteeringEnabled = 0;                   //油压禁能
//                LastSteerPosCnt = 1001;
//            }
//        }
//    }

    RegReal_ChassisParam.u8ChassisEnabled.bit.SteeringEnabled = 1;//VCU转向使能

}
/*
* @brief       饱和函数
* @param       无
* @retval      无
*
*/
int int32DataSat ( int data, int Max, int Min ) {
    if ( data > Max )
    {
        data = Max;
    }
    else if ( data < Min )
    {
        data = Min;
    }
    else
    {
        data = data;  // 保持原始值
    }
    return data;
}



