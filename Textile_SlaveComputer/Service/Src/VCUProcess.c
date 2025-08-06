/**
* ��Ȩ����(C)
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
* <����> | <�汾> | <����> | <����>
*
* xxxx/xx/xx | 1.0.0 | HWW | �����ļ�
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
RegDef_ChassisParam RegReal_ChassisParam;  //�Ĵ���0x201
RegDef_ForkParam RegReal_ForkParam;  //�Ĵ���0x301
RegDef_PartsAndValve1Param RegReal_PartsAndValve1Param;  //�Ĵ���0x401
RegDef_PartsAndValve2Param RegReal_PartsAndValve2Param;  //�Ĵ���0x501
RegDef_ForkLiftState1 RegReal_ForkLiftState1;  //�Ĵ���0x181
RegDef_ForkLiftState2 RegReal_ForkLiftState2;  //�Ĵ���0x281
RegDef_ForkLiftState3 RegReal_ForkLiftState3;  //�Ĵ���0x381
RegDef_ForkLiftState4 RegReal_ForkLiftState4;  //�Ĵ���0x481
RegDef_ForkLiftState5 RegReal_ForkLiftState5;  //�Ĵ���0x182
RegDef_ForkLiftState6 RegReal_ForkLiftState6;  //�Ĵ���0x19E
RegDef_HARTCMD RegReal_HARTCMD;                //�Ĵ���0x1A3���ֲ���ָ��
VCU_ParamStruct VCU_ParamHandle;               //VCU���ݽ����ṹ��
VCU_ParamStruct VCU_ParamHandle;  //VCU���ݽ����ṹ��
CanTransferTypeDef CanTransferStruct;          //CAN���丨���ֶ�
ForkLiftPos ForkLiftAllPos;                     //����Һѹ�巴����λ����Ϣ
RCCnt RC_Speed_Cnt;                             //ң���˶��ٶȿ���
int16_t VCULife = -1;                       //VCU��������

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
 * @brief       VCU��Ӧ�Ĵ���״̬����static
 * @param       �Ĵ���ID
 * @retval      ��
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
 * @brief       ����CAN1�����ݲ�������䵽VCU�ṹ��
 * @param       ��
 * @retval      ��
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
        __disable_irq();         //�ر����ж�
        memcpy ( ( uint8_t * ) &Reg_AGVBMSStaOrigDataFb, Rx_Buffer, 8 );
        __enable_irq();          // �������ж�
    }
    break;

    case Step_AGVDataFb1:
    {
        __disable_irq();         //�ر����ж�
        memcpy ( ( uint8_t * ) &Reg_AGVDataFb1, Rx_Buffer, 8 );
        __enable_irq();          // �������ж�
    }
    break;

    case Step_AGVDataFb2:
    {
        __disable_irq();         //�ر����ж�
        memcpy ( ( uint8_t * ) &Reg_AGVDataFb2, Rx_Buffer, 8 );
        __enable_irq();          // �������ж�
    }
    break;

    case Step_AGVPumpCtrFb:
    {
        __disable_irq();         //�ر����ж�
        memcpy ( ( uint8_t * ) &Reg_AGVPumpCtrFb, Rx_Buffer, 8 );
        __enable_irq();          // �������ж�
    }
    break;
    
    case Step_AGVTurnFb:
    {
        __disable_irq();         //�ر����ж�
        memcpy ( ( uint8_t * ) &Reg_AGVTurnFb, Rx_Buffer, 8 );
        __enable_irq();          // �������ж�
    }
    break;
    case Step_AGVChargeSW:
    {
          __disable_irq();         //�ر����ж�
        memcpy ( ( uint8_t * ) &Reg_AGVChargeSWFb, Rx_Buffer, 8 );
        __enable_irq();          // �������ж�
     }
     break;
    case Step_RCCtrlCmdFb:
    {
        __disable_irq();         //�ر����ж�
        memcpy ( ( uint8_t * ) &Reg_RCCtrlCmdFb, Rx_Buffer, 8 );
        __enable_irq(); 
       
    }
     break;
    default:
        break;
    }
}
/**
 * @brief       VCU��Ӧ�Ĵ���״̬���ͣ�20HZ����
 * @param       ��
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

//**************************************ң�������ƽ��մ����߼�*************************************************//
uint16_t LiftSN, ClawSN, BowRiseSN, RotateSN, SideSN, SteerSN; //�Լ�SN
int RCLiftPos, RCClawPos, RCBowRisePos, RCRotatePos, RCSidePos; //λ��ָ���м�ֵ
#if 0
/*
* @brief       �ֲ�������
* @param       ��
* @retval      ��
*
*/
void Hart_SolenoidCmdInit ( void )    //ң��Һѹ������ָ���ʼ��
{
    RegReal_HARTCMD.u8HARTCMD_SteerAngle = 0;//�Ƕ�ָ�����
}
/*
* @brief       �ֲ�������Һѹ�����庯�� 10hz
* @param       ��
* @retval      ��
*
*/

void Hart_SolenoidCmd ( void )    //ң��Һѹ������ָ��
{
    ForkLiftAllPos.LiftPos = g_sSolenoid_Info1[RodID_Lift].Pos;         //����
    ForkLiftAllPos.ClawPos = g_sSolenoid_Info1[RodID_Claw].Pos;         //����
    ForkLiftAllPos.BowRisePos = g_sSolenoid_Info1[RodID_BowRise].Pos;   //����
    ForkLiftAllPos.RotatePos = g_sSolenoid_Info1[RodID_Rotate].Pos;     //��ת
    ForkLiftAllPos.SidePos = g_sSolenoid_Info1[RodID_SideShift].Pos;    //����
//����
    if ( RegReal_HARTCMD.u8HARTCMD_Forkside == 0xFF ) //��������
    {
        RC_Speed_Cnt.SideMoveRightCnt++;
        if ( RC_Speed_Cnt.SideMoveRightCnt > 50 ) //���5�룬ϵ��Ϊ5
            RC_Speed_Cnt.SideMoveRightCnt = 50;

        RCSidePos = ForkLiftAllPos.SidePos - _IQ ( 0.01 ) * ( RC_Speed_Cnt.SideMoveRightCnt / 10 ); //ParamHandle_Control.Reg_RC_LiftPos
        m_sCurSideShiftCmd.u16SN = SideSN++;
        ForkLiftAllPos.SideState = 1;//
    }
    else if ( RegReal_HARTCMD.u8HARTCMD_Forkside == 0x01 ) //��������
    {
        RC_Speed_Cnt.SideMoveLeftCnt++;
        if ( RC_Speed_Cnt.SideMoveLeftCnt > 50 ) //���5�룬ϵ��Ϊ5
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
            RCSidePos = ForkLiftAllPos.SidePos + _IQ ( 0.01 );//���ƽ��������
        }
        else if ( ForkLiftAllPos.SideState == 2 )
        {
            RCSidePos = ForkLiftAllPos.SidePos - _IQ ( 0.01 );//���ƽ��������
            ForkLiftAllPos.SideState = 0;
        }
    }
//����    
    if ( RegReal_HARTCMD.u8HARTCMD_ForkLift == 0xFF ) //����
    {
        RC_Speed_Cnt.LiftUpCnt++;
        if ( RC_Speed_Cnt.LiftUpCnt > 50 ) //���5�룬ϵ��Ϊ5
            RC_Speed_Cnt.LiftUpCnt = 50;

        RCLiftPos = ForkLiftAllPos.LiftPos + _IQ ( 0.01 ) * ( RC_Speed_Cnt.LiftUpCnt / 10 ); //ParamHandle_Control.Reg_RC_LiftPos
//        m_sCurLiftCmd.u16SN = LiftSN++;
        ForkLiftAllPos.LiftState = 1;
    }
    else if ( RegReal_HARTCMD.u8HARTCMD_ForkLift == 0x01 ) //�½�
    {
        RC_Speed_Cnt.LiftDownCnt++;
        if ( RC_Speed_Cnt.LiftDownCnt > 50 ) //���5�룬ϵ��Ϊ5
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
            RCLiftPos = ForkLiftAllPos.LiftPos + _IQ ( 0.01 );//������������ߵ�
        }
        else if ( ForkLiftAllPos.LiftState == 2 )
        {
            RCLiftPos = ForkLiftAllPos.LiftPos - _IQ ( 0.01 );//�½���������ߵ�
            ForkLiftAllPos.LiftState = 0;
        }
    }
//����
    if ( RegReal_HARTCMD.u8HARTCMD_ForkClaw == 0xFF ) //��
    {
        RC_Speed_Cnt.ClawClosCnt++;
        if ( RC_Speed_Cnt.ClawClosCnt > 50 ) //���5�룬ϵ��Ϊ5
            RC_Speed_Cnt.ClawClosCnt = 50;

        RCClawPos = ForkLiftAllPos.ClawPos + _IQ ( 0.01 ) * ( RC_Speed_Cnt.ClawClosCnt / 10 ); //ParamHandle_Control.Reg_RC_LiftPos
//        m_sCurClampCmd.u16SN = ClawSN++;
        ForkLiftAllPos.ClawState = 1;
    }
    else if ( RegReal_HARTCMD.u8HARTCMD_ForkClaw == 0x01 ) //��
    {
        RCClawPos = ForkLiftAllPos.ClawPos - _IQ ( 0.01 ); //ParamHandle_Control.Reg_RC_ClawPos
        RC_Speed_Cnt.ClawOpenCnt++;
        if ( RC_Speed_Cnt.ClawOpenCnt > 50 ) //���5�룬ϵ��Ϊ5
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
            RCClawPos = ForkLiftAllPos.ClawPos + _IQ ( 0.005 );//�����������ٱ�����
        }
        else if ( ForkLiftAllPos.ClawState == 2 )
        {
            RCClawPos = ForkLiftAllPos.ClawPos - _IQ ( 0.005 );//�򿪽������ٶ��Щ
            ForkLiftAllPos.ClawState = 0;
        }
    }
//����    
    if ( RegReal_HARTCMD.u8HARTCMD_ForkBowrise == 0xFF ) //ǰ��
    {
        RC_Speed_Cnt.BowRiseForCnt++;
        if ( RC_Speed_Cnt.BowRiseForCnt > 50 ) //���5�룬ϵ��Ϊ5
            RC_Speed_Cnt.BowRiseForCnt = 50;

        RCBowRisePos = ForkLiftAllPos.BowRisePos +  _IQ ( 0.0169 ) * ( RC_Speed_Cnt.BowRiseForCnt / 10 ); //ParamHandle_Control.Reg_RC_LiftPos
        ForkLiftAllPos.RowRiseState = 1;
//        m_sCurBowRiseCmd.u16SN = BowRiseSN++;
    }
    else if ( RegReal_HARTCMD.u8HARTCMD_ForkBowrise == 0x01 ) //����
    {
        RC_Speed_Cnt.BowRiseRevCnt++;
        if ( RC_Speed_Cnt.BowRiseRevCnt > 50 ) //���5�룬ϵ��Ϊ5
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
            RCBowRisePos = ForkLiftAllPos.BowRisePos + _IQ ( 0.005 );//��ǰ���
        }
        else if ( ForkLiftAllPos.RowRiseState == 2 )
        {
            RCBowRisePos = ForkLiftAllPos.BowRisePos - _IQ ( 0.005 );//�������
            ForkLiftAllPos.RowRiseState = 0;
        }
    }
//��ת
    if ( RegReal_HARTCMD.u8HARTCMD_ForkRotate == 0xFF ) //����
    {
        RC_Speed_Cnt.Rotate_0_Cnt++;
        if ( RC_Speed_Cnt.Rotate_0_Cnt > 50 ) //���5�룬ϵ��Ϊ5
            RC_Speed_Cnt.Rotate_0_Cnt = 50;

        RCRotatePos = ForkLiftAllPos.RotatePos + _IQ ( 0.0028 ) * ( RC_Speed_Cnt.Rotate_0_Cnt / 5 ); //ParamHandle_Control.Reg_RC_LiftPos
        ForkLiftAllPos.RotateState = 1;
        m_sCurRotateCmd.u16SN = RotateSN++;
    }
    else if ( RegReal_HARTCMD.u8HARTCMD_ForkRotate == 0x01 ) //ȥ180��
    {
        RC_Speed_Cnt.Rotate_180_Cnt++;
        if ( RC_Speed_Cnt.Rotate_180_Cnt > 50 ) //���5�룬ϵ��Ϊ5
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
    
    //****************�Լоߵ�λ��ָ���޷�*************************//
    m_sCurSideShiftCmd.unSideShiftDegree =  int32DataSat ( RCSidePos, 65535, 0 );           //�����м����
//    m_sCurLiftCmd.unLiftPos =  int32DataSat ( RCLiftPos, 65535, 0 );                        //����
//    m_sCurClampCmd.unOpenDegree = int32DataSat ( RCClawPos, 65535, 0 );                     //����
//    m_sCurBowRiseCmd.unBowRiseDegree = int32DataSat ( RCBowRisePos, 65535, 0 );             //���� 
//    m_sCurRotateCmd.unRotateDegree = int32DataSat ( RCRotatePos, 65535, 0 );                //��ת

//���·��ṹ�帳ֵ��ʹ�ܶ�Ӧ����
    HostCom_ReadLiftCommand ();                                                             //��������ֵ����
//    HostCom_ReadClampCommand ();                                                            //����
    HostCom_ReadBowRiseCommand ();                                                          //���� 
//    HostCom_ReadRotateCommand ();                                                           //��ת
    HostCom_ReadSideShiftCommand();                                                         //����

}

/*
* @brief       �ֲ�������VCU�н�����--1k
* @param       ��
* @retval      ��
*
*/
void Hart_DriveCmd ( void )
{
    static uint8_t u8LastDriveMode = 0;
    static uint16_t u16DelayTime = 0;

    if ( RegReal_HARTCMD.u8HARTCMD_DriveMode != u8LastDriveMode ) u16DelayTime = 0;//�н�ָ��
    else
    {
        if ( u16DelayTime < 2000 ) u16DelayTime++;
    }

    if ( RegReal_HARTCMD.u8HARTCMD_DriveMode == 0xFF )  //�н�ָ��--ǰ��
    {
        RunState_RC = RunSta_Run_D;
        s16TargetSpeed = RCSpeed;//4000 - 0.66m/s
        m_eRunState = RunSta_Run_D;
        if ( u16DelayTime > 200 )//��ʱ0.2s  ��
        {
            s16TargetSpeed = RCSpeed;
        }
        else
        {
            BreakPos = 0;//ȡ��ɲ��
        }
    }
    else if ( RegReal_HARTCMD.u8HARTCMD_DriveMode == 0x01 ) //�н�ָ��--����
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
            BreakPos = BrakeDeep;//ɲ��
            BreakFlag = 1;
        }

    }
    u8LastDriveMode = RegReal_HARTCMD.u8HARTCMD_DriveMode;//�н�ָ��
}
#endif
/*
* @brief       �ֲ���ģʽת����ƺ���
* @param       ��
* @retval      ��
*
*/
int16_t LastSteerPos, LastSteerPosCnt = 0, SteerPosCheakCnt = 0;
void Hart_SteerCmd ( void )
{
//    if ( SteerPosCheakCnt % 300 == 0 )
//    {
//        LastSteerPos =  m_sCurSteerCmd.nSteerPos;
//    }
//    m_sCurSteerCmd.nSteerPos = 16384 + ( 128 - RegReal_HARTCMD.u8HARTCMD_SteerAngle ) * 40;//�ֲ�������λ��
//    m_sCurSteerCmd.u16SN = SteerSN++;
//    SteerPosCheakCnt++;
//    if ( ( LastSteerPos > ( m_sCurSteerCmd.nSteerPos + 120 ) ) || ( LastSteerPos < ( m_sCurSteerCmd.nSteerPos - 120 ) ) ) //ң����ָ��300ms����1.5��仯������
//    {
//        RegReal_ChassisParam.u8ChassisEnabled.bit.SteeringEnabled = 1;//VCUת��ʹ��
//        RegReal_SteerPosCtrlCmd.u16EN = ENABLE;

//        LastSteerPosCnt = 0;
//    }
//    else
//    {
//        if ( abs ( m_sCurSteerCmd.nSteerPos - g_u16SteeringWheelAbsPos ) < 320 ) //����Ŀ��λ��4�����ڣ�1���رձ�
//        {
//            LastSteerPosCnt++;
//            if ( LastSteerPosCnt > 1000 )
//            {
////                RegReal_SteerPosCtrlCmd.u16EN = DISABLE;
////                RegReal_ChassisParam.u8ChassisEnabled.bit.SteeringEnabled = 0;                   //��ѹ����
//                LastSteerPosCnt = 1001;
//            }
//        }
//    }

    RegReal_ChassisParam.u8ChassisEnabled.bit.SteeringEnabled = 1;//VCUת��ʹ��

}
/*
* @brief       ���ͺ���
* @param       ��
* @retval      ��
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
        data = data;  // ����ԭʼֵ
    }
    return data;
}



