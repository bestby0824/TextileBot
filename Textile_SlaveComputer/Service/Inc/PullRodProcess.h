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
    _iq Pos_Q_1m;   //Q��ʽ���ȣ�IQ(1)����1m������(1/65535)m
    _iq Pos_Q_1m_Last;
    _iq Pos_Src;    //������ԭʼ���� ��λ΢��
    _iq Spd_Q;      //Q��ʽ�ٶȣ�_IQ(1)����1m/s
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
    uint16_t unSN; //������ţ�ack��Ӧʹ��
    int16_t EN; //��λ������ͣ
    uint16_t unPos; //Ŀ��Lift�߶� 32767 = 6m
    uint16_t CtrlMode; //����ģʽ
} Cmd;

typedef struct            //EndOfCtrl�жϽṹ��
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
    SolenoidMask CmdSet;    //���յ������д�����ͬʱ��1�����յ��Ƹ���ӦACK����0���������������Ͷ�������ģʽ
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
    uint16_t u16StaFlg;                            //bit0:��λ��1
    uint16_t LiftMode;                             //����ģʽ��0��λ���ŷ���1���½�����
    uint16_t u16CRC;
} SolenoidCommand;

typedef struct {
    uint16_t u16SyncWord;   //HEAD
    uint8_t u8ID, u8Fun;
    uint16_t unBrakePercentage;
    uint16_t unClampHight;                  //���е�ǰ�߶�
    uint16_t unClampOpenDegree;             //���е�ǰ����
    uint16_t unBowRiseDegree;               //�����Ƕ�
    uint16_t unRotateDegree;                //��ת�Ƕ�
    uint16_t unSideShiftDegree;             //����λ��
    uint16_t unLiftRaderDegree;             //��������λ��

    uint16_t unLift_Pressure;               //������ѹ
    uint16_t unClaw_Pressure;               //������ѹ
    uint16_t unSideShift_Pressure1;         //������ѹ
    uint16_t unSideShift_Pressure2;         //������ѹ

    uint16_t unLiftAck;                     //��������SN
    uint16_t unClampAck;                    //��������SN
    uint16_t unBowRiseAck;                  //��������SN
    uint16_t unRotateAck;                   //��ת����SN
    uint16_t unSideShiftAck;                //��������SN
    uint16_t unLiftRaderAck;                //������������SN

    uint16_t SolenoidEndOfCtrl;             //EndOfCtrl�� bit/0~3/4~7/8~11/12~15,�ֱ�������������У���������ת
    uint16_t SolenoidEndOfCtrl2;            //���ƻ���EndOfCtrl��
    uint16_t PosStopFlag;                   //����ͣ����־
    uint32_t u32FaultNum;                   //���ϱ���
    uint16_t g_u16WarningNum;               //�������
    uint16_t u16StaFlg;                     //bit0:��ʼ��ʱ״̬Ϊ1��5�����Ϊ3������⵽һ�δ�1��3�ı仯����������һ��
    uint16_t ClawState;                     //�����Ƿ��ֽ��״̬
    uint16_t  SideShiftState;
    uint16_t  Res1;
    uint16_t u16CRC;
} SolenoidReport_Pack;


#define Solenoid_Opt_Cmd          1   //λ�ø���
#define Solenoid_Opt_Report       2   //�Զ��ϴ�,100Hz
#define Solenoid_Opt_Read         3   //��ȡ״̬
#define Solenoid_Opt_SetSta       4   //����״̬
#define Solenoid_Opt_Param_Solenoid       5   //�ϴ��Ƹ˲���
#define Solenoid_Opt_Param_Hub485Fork    6   //�ϴ�Hub485����
#define Solenoid_Opt_Param_Hub485Body    7   //�ϴ�Hub485����

#define Solenoid_Reg_EN          0x00    //1-ʹ�ܣ�0-��ʹ��


extern Solenoid_Info g_sSolenoid_Info1[RodID_Num];
extern SolenoidCommand SolenoidCtrlCmd;

extern SolenoidReport_Pack SolenoidReport;
extern uint16_t ClawState;                 //���мн�״̬1�н���0δ�н�
extern Cmddown SolenoidCmddown;          //Һѹ���˶�EndOfCtrl����;

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
