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

#define MessageID_SteerCmd          1       //ת��ָ��
#define MessageID_GasCmd            2       //����ָ��
#define MessageID_LiftCmd           3       //����ָ��
#define MessageID_HuggingCmd        4       //����ָ��
#define MessageID_AutoCharCmd       5       //�Զ����
#define MessageID_GlandCmd          6       //̧ѹָ��
#define MessageID_TelescopicCmd     7       //����ָ��
#define MessageID_RotateCmd         8       //��תָ��
#define MessageID_BrakeCmd          9       //����ɲ��������ͣ����
#define MessageID_RadarCmd          0x0A    //�����״￪��
#define MessageID_FaultClearCmd     0x0B    //�������
#define MessageID_BreakCanelCmd     0x0C    //����ɲ��
#define MessageID_ParamMngCmd       0x0D    //��������
#define MessageID_RaderLiftCmd      0x0E    //�״���������
#define MessageID_SlowSpdCmd        0x0F    //����Сλ�û��У��䶯ģʽ��
#define MessageID_ElectronMagCmd    0x10    //���������
#define MessageID_Reset             0x11    //���ϻָ�������
#define MessageID_EncoderCalaCmd    0x12    //���̰�װ��У׼����
#define MessageID_SteerCalaCmd      0x13    //ת��������0��궨����
#define MessageID_PumpEnableCmd     0x14    //�õ�����ƣ�Ԥ����
#define MessageID_HandAskCmd        0x15    //Э��汾����
#define MessageID_PowerSwCmd        0x16    //��Դ���ؿ���
#define MessageID_TelescopicMBDRelayCmd        0x17    //��Դ���ؿ���
#define MessageID_WatchDogCmd       0x63    //���Ź�

#define MessageID_BootLinkCmd       0x30    //��������ָ��ID
#define ProtocolVer_1               2
#define ProtocolVer_2               1
#define TurnDegThreshold            2000
//#define   PCDataToAgvWorkSpd(Data)    _IQdiv(Data, _IQ(7.084))  //(2m/s-4625.5rpm))
//#define   AgvWorkSpdToPCData(Data)    _IQdiv(Data, _IQ(0.14116121))  //(2m/s-4625.5rpm))   
#define   PCDataToAgvWorkSpd(Data)    _IQdiv(Data, _IQ(9.480))//(2m/s-4625.5rpm))
#define   AgvWorkSpdToPCData(Data)    _IQrmpy(Data, _IQ(9.480)) //(2m/s-4625.5rpm))   
#define AGVReadErrMaxCnt        1000
// Ԥ���� 1500/1024 �� Q15 ֵ
#define CONSTANT_1500_DIV_1024 _IQ(1500.0/1024.0)  // �� 1.46484375
#define LiftPosBias     2600
//---------------------------------------------------------------------------------------
//-------------------- public data ------------------------------------------
/*textile*/
typedef enum {
    SlaveBoard = 1,
    RotatingBoard,    /*��ת���*/
    TelescopicBoard, /*�������*/
    HuggingBoard,   /*�������*/
    GlandBoard      /*̧ѹ���*/
} BoardIdDef;
typedef struct  //1
{
    uint16_t u16SN;                        //������ţ�ack��Ӧʹ��
    uint16_t u16CtrlMode;                    //����ģʽ
    int16_t  s16Pos;                    //steerλ�ã�����λ��
} SteerCommand;

typedef struct //2
{
    uint16_t u16SN;                       //������ţ�ack��Ӧʹ��
    int16_t  s16Spd;                   //���ӽ��ٶȣ������ٶ�
} GasCommand;

typedef struct //3
{
    uint16_t u16SN;                    //������ţ�ack��Ӧʹ��
    uint16_t CtrlMode;                    //����ģʽ
    uint16_t unLiftPos;                  //Ŀ��Lift�߶� 32767 = 6m
    uint16_t LiftMode;                   //����ģʽ��0��λ���ŷ���1���½����ף�����λ�ô���Ŀ��λ��15cm����2����������Ҫ���ȣ�����Ҫ�ﵽ��Ӧ�߶ȣ����Զ಻�����٣�
} LiftCommand;

typedef struct//4
{
    uint16_t u16SN;                     //������ţ�ack��Ӧʹ��
    uint16_t u16CtrlMode;                    //����ģʽ
    uint16_t u16OpenDegree;                //���ȣ�32767=1m
} HuggingCommand;

typedef struct //5
{
    uint16_t u16SN;                        //������ţ�ack��Ӧʹ��
    uint16_t u16SW;                        //����
    uint16_t res;                      //
} AutoCharCommand;

typedef struct//6
{
    uint16_t u16SN;                    //������ţ�ack��Ӧʹ��
    uint16_t u16CtrlMode;                    //����ģʽ
    uint16_t u16Pos;                  //Ŀ��Lift�߶� 32767 = 6m
    uint16_t res;                   //����ģʽ��0��λ���ŷ���1���½����ף�����λ�ô���Ŀ��λ��15cm����2����������Ҫ���ȣ�����Ҫ�ﵽ��Ӧ�߶ȣ����Զ಻�����٣�
} GlandCommand;

typedef struct//7
{
    uint16_t u16SN;                    //������ţ�ack��Ӧʹ��
    uint16_t u16CtrlMode;                    //����ģʽ
    uint16_t u16Pos;            //���ȣ�32767=1m
} TelescopicCommand;

typedef struct//8
{
    uint16_t u16SN;                  //������ţ�ack��Ӧʹ��
    uint16_t u16CmdMode;              //��SolenoidPos_ ��
    uint16_t u16RotateDegree;           //���ȣ�32767=1m
} RotateCommand;

typedef struct //9
{
    uint16_t u16SN;                    //������ţ�ack��Ӧʹ��
    int16_t  CarWheelPos;               //��SolenoidPos_ ��
} WheelPosMode;

typedef struct {
    uint16_t u16SN;                        //������ţ�ack��Ӧʹ��
    RunStates unDriverMode;                  //��RunStates ��
} SetModeCommand;
typedef struct {
    uint16_t unSyncWord;    //0xfee1
    uint16_t unMsgID;
    uint16_t unTimeStamp;    //��λ1ms
    uint16_t unDataLen;      //��λbyte

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
    uint16_t unSteerAck;//ת��Ack
    uint16_t unGasAck;//�н�Ack
    uint16_t unCurLiftAck;//����ack
    uint16_t unHuggingAck;//����ack
    uint16_t unAutoCharAck;//�Զ����
    uint16_t unGlandAck;//̧ѹack
    uint16_t unTelescopicAck;//����ack
    uint16_t unRotatingAck;//��תack
    uint16_t unBrakeAck;//ɲ��
    uint16_t unBrakeCanelAck;//����ɲ��
    uint16_t unSetModeAck;
    uint16_t unTelescopicRelayAck;
    uint16_t unPowerSwAck;
    uint16_t unRaderAck;
    uint16_t unFaultClearAck;
    uint16_t unResetAck;//��λ
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
} CommandAck;                               //����ת����λ��������SN

typedef struct //10
{
    uint16_t u16SN;                        //������ţ�ack��Ӧʹ��
    int16_t  Radar_Switch;                    //�����ź�
} RadarCommand;                             //�����״￪���ź�

typedef struct//0x0B
{
    uint16_t u16SN;                        //������ţ�ack��Ӧʹ��
    uint16_t u16BoardID;                   //����ź����� 1��λ�� 2Һѹ�� 3ת��� 4ȫ��
    uint16_t u16Mod;
} FaultClear;                               //�������ָ��

typedef struct //0x0C
{
    uint16_t u16SN;                        //������ţ�ack��Ӧʹ��
    uint16_t BreakTime;                     //ɲ��ʱ��
} BreakCanel;                               //����ɲ��ָ��

typedef struct 
{
    uint16_t u16SN;                       //������ţ�ack��Ӧʹ��
    uint16_t Weight;                        //ֽ������(KG)
    uint16_t CtrlMode;                      //����ģʽ
    int16_t  unSideShiftDegree;             //��SolenoidPos_ ��
} SideShiftCommand;

typedef struct {
    uint16_t u16SN;                        //������ţ�ack��Ӧʹ��
    uint16_t unCRes;                        //������ţ�ack��Ӧʹ��
    uint16_t  RadLiftPos;                    //�״�����λ��
} RadLiftCmd;                               //�״�����ָ��

typedef struct //0x0F
{
    uint16_t u16SN;                    //������ţ�ack��Ӧʹ��
    uint16_t PosDir;                    //����
    uint16_t CarWheelPos;                //����
} Slow_Spd_Mode;                          //����ģʽ���䶯��

typedef struct //0x10
{
    uint16_t u16CmdSN;                        //������ţ�ack��Ӧʹ��
    uint16_t u16SW;                        //����
    uint16_t res;                      //
} ElectronMagCommand;

typedef struct {
    uint16_t u16SN;                    //������ţ�ack��Ӧʹ��
    uint16_t Ctrl_ID;                    //���ƶ���
    uint16_t Switch;                     //����
} Sound_Lignt_Ctrl;                      //�������

typedef struct {
    uint16_t u16SN;                    //������ţ�ack��Ӧʹ��
    uint16_t Board_ID;                   //�忨ID
    uint32_t Fault_Num;                  //������ϸ��
} Fault_Disable;                         //��������

typedef struct {
    uint16_t u16SN;                    //������ţ�ack��Ӧʹ��
    uint16_t Res;                       //Ԥ��
    uint16_t ID;                        //У׼ID
} EncoderCala;                         //����У׼

typedef struct {
    uint16_t u16SN;                    //������ţ�ack��Ӧʹ��
    uint16_t Switch;                     //����
} Steer_EncoderCala;                     //ת�����̱궨

typedef struct {
    uint16_t u16SN;                    //������ţ�ack��Ӧʹ��
    uint16_t ID;                        //���ƶ���
    uint16_t Switch;                   //����
} PumpEnable;                         //�õ��ʹ�ܿ���

typedef struct {
    uint16_t u16SN;                    //������ţ�ack��Ӧʹ��
    uint16_t Version_1;                 //��汾
    uint16_t Version_2;                 //С�汾
} HandAsk;                              //Э������
typedef struct//0x11
{
    uint16_t u16SN;                    //������ţ�ack��Ӧʹ��
    uint16_t u16BoardID;                   //�忨ID
    uint16_t u16Delay;                   //��ʱ��λ����λ10mS
} RegDef_Reset;
extern RegDef_Reset m_Reset;

typedef struct//0x63
{
    uint16_t WDCnt;
    uint16_t res;
} RegDef_WatchDogCmd;

typedef struct//0x13
{
    uint16_t u16SN;                    //������ţ�ack��Ӧʹ��
    uint16_t u16Cmd;
    uint16_t u16BoardID;                   //�忨ID
} RegDef_MotorCala;
extern RegDef_MotorCala m_MotorCala;

typedef struct//0x0D
{
    uint16_t u16SN;                    //������ţ�ack��Ӧʹ��
    uint16_t u16Cmd;
    uint16_t u16BoardID;                   //�忨ID
    uint16_t u16Addr;                     //��ַ
    uint16_t u16Value;                   //�Ĵ���ֵ
} RegDef_ParamMngCmd;
extern RegDef_ParamMngCmdAck m_ParamMngCmdAck;
typedef struct//0x30
{
    uint16_t u16Cmd;
    uint16_t u16BoardID;                   //�忨ID
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
    uint16_t u16SN;                    //������ţ�ack��Ӧʹ��
    PowerSw_REG PowerSwOpt;             //��Դ����ѡ��
    uint16_t u16Delay;                   //��ʱ����λ10mS
} RegDef_PowerSwCmd;
typedef struct//0x17
{
    uint16_t u16SN;                    //������ţ�ack��Ӧʹ��
    uint16_t u16Cmd;
    uint16_t u16Delay;
}HostCom_TelescopicMBDRelayCmd;
extern RegDef_PowerSwCmd m_PowerSwCmd;
//-------------------- public functions -------------------------------------
//��ȡ����ָ���0Ϊ��ָ��
int8_t HostCom_ReadBowRiseCommand ( void );
//��ȡ��תָ���0Ϊ��ָ��
int8_t HostCom_ReadRotateCommand ( void );
//��ȡ��תָ���0Ϊ��ָ��
int8_t HostCom_ReadSideShiftCommand ( void );
//��ȡɲ��ָ���0Ϊ��ָ��
int8_t HostCom_ReadBrakeCommand ( int16_t* pnSolenoidPos, uint16_t* punRotateDegree );

//��ȡ����ָ���0Ϊ��ָ��
int8_t HostCom_ReadClampCommand ( void );
//��ȡ����ָ���0Ϊ��ָ��
int8_t HostCom_ReadLiftCommand ( void );
//��ȡת��ָ���0Ϊ��ָ��
int8_t HostCom_ReadSteerCommand ( int32_t* pnSteerPos );
//��ȡ�ٶ�ָ���0Ϊ��ָ��
int8_t HostCom_ReadGasCommand ( int16_t* pnWheelSpeed );
//��ȡ��ʻģʽ����0Ϊ��ָ��
int8_t HostCom_ReadDrvModeCommand ( int16_t* pnDrvMode );
//��ʼ��
int8_t HostCom_Init();
//�����������
int8_t HostCom_ProcRecvData();
//�����ϱ�����
int8_t HostCom_SendReport();
//��ȡ�����״�����
int8_t  HostCom_RadarCommand ( uint16_t* Radar_Switch ) ;

//�״���������
int8_t HostCom_RadLiftCommand ( void );
int8_t HostCom_SlowSpdCommand ( uint16_t*Dir, uint16_t* Pos );      //���ٻ���λ���ŷ�
int8_t Sound_LigntCommand ( void );     //�������
int8_t HostCom_Fault_DisableCommand ( void );     //���Ͻ��ܣ����Σ�
int8_t EncoderCalaCommand ( void );     //����λ�ñ궨��ʼ��
int8_t SteerCalaCommand ( void );     //ת�����̱궨
int8_t PumpEnableCommand ( void );     //�õ��ʹ��
int8_t HandAskCommand ( void );     //Э��汾����
int8_t HostCom_AutoCharCommand ( void );



void HostCom_Test();

extern int8_t HostCom_ResetCommand ( void );
//void Fault_SendReport() ;         //�ϱ�������Ϣ
void SolenoidInit ( void ) ;      //�Ƹ�ָ���ʼ������ֹ���ϵ磬�Ƹ��˶�
void Hart_DriveCmd ( void )  ; //��ȡң�����н�ָ��
int8_t  BreakCanelFun ( uint8_t* Break_Switch, uint16_t* Break_Time );     //����ɲ��
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
extern FaultClear     m_sFaultClear;                               //�������
extern uint16_t AGVReadErrCnt;
extern _iq iqVCUTurnDegFB;
extern void HostCom_ClearCommand();
extern ElectronMagCommand  m_sElectronMagCmd;                  //���������
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
