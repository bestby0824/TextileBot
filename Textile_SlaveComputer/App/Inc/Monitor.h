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
#ifndef _Monitor__H_
#define _Monitor__H_
//-------------------- include files ----------------------------------------
#include "Driver.h"
#include "Oled_com.h"
#include "AngleMath.h"
//#include "SteeringWheelProcess.h"
#include "NodeProcess.h"

//-------------------- public definitions -----------------------------------
#define R1                  (30000)                                             // ��ѹ����R1��30k��
#define R2                  (2200)                                              // ��ѹ����R2��2.2k��
#define ADC_MAX             (40950)                                              // ADC���ֵ��12λADC
#define REF_VOLTAGE         (33)                                                 // �ο���ѹ��3.3V
#define VREF_PER_ADC        _IQdiv(REF_VOLTAGE, ADC_MAX)                        // �ο���ѹ��ADC���ֵ�ı�ֵ
#define RESISTOR_RATIO      _IQdiv((R1 + R2), R2)                               // ��ѹ�ȣ�R2 / (R1 + R2)
#define POWERCURR_RATIO     _IQ ( -10 )                                          //��Դ����ϵ��10A/V
#define NoFault                            0
#define Fault                              1
#define HighTempValue                      7000
//struct Error_BITS_Steer              //ת�������������
//{
//    uint32_t    uE2promErr                     : 1; //E2p����
//    uint32_t    uDBVersion                     : 1; //���ݿ�汾��һ��
//    uint32_t    uLackUVW                       : 1; //ȱ��
//    uint32_t    uReangleLack                   : 1; //��������������
//    
//    uint32_t    uCoderTama1_Lack               : 1; //����1�ű���������
//    uint32_t    uCoderTama2_Lack               : 1; //����1�ű���������
//    uint32_t    uCmdDogLose                    : 1; //������ʱ
//    uint32_t    uSpdCircle                     : 1; //�ٶȸ������ز���
//    
//    uint32_t    uLodeOver                      : 1; //����
//    uint32_t    uTempOver                      : 1; //����
//    uint32_t    uVolOver                       : 1; //��ѹ
//    uint32_t    uBoardUpdata                   : 1; //���ڰ忨����/�����޸�
//    uint32_t    NO_AKM                          : 1; //�����ϰ�����ģ��

//    uint32_t    rsvd                           : 10;
//};
//typedef union
//{
//    uint32_t    all;
//    struct      Error_BITS_Steer   bit;
//} Fault_REG_Steer;

//struct Error_BITS_Solenoid              //Һѹ�����������
//{
//    uint32_t    LiftSolenoidInitErr                     : 1; //��ŷ��Լ����           
//    uint32_t    LiftSensorLinkLose                      : 1; //����������ʧ��          
//    uint32_t    LiftSensorDataError                     : 1; //�������ߴ����������쳣�������������ݲ�һ�£���Ҫע�������������׼�⣩
//    uint32_t    LiftMotorPosLinkLose                    : 1; //������ʧЧ                
//    uint32_t    LiftMoveTimeOut                         : 1; //�˶���ʱ                     
//    uint32_t    LiftSolenoidRun_Err                     : 1; //�����������ʵ���˶�����       
//    
//    uint32_t    ClawSolenoidInitErr                     : 1; //��ŷ��Լ����              
//    uint32_t    ClawSensorLinkLose                      : 1; //������ʧ��                  
//    uint32_t    ClawSensorDataError                     : 1; //���д����������쳣�����ޣ�  
//    uint32_t    ClawMotorPosLinkLose                    : 1; //������ʧЧ                
//    uint32_t    ClawMoveTimeOut                         : 1; //�˶���ʱ                       
//    uint32_t    ClawSolenoidRun_Err                     : 1; //�����������ʵ���˶�����       

//    uint32_t    BowRiseSolenoidInitErr                  : 1; //��ŷ��Լ����          
//    uint32_t    BowRiseSensorLinkLose                   : 1; //������ʧ��              
//    uint32_t    BowRiseSensorDataError                  : 1; //�������ߴ����������쳣�����ޣ�
//    uint32_t    BowRiseMotorPosLinkLose                 : 1; //������ʧЧ            
//    uint32_t    BowRiseMoveTimeOut                      : 1; //�˶���ʱ                 
//    uint32_t    BowRiseSolenoidRun_Err                  : 1; //�����������ʵ���˶�����      
//    
//    uint32_t    RotateSolenoidInitErr                   : 1; //��ŷ��Լ����         
//    uint32_t    RotateSensorLinkLose                    : 1; //������ʧ��  
//    uint32_t    RotateSensorDataError                   : 1; //��ת�����������쳣�����ޣ�    
//    uint32_t    RotateMotorPosLinkLose                  : 1; //������ʧЧ              
//    uint32_t    RotateMoveTimeOut                       : 1; //�˶���ʱ               
//    uint32_t    RotateSolenoidRun_Err                   : 1; //�����������ʵ���˶�����       

//    uint32_t    EepromError                             : 1; //Eeprom����
//    uint32_t    LiftToBottomErr                         : 1; //һ������ָ�����
//    uint32_t    Claw_Rotate_Err                         : 1; //������תָ���ͻ
//    uint32_t    Claw_Rotate_Switch_Err                  : 1; //������ת���Ʒ��л�����
//    uint32_t    SideSensorLinkLose                      : 1; //���ƴ�����ʧ��          
//    uint32_t    SideSensorErr                           : 1; //���ƴ������쳣          
//    uint32_t    SideMoveTimeOut                         : 1; //�����˶���ʱ               
//    uint32_t    Claw_Lift_CmdErr                               : 1;  //����ֽ���ֹ�½�
//};

//typedef union
//{
//    uint32_t    all;
//    uint8_t     RevBuff[4];
//    struct      Error_BITS_Solenoid   bit;
//} Fault_REG_Solenoid;

struct Error_BITS_Control
{
    uint32_t    HostLinkLose            : 1; //��λ��ʧ��   
    uint32_t    VCULinkLose             : 1; //VCU����           
    uint32_t    RotatingLinkLose        : 1; //��ת���ʧ��                 
    uint32_t    TelescopicLinkLose      : 1; //�������ʧ��             
    uint32_t    HuggingLinkLose         : 1; //�������ʧ��                   
    uint32_t    GlandLinkLose           : 1; //̧ѹ���ʧ��     
    uint32_t    MOTOR1CurrOver          : 1; //MOTOR1����             
    uint32_t    MOTOR2CurrOver          : 1; //MOTOR2���� 
    
    uint32_t    PCCurrOver              : 1; //PC����       
    uint32_t    SensorCurrOver          : 1; //����������                                        
    uint32_t    TurnError               : 1; //ת�����                 
    uint32_t    EmergencyStop           : 1; //0�༱ͣ����           
    uint32_t    ExcessTemperature       : 1; //�¶ȹ���
    uint32_t    EepromReadErr           : 1; //Eeprom������
    uint32_t    EepromWriteErr          : 1; //Eepromд�����
    uint32_t    TelescopicMBDRelayErr   : 1; //����̵�������
    
};
struct Warning_BITS
{
    uint32_t    PC_Power24V_Low             : 1; //PC24V��ѹ��
    uint32_t    HandleBatteryLow            : 1; //�ֱ���ص�ѹ��
    uint32_t    BLDC1_Power24V_Low          : 1; //���1����24V��ѹ��
    uint32_t    BLDC2_Power24V_Low          : 1; //���2����24V��ѹ��
    uint32_t    Sensor_Power24V_Low         : 1; //���������ѹ��
    uint32_t    TurnWarning                 : 1; //���򾯸�
    uint32_t    IMULinkLose                 : 1; //IMUʧ��
    uint32_t    RadarLinkLose                 : 1; //�״�ʧ��
    
    uint32_t    DrawWireLinkLose            : 1;
    uint32_t    VolLow            : 1;
    uint32_t    MotorNoSt            : 1;
    uint32_t    IMUInitErr            : 1;//IMU��ʼ��ʧ��
    uint32_t    Res2            : 1;
    uint32_t    Res3            : 1;
    uint32_t    Res4            : 1;
    uint32_t    Res5            : 1;
    
    uint32_t    Res31           : 1; //����80V��ѹ��
    uint32_t    Res32       : 1; //���õ�ص�ѹ��
    uint32_t    Res33              : 1; //IMU��ʼ��ʧ��
    uint32_t    Res34             : 1; //IMUʧ��
    uint32_t    Res35      : 1; //Һѹ�嶯��24V��ѹ��
    uint32_t    Res36       : 1; //ת��嶯��24V��ѹ��
    uint32_t    Res37           : 1; //��תָ����󣨱��и߶ȹ��ͣ���ֹ��ת��
    uint32_t    Res38           : 1;
    
    uint32_t    Res41           : 1; //����80V��ѹ��
    uint32_t    Res42           : 1; //���õ�ص�ѹ��
};
//struct Solenoid_Warning_BITS
//{
//    uint32_t    SolenoidResWarning                  : 1; //��ŷ���Ȧ�迹�쳣
//    uint32_t    HighTempWarning                     : 1; //�¶ȹ���
//    uint32_t    HighVoltageWarning                  : 1; //�ߵ�ѹ
//    uint32_t    LowVoltageWarning                   : 1; //�͵�ѹ
//    uint32_t    PowerLossWarning                    : 1; //�͵�ѹ
//    uint32_t    Lift2SensorLinkWarning              : 1; //�����͸��ߴ�����ʧ��
//    uint32_t    Lift2SensorDataWarning              : 1; //�����͸��ߴ�������ֵ�쳣
//    uint32_t    RotateSNError                       : 1; //��תָ����󣨱��и߶ȹ��ͣ���ֹ��ת��
//    uint32_t    SideShiftSNWarning                  : 1; //����ָ�����
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
    
//} Fault_Handle ;//��������ܽṹ��
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
}Energy_Struct;         //���ļ���ṹ��

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
extern int16_t  m_nSolenoidLife;                   //�Ƹ˵����ͨ������
extern int16_t  m_nSteeringLife;                   //ת������ͨ������
extern  int16_t  IMULife ;                        //IMUͨ������
extern int16_t  m_nHostLife;
extern int16_t  HostSpdLife ;                  //��λ���ٶȸ��¼���
extern uint16_t LightState;     
extern int16_t  HARTCMDLife ;//ң��������
extern int16_t Temperature;        //CPU�¶�
extern int16_t BLDC_24V, VALVE_24V;
extern uint16_t  HostFaultClear;
//extern Solenoid_Warning_REG Solenoid_Warning;
extern uint16_t Radar_Zone_Left,Radar_Zone_Right;
extern uint8_t Stop_1A_State_Flag;               //��⼱ͣ��־
extern uint8_t   HostShutDowncmd;
extern Light_REG Light_State;                                     //����ƿ���
extern RadarOut RadarOut_R, RadarOut_L, RadarResult;
extern Energy_Struct Energy_PC, Energy_SENSOR, Energy_MOTOR1, Energy_MOTOR2, Energy_ElecMag;
//-------------------- public functions -------------------------------------
extern uint8_t ElecMagSta;
uint16_t FaultNum_Get ( void );
void Monitordisplay ( void );
void Stop_1A_State ( void );
void Buzzer_Warning ( void );
void HeartBeat_Check (void);         //����״̬���
void Motor_24Check(void) ;           //����24V���
void MonitorProcess(void);
void BumperStop(void);
void LightStateCheck(void);        //�ƹ�״̬ 0���̵Ƴ�����1���̵���˸��2�ǻƵƳ�����3�ǻƺ���˸��4�Ǻ�Ƴ���
uint16_t SelectLeftRadarZone (void);
uint16_t SelectRightRadarZone (void);
void Radar_Process ( void ) ;
void RGBCtrl();
void Energy_Consumption(void);
#endif // _XXX_H_

//-----------------------End of file------------------------------------------
/** @}*/
