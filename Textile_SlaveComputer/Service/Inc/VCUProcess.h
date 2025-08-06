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
#ifndef _VCUProcess_H_
#define _VCUProcess_H_

#include "stdint.h"
#include "AngleMath.h"
#include "DataTypesDef.h"
#define CMD_FIFO_SIZE                512
//#define Equip_HyDriveBoard           0x01
//#define Equip_SteerDriveBoard        0x02

//#define Reg_ChassisParam             0x201
//#define Reg_ForkParam                0x301
//#define Reg_PartsAndValve1Param      0x401
//#define Reg_PartsAndValve2Param      0x501

//#define Reg_ForkLiftState1           0x181
//#define Reg_ForkLiftState2           0x281
//#define Reg_ForkLiftState3           0x381
//#define Reg_ForkLiftState4           0x481
//#define Reg_ForkLiftState5           0x182
//#define Reg_ForkLiftState6                      0x19E
//#define Reg_HARTCMD                             0x1A3//

/*textile*/
#define Step_AGVCtrlCmd1                        0x226
#define Step_AGVCtrlCmd2                        0x222
#define Step_AGVBMSCtrlCmd                      0x6AC
#define Step_AGVTurnCtrlCmd1                    0x201//10ms---100hz
#define Step_AGVTurnCtrlCmd2                    0x301

#define Step_AGVChargeSW                        0x1F0//0x101
#define Step_AGVBMSStaOrigDataFb                0x2F4
#define Step_AGVDataFb1                         0x1A6//20ms---50Hz
#define Step_AGVDataFb2                         0x1A7//20ms---50Hz
#define Step_AGVPumpCtrFb                       0x1A2//20ms---50Hz

#define Step_AGVTurnFb                          0x181//10ms---100hz
#define Step_RCCtrlCmdFb                        0x88//�ֱ�
#define Step_RCWarningFb                        0x89
#define BrakeDeep       50

typedef struct
{
    uint32_t ID;
    uint32_t Len;
    uint8_t Buff[CMD_FIFO_SIZE];
} UserFrame_t;
extern UserFrame_t Tx_frame;

struct Reg_ChassisEnabled_Bits
{
    uint8_t AutoModeEnabled                 : 1; //�Զ�ģʽʹ�ܣ�1��Ч��0��Ч ��1ʱ��VCU��Ӧ�л����Զ�ģʽ
    uint8_t WalkingEnabled                  : 1; //����ʹ�ܣ�1��Ч��0��Ч
    uint8_t BrakeEnabled                    : 1; //�ƶ�ʹ�ܣ�1��Ч��0��Ч
    uint8_t SteeringEnabled                 : 1; //ת��ʹ�ܣ�1��Ч��0��Ч
    uint8_t ChargingEnabled                 : 1; //�Զ����ʹ�ܣ�1��Ч��0��Ч
    uint8_t ForwardEnabled                  : 1; //ǰ��ʹ�ܣ�1��Ч��0��Ч
    uint8_t ReverseEnabled                  : 1; //����ʹ�ܣ�1��Ч��0��Ч
    uint8_t ParkingBrakeEnabled             : 1; //פ���ƶ�ʹ�ܣ�1��Ч��0��Ч
};
typedef union
{
    uint8_t all;
    struct Reg_ChassisEnabled_Bits bit;
} Reg_ChassisEnabled_Byte;
typedef struct
{
    Reg_ChassisEnabled_Byte u8ChassisEnabled;   //����ʹ��
    uint8_t u8SerServiceBrakes;                 //�г��ƶ�
    uint8_t u8SteerTargetAngle_Low;             //ת��Ŀ��Ƕȵ��ֽ�
    uint8_t u8SteerTargetAngle_High;            //ת��Ŀ��Ƕȸ��ֽ�
    uint8_t u8ChassisMotorTargetSpd_Low;        //���ߵ��Ŀ��ת�ٵ��ֽ�
    uint8_t u8ChassisMotorTargetSpd_High;       //���ߵ��Ŀ��ת�ٸ��ֽ�
    uint8_t u8ChassisMotorAcc;                  //���ߵ��������
    uint8_t u8ChassisMotorDec;                  //���ߵ��������
} RegDef_ChassisParam;
extern RegDef_ChassisParam RegReal_ChassisParam;//�Ĵ���0x201

struct Reg_ForkEnabled_Bits
{
    uint8_t PumpMotorEnabled                : 1; //�ͱõ��ʹ�ܣ�1ʹ�ܣ�0��ֹ
    uint8_t PumpMotorBrake                  : 1; //�ͱõ��ɲ����1��Ч��0��Ч
    uint8_t Rotate_or_Claw                  : 1; //��ת�ͱ����л���1���У�0��ת
    uint8_t res0                            : 1;                    
    uint8_t res1                            : 1;             
    uint8_t res2                            : 1;               
    uint8_t res3                            : 1;               
    uint8_t res4                            : 1;               
};
typedef union
{
    uint8_t all;
    struct Reg_ForkEnabled_Bits bit;
} Reg_ForkEnabled_Byte;
typedef struct
{
    Reg_ForkEnabled_Byte u8ForkEnabled;         //����ʹ��
    uint8_t u8PumpMotorTargetSpd_Low;           //�ͱõ��Ŀ��ת�ٵ��ֽ�
    uint8_t u8PumpMotorTargetSpd_High;          //�ͱõ��Ŀ��ת�ٸ��ֽ�
    uint8_t u8BreakCurrent;                     //���ŷ���������ƫ������-10 ~ 0��Ĭ��ֵ10��10A���������10A��
    uint8_t u8BreakSpeed;                       //���ŷ��ٶȿ���ƫ������-90 ~ 10��Ĭ��ֵ90��90%���������100%��
    uint8_t u8PumpMotorAcc;                     //�ͱõ��������
    uint8_t u8PumpMotorDec;                     //�ͱõ��������
    uint8_t res2;                               //Ԥ�� 
} RegDef_ForkParam;
extern RegDef_ForkParam RegReal_ForkParam;      //�Ĵ���0x301

struct Reg_PartSwitch_Bits
{
    uint8_t TurnLeftLight                   : 1; //����ƣ�1������0��
    uint8_t TurnRightLight                  : 1; //����ƣ�1������0��
    uint8_t Headlights                      : 1; //ǰ��ƣ�1����0��
    uint8_t WidthIndicator                  : 1; //ʾ��ƣ�1����0��
    uint8_t WarningLights                   : 1; //��ʾ�ƣ�1����0��
    uint8_t Horn                            : 1; //���ȣ�1���ѣ�0��Ч
    uint8_t ReversingLightsAndBeeps         : 1; //�����Ƽ�����������1��Ч��0��Ч
    uint8_t Wiper                           : 1; //�������1��Ч��0��Ч
};
typedef union
{
    uint8_t all;
    struct Reg_PartSwitch_Bits bit;
} Reg_PartSwitch_Byte;
typedef struct
{
    Reg_PartSwitch_Byte u8PartSwitch;           //�豸����
    uint8_t res0;                               //Ԥ��
    uint8_t LiftValveStartValue_Low;            //��������������ֵ���ֽ�
    uint8_t LiftValveStartValue_High;           //��������������ֵ���ֽ�
    uint8_t DropValveStartValue_Low;            //�½�����������ֵ���ֽ�
    uint8_t DropValveStartValue_High;           //�½�����������ֵ���ֽ�
    uint8_t ForwardValveStartValue_Low;         //ǰ�����������ֵ���ֽ�
    uint8_t ForwardValveStartValue_High;        //ǰ�����������ֵ���ֽ� 
} RegDef_PartsAndValve1Param;
extern RegDef_PartsAndValve1Param RegReal_PartsAndValve1Param;      //�Ĵ���0x401

typedef struct
{
    uint8_t ReclineValveStartValue_Low;         //��������������ֵ���ֽ�
    uint8_t ReclineValveStartValue_High;        //��������������ֵ���ֽ�
    uint8_t ShiftLeftValveStartValue_Low;       //���Ʊ���������ֵ���ֽ�
    uint8_t ShiftLeftValveStartValue_High;      //���Ʊ���������ֵ���ֽ�
    uint8_t ShiftRightValveStartValue_Low;      //���Ʊ���������ֵ���ֽ�
    uint8_t ShiftRightValveStartValue_High;     //���Ʊ���������ֵ���ֽ�
    uint8_t res0;                               //Ԥ��
    uint8_t res1;                               //Ԥ��
} RegDef_PartsAndValve2Param;
extern RegDef_PartsAndValve2Param RegReal_PartsAndValve2Param;      //�Ĵ���0x501

struct Reg_TravelMode_Bits
{
//    uint8_t AllowedAutoMode               : 1; //�Ƿ���������Զ�ģʽ״̬��1����0������
    uint8_t res                             : 1; //Ԥ��
    uint8_t ForkLiftMode                    : 1; //����ģʽ״̬��1�Զ���0�ֶ�
    uint8_t ServiceBrakeMode                : 1; //�г��ƶ�״̬��1�ƶ���0�ͷ�
    uint8_t ParkingBrakeMode                : 1; //פ���ƶ�״̬��1�ͷţ�0�ƶ�
    uint8_t res0                            : 1; //Ԥ��
    uint8_t res1                            : 1; //Ԥ��
    uint8_t res2                            : 1; //Ԥ��
    uint8_t res3                            : 1; //Ԥ��
};
typedef union
{
    uint8_t all;
    struct Reg_TravelMode_Bits bit;
} Reg_TravelMode_Byte0;
struct Reg_CANConState_Bits
{
    uint8_t SmartDrivingState               : 1; //VCU�����Ǽ�CAN����״̬��1���ӣ�0δ����
    uint8_t ChassisMotorCtrlState           : 1; //VCU�������ߵ��CAN����״̬��1���ӣ�0δ����
    uint8_t SteerMotorCtrlState             : 1; //VCU����ת����CAN����״̬��1���ӣ�0δ����
    uint8_t PumpMotorCtrlState              : 1; //VCU�����ͱõ��CAN����״̬��1���ӣ�0δ����
    uint8_t AbsoluteEncoderState            : 1; //VCU����ת�����ֵ������CAN����״̬��1���ӣ�0δ����
    uint8_t BMSState                        : 1; //VCU����BMS CAN����״̬��1���ӣ�0δ����
    uint8_t ElectricActuatorState           : 1; //VCU���ӵ綯��ŷ�CAN����״̬��1���ӣ�0δ����
    uint8_t res0                            : 1; //Ԥ��
};
typedef union
{
    uint8_t all;
    struct Reg_CANConState_Bits bit;
} Reg_CANConState_Byte1;
struct Reg_BoundaryState_Bits
{
    uint8_t ForkLimitSignal                 : 1; //��������λ�źţ�1�У�0��
    uint8_t LeftForkBarrierState            : 1; //������ϰ�����ź�״̬��1��Ч��0��Ч
    uint8_t RightForkBarrierState           : 1; //�һ����ϰ�����ź�״̬��1��Ч��0��Ч
    uint8_t CargoHeartbeatState             : 1; //�������ź�״̬��1��Ч��0��Ч
    uint8_t EmergencyState                  : 1; //��ͣ����״̬��1�������Զ�����0���������Զ���
    uint8_t TouchedgeSignal                 : 1; //�����źţ�1�У�0��
    uint8_t res0                            : 1; //Ԥ��
    uint8_t res1                            : 1; //Ԥ��
};
typedef union
{
    uint8_t all;
    struct Reg_BoundaryState_Bits bit;
} Reg_BoundaryState_Byte2;
typedef struct
{
    Reg_TravelMode_Byte0 u8TravelMode;          //��ʻģʽ
    Reg_CANConState_Byte1 u8CANConState;        //CAN����״̬
    Reg_BoundaryState_Byte2 u8BoundaryState;    //�߽���״̬
    uint8_t u8ServiceBrakeFeedback;             //�г��ƶ��ʰٷֱȷ�����0-255��Ӧ0-100%
    uint8_t u8TiltedAngleFeedback_Low;          //��б��λ���Ƕȼ�ⷴ��0-5000��Ӧ0-5V ���ֽ�
    uint8_t u8TiltedAngleFeedback_High;         //��б��λ���Ƕȼ�ⷴ��0-5000��Ӧ0-5V ���ֽ�
    uint8_t u8SideshiftAngleFeedback_Low;       //���Ƶ�λ���Ƕȼ�ⷴ��0-5000��Ӧ0-5V ���ֽ�
    uint8_t u8SideshiftAngleFeedback_High;      //���Ƶ�λ���Ƕȼ�ⷴ��0-5000��Ӧ0-5V ���ֽ�
} RegDef_ForkLiftState1;
extern RegDef_ForkLiftState1 RegReal_ForkLiftState1;  //�Ĵ���0x181

typedef struct
{
    uint8_t u8SteerAngleFeedback_Low;            //ת��Ƕȷ���-7000 ~ 7000 ��Ӧ -70��~70�� ���ֽ�
    uint8_t u8SteerAngleFeedback_High;           //ת��Ƕȷ���-7000 ~ 7000 ��Ӧ -70��~70�� ���ֽ�
    uint8_t res0;                               //Ԥ��
    uint8_t res1;                               //Ԥ��
    uint8_t u8ChassisMotorCtrlErr_Low;          //���ߵ�ع��ϴ��뷴�� ���ֽ�
    uint8_t u8ChassisMotorCtrlErr_High;         //���ߵ�ع��ϴ��뷴�� ���ֽ�
    uint8_t u8SteerMotorCtrlErr_Low;            //ת���ع��ϴ��뷴�� ���ֽ�
    uint8_t u8SteerMotorCtrlErr_High;           //ת���ع��ϴ��뷴�� ���ֽ�
} RegDef_ForkLiftState2;
extern RegDef_ForkLiftState2 RegReal_ForkLiftState2;  //�Ĵ���0x281

typedef struct  //������int�ͣ�ʵ����uint(���ݲɼ������з������⣬�����ݴ���ű��д򲹶������ݲɼ���֮���޸�)
{
    int8_t u8ChassisMotorSpd_Low;              //���ߵ��ת�ٷ������ֽ� -5000 ~ 5000 ��Ӧ-5000 ~ 5000 rpm
    int8_t u8ChassisMotorSpd_High;             //���ߵ��ת�ٷ������ֽ� -5000 ~ 5000 ��Ӧ-5000 ~ 5000 rpm
    int8_t u8ChassisMotorCurrent_Low;          //���ߵ�������������ֽ� �C10000 �C 10000��Ӧ �C1000 �C 1000 A
    int8_t u8ChassisMotorCurrent_High;         //���ߵ�������������ֽ� �C10000 �C 10000��Ӧ �C1000 �C 1000 A
    int8_t u8ChassisMotorTemp_Low;             //���ߵ���¶ȷ������ֽ� �C1000 �C 3000��Ӧ �C100 �C 300��C
    int8_t u8ChassisMotorTemp_High;            //���ߵ���¶ȷ������ֽ� �C1000 �C 3000��Ӧ �C100 �C 300��C
    int8_t u8ChassisMotorCtrlTemp_Low;         //���ߵ���¶ȷ������ֽ� �C1000 �C 3000��Ӧ �C100 �C 300��C
    int8_t u8ChassisMotorCtrlTemp_High;        //���ߵ���¶ȷ������ֽ� �C1000 �C 3000��Ӧ �C100 �C 300��C
} RegDef_ForkLiftState3;
extern RegDef_ForkLiftState3 RegReal_ForkLiftState3;  //�Ĵ���0x381

typedef struct
{
    uint8_t u8PumpMotorSpd_Low;                 //�ͱõ��ת�ٷ������ֽ� -5000 ~ 5000 ��Ӧ-5000 ~ 5000 rpm
    uint8_t u8PumpMotorSpd_High;                //�ͱõ��ת�ٷ������ֽ� -5000 ~ 5000 ��Ӧ-5000 ~ 5000 rpm
    uint8_t u8PumpMotorCurrent_Low;             //�ͱõ�������������ֽ� �C10000 �C 10000��Ӧ �C1000 �C 1000 A
    uint8_t u8PumpMotorCurrent_High;            //�ͱõ�������������ֽ� �C10000 �C 10000��Ӧ �C1000 �C 1000 A
    uint8_t u8PumpMotorTemp_Low;                //�ͱõ���¶ȷ������ֽ� �C1000 �C 3000��Ӧ �C100 �C 300��C
    uint8_t u8PumpMotorTemp_High;               //�ͱõ���¶ȷ������ֽ� �C1000 �C 3000��Ӧ �C100 �C 300��C
    uint8_t u8PumpMotorCtrlTemp_Low;            //�ͱõ���¶ȷ������ֽ� �C1000 �C 3000��Ӧ �C100 �C 300��C
    uint8_t u8PumpMotorCtrlTemp_High;           //�ͱõ���¶ȷ������ֽ� �C1000 �C 3000��Ӧ �C100 �C 300��C
} RegDef_ForkLiftState4;
extern RegDef_ForkLiftState4 RegReal_ForkLiftState4;  //�Ĵ���0x481

typedef struct
{
    uint8_t u8PumpMotorCtrlErr_Low;             //�ͱõ�ع��ϴ��뷴�� ���ֽ�
    uint8_t u8PumpMotorCtrlErr_High;            //�ͱõ�ع��ϴ��뷴�� ���ֽ�
    uint8_t res0;                               //Ԥ��
    uint8_t res1;                               //Ԥ��
    uint8_t u8VCUErrFeedback;                   //VCUϵͳ���Ϸ���
    uint8_t res2;                               //Ԥ��
    uint8_t res3;                               //Ԥ��
    uint8_t res4;                               //Ԥ��
} RegDef_ForkLiftState5;
extern RegDef_ForkLiftState5 RegReal_ForkLiftState5;  //�Ĵ���0x182

struct Reg_BatteryError1_Bits
{
    uint8_t OverVoltage                     : 1; //��ѹ���ߣ�0������1��ѹ����
    uint8_t OverRelease                     : 1; //������ţ�0������1�������
    uint8_t CommunicationLost               : 1; //ͨѶ�жϣ�0������1�ж�
    uint8_t UnderVoltage                    : 1; //����Ƿѹ��0������1����Ƿѹ
    uint8_t OverCurrent                     : 1; //��������0������1������
    uint8_t OverTemperature                 : 1; //���±�����0������1���±���
    uint8_t TempProtection                  : 1; //�¶ȱ�����0������1�¶ȱ���
    uint8_t ChargingConnection              : 1; //������ӣ�0������1�������
};
typedef union
{
    uint8_t all;
    struct Reg_BatteryError1_Bits bit;
} Reg_BatteryError_Byte6;
struct Reg_BatteryError2_Bits
{
    uint8_t ForcedFullCharge                : 1; //ǿ�����䣺0������1����������
    uint8_t OffPowerProtection              : 1; //�Ϲ��ʱ�����0������1���ֹͣ�ŵ�
    uint8_t res0                            : 1; //Ԥ��
    uint8_t res1                            : 1; //Ԥ��
    uint8_t res2                            : 1; //Ԥ��
    uint8_t res3                            : 1; //Ԥ��
    uint8_t res4                            : 1; //Ԥ��
    uint8_t res5                            : 1; //Ԥ��
};
typedef union
{
    uint8_t all;
    struct Reg_BatteryError2_Bits bit;
} Reg_BatteryError_Byte7;
typedef struct
{
    uint8_t u8BatteryVoltage_Low;                //����ܵ�ѹ���ֽ� ��Χ��0 ~10000 �������ӣ�0.1V/bit ʵ�����̣�0��1000 V
    uint8_t u8BatteryVoltage_High;               //����ܵ�ѹ���ֽ� ��Χ��0 ~10000 �������ӣ�0.1V/bit ʵ�����̣�0��1000 V
    uint8_t u8BatteryCurrent_Low;                //����ܵ������ֽ� ��Χ��0 ~ 65535 ƫ������-32000 �������ӣ�0.1A/bit ʵ�����̣�-3200A��3353.5 A
    uint8_t u8BatteryCurrent_High;               //����ܵ������ֽ� ��Χ��0 ~ 65535 ƫ������-32000 �������ӣ�0.1A/bit ʵ�����̣�-3200A��3353.5 A
    uint8_t u8SOC;                               //SOC ��Χ��0 ~ 250 �������ӣ�0.4%/bit ʵ�����̣�0��100%
    uint8_t u8BatteryCapacity;                   //������� ��Χ��0 ~ 250 �������ӣ�5Ah/bit ʵ�����̣�0��1250Ah
    Reg_BatteryError_Byte6 u8BatteryError1;      //��ع�����Ϣ
    Reg_BatteryError_Byte7 u8BatteryError2;      //��ع�����Ϣ
} RegDef_ForkLiftState6;
extern RegDef_ForkLiftState6 RegReal_ForkLiftState6;  //�Ĵ���0x19E

typedef struct
{
    uint8_t u8HARTCMD_Forkside;                  //����ָ�0x01:���ƣ�0xFF:����
    uint8_t u8HARTCMD_DriveMode;                   //�н�ָ�0x01�����ˣ�0xFF��ǰ��
    uint8_t u8HARTCMD_SteerAngle;                //�Ƕ�ָ�0x00-0xFF������ת��Ƕȣ�-90��-90�㣩
    uint8_t u8HARTCMD_EmergencyStop;                  //ģʽ��bit1����ͣ����
    uint8_t u8HARTCMD_ForkLift;                  //����/�½���0x01�������½���0xFF����������
    uint8_t u8HARTCMD_ForkClaw;                 //���ϣ�0x01������0xFF����
    uint8_t u8HARTCMD_ForkRotate;               //��ת��0x01��ȥ180�㣬0xFF����0��
    uint8_t u8HARTCMD_ForkBowrise;                //������0x01��������ƣ�0xFF������ǰ��
} RegDef_HARTCMD;
extern RegDef_HARTCMD RegReal_HARTCMD;                //�Ĵ���0x1A3���ֲ�������ṹ��

typedef struct
{
    uint8_t u8ForkLiftMode;                      //����ģʽ״̬��1�Զ���0�ֶ�
    uint8_t u8ServiceBrakeMode;                  //�г��ƶ�״̬��1�ƶ���0�ͷ�
    uint8_t u8ParkingBrakeMode;                  //פ���ƶ�״̬��1�ͷţ�0�ƶ�
    uint8_t u8SmartDrivingState;                 //VCU�����Ǽ�CAN����״̬��1���ӣ�0δ����
    uint8_t u8ChassisMotorCtrlState;             //VCU�������ߵ��CAN����״̬��1���ӣ�0δ����
    uint8_t u8SteerMotorCtrlState;               //VCU����ת����CAN����״̬��1���ӣ�0δ����
    uint8_t u8PumpMotorCtrlState;                //VCU�����ͱõ��CAN����״̬��1���ӣ�0δ����
    uint8_t u8AbsoluteEncoderState;              //VCU����ת�����ֵ������CAN����״̬��1���ӣ�0δ����
    uint8_t u8BMSState;                          //VCU����BMS CAN����״̬��1���ӣ�0δ����
    uint8_t u8ElectricActuatorState;             //VCU����Һѹ������CAN����״̬��1���ӣ�0δ����
    uint8_t u8ForkLimitSignal;                   //��������λ�źţ�1�У�0��
    uint8_t u8LeftForkBarrierState;              //������ϰ�����ź�״̬��1��Ч��0��Ч
    uint8_t u8RightForkBarrierState;             //�һ����ϰ�����ź�״̬��1��Ч��0��Ч
    uint8_t u8CargoHeartbeatState;               //�������ź�״̬��1��Ч��0��Ч
    uint8_t u8EmergencyState;                    //��ͣ����״̬��1�������Զ�����0���������Զ���
    uint8_t u8TouchedgeSignal;                   //�����źţ�1�У�0��
    uint8_t u8BrakeFeedback;                     //�г��ƶ��ʰٷֱȷ�����0-255��Ӧ0-100%
    uint16_t u16TiltedAngleFeedback;             //��б��λ���Ƕȼ�ⷴ��0-5000��Ӧ0-5V����λmV
    uint16_t u16SideshiftAngleFeedback;          //���Ƶ�λ���Ƕȼ�ⷴ��0-5000��Ӧ0-5V����λmV    ����Ϊ0x181�Ĵ������ݽ�������
    
    int16_t s16SteerAngleFeedback;               //ת��Ƕȷ���-7000 ~ 7000 ��Ӧ -70��~70�㣬��λ0.01��
    uint16_t u16ChassisMotorCtrlErr;             //���ߵ�ع��ϴ��뷴��
    uint16_t u16SteerMotorCtrlErr;               //ת���ع��ϴ��뷴��    ����Ϊ0x281�Ĵ������ݽ�������
    
    int16_t s16ChassisMotorSpd;                  //���ߵ��ת�ٷ��� -5000 ~ 5000 ��Ӧ-5000 ~ 5000 rpm
    int16_t s16ChassisMotorCurrent;              //���ߵ���������� �C10000 �C 10000��Ӧ �C1000 �C 1000 A����λ0.1A
    int16_t s16ChassisMotorTemp;                 //���ߵ���¶ȷ��� �C1000 �C 3000��Ӧ �C100 �C 300��C����λ0.1��
    int16_t s16ChassisMotorCtrlTemp;             //���ߵ���¶ȷ��� �C1000 �C 3000��Ӧ �C100 �C 300��C����λ0.1��    ����Ϊ0x381�Ĵ������ݽ�������
    
    int16_t s16PumpMotorSpd;                     //�ͱõ��ת�ٷ��� -5000 ~ 5000 ��Ӧ-5000 ~ 5000 rpm
    int16_t s16PumpMotorCurrent;                 //�ͱõ���������� �C10000 �C 10000��Ӧ �C1000 �C 1000 A����λ0.1A
    int16_t s16PumpMotorTemp;                    //�ͱõ���¶ȷ��� �C1000 �C 3000��Ӧ �C100 �C 300��C����λ0.1��
    int16_t s16PumpMotorCtrlTemp;                //�ͱõ���¶ȷ��� �C1000 �C 3000��Ӧ �C100 �C 300��C����λ0.1��    ����Ϊ0x481�Ĵ������ݽ�������
    
    uint16_t u16PumpMotorCtrlErr;                //�ͱõ�ع��ϴ��뷴��
    uint8_t u8VCUSysErr;                         //VCUϵͳ���Ϸ���    ����Ϊ0x182�Ĵ������ݽ�������
    
    uint16_t u16BatteryVoltage;                  //����ܵ�ѹ ��Χ��0 ~10000 �������ӣ�0.1V/bit ʵ�����̣�0��1000 V
    int16_t s16BatteryCurrent;                   //����ܵ��� ��Χ��0 ~ 65535 ƫ������-32000 �������ӣ�0.1A/bit ʵ�����̣�-3200A��3353.5 A
    uint8_t u8BatterySOC;                        //SOC ��Χ��0 ~ 250 �������ӣ�0.4%/bit ʵ�����̣�0��100%
    uint8_t u8BatteryCapacity;                   //������� ��Χ��0 ~ 250 �������ӣ�5Ah/bit ʵ�����̣�0��1250Ah
    uint8_t u8OverVoltage;                       //��ѹ���ߣ�0������1��ѹ����
    uint8_t u8OverRelease;                       //������ţ�0������1�������
    uint8_t u8CommunicationLost;                 //ͨѶ�жϣ�0������1�ж�
    uint8_t u8UnderVoltage;                      //����Ƿѹ��0������1����Ƿѹ
    uint8_t u8OverCurrent;                       //��������0������1������
    uint8_t u8OverTemperature;                   //���±�����0������1���±���
    uint8_t u8TempProtection;                    //�¶ȱ�����0������1�¶ȱ���
    uint8_t u8ChargingConnection;                //������ӣ�0������1�������
    uint8_t u8ForcedFullCharge;                  //ǿ�����䣺0������1����������
    uint8_t u8OffPowerProtection;                //�Ϲ��ʱ�����0������1���ֹͣ�ŵ�    ����Ϊ0x19E�Ĵ������ݽ�������
} VCU_ParamStruct;
extern VCU_ParamStruct VCU_ParamHandle;          //VCU���ݽ����ṹ��

typedef struct
{
    uint8_t u8StartOfTransfer;                   //������俪ʼ��־
    uint8_t u8EndOfTransfer;                     //������������־
    uint8_t u8Toggle;                            //��������л�λ��־
    uint8_t u8TransferID;                        //���ݴ��䵱ǰ����
} CanTransferTypeDef;

typedef struct
{
    _iq LiftPos;
    _iq ClawPos;
    _iq BowRisePos;
    _iq RotatePos;
    _iq SidePos;
    uint8_t LiftState;
    uint8_t ClawState;
    uint8_t RowRiseState;
    uint8_t RotateState;
    uint8_t SideState;
    
}ForkLiftPos;

typedef struct
{
    uint16_t SideMoveRightCnt;
    uint16_t SideMoveLeftCnt;
    uint16_t LiftUpCnt;
    uint16_t LiftDownCnt;
    uint16_t ClawOpenCnt;
    uint16_t ClawClosCnt;
    uint16_t BowRiseForCnt;
    uint16_t BowRiseRevCnt;
    uint16_t Rotate_0_Cnt;
    uint16_t Rotate_180_Cnt;
}RCCnt;
/*textile*/
struct AGVCtrlCmd1_Byte5
{
    uint8_t up                              : 1; //1:����
//    uint8_t forward                         : 1; //1:ǰ��
//    uint8_t backward                        : 1; //1:����
    uint8_t res1                            : 7; //Ԥ��
};
struct TurnStaWord
{
    uint16_t ReadyPowerOn           : 1; //׼���ϵ�
    uint16_t PowerOn                : 1; //���ϵ�
    uint16_t MotorEN                : 1; //ʹ��
    uint16_t MotorErr               : 1; //����
    
    uint16_t ForbiddenVoltOut       : 1; //��ֹ�����ѹ
    uint16_t RapidStop              : 1; //����ֹͣ
    uint16_t ForbiddenPowerOn       : 1;//�ϵ��ֹ
    uint16_t MotorWarning           : 1;//����
    
    uint16_t res                    : 1;//����
    uint16_t RemoteCtrl             : 1;//Զ�̿���
    uint16_t PosReached             : 1;//Ŀ�굽λ
    uint16_t InterLimitActivation   : 1;//�ڲ���λ����
    
    uint16_t ImpulseResponse        : 1;//������Ӧ
    uint16_t TrackingErr            : 1;//�������
    uint16_t ElecmagExcitation      : 1;//�ҵ��������
    uint16_t OriginLocated          : 1;//�ҵ�ԭ��
};
typedef union
{
    uint16_t     all;
    struct      TurnStaWord   bit;
} TurnStaWord_REG;

typedef union
{
    uint8_t all;
    struct AGVCtrlCmd1_Byte5 bit;
} Reg_AGVCtrlCmd1_Byte5;

typedef struct
{
    int16_t s16WalkSpd;         //�����ٶȣ� 1=1rpm,-32767-32767
    uint8_t u8Res3;                    //Ԥ��1
    uint8_t u8Res4;                    //Ԥ��2
    uint8_t EMVCtrl;                   //��ŷ�����
    uint8_t u8WalkDnPV;                //�����½�������---0~200��0-2000mA
    uint8_t u8Res7;                    //Ԥ��3
    uint8_t u8Res8;                    //Ԥ��4
} RegDef_Reg_AGVCtrlCmd1;
extern RegDef_Reg_AGVCtrlCmd1 Reg_AGVCtrlCmd1;
typedef struct
{
    uint8_t u8PumpSpd;                 //���ٶȣ� 1-255��Ӧ0-100%
    uint8_t u8Res2;                    //Ԥ��1
    uint8_t u8Res3;                    //Ԥ��2
    uint8_t u8Res4;                    //Ԥ��3
    uint8_t u8Res5;                    //Ԥ��4
    uint8_t u8Res6;                    //Ԥ��5
    uint8_t u8WalkAccTime;             //���߼���ʱ�䣺0-255 ��Ӧ0-25.5S 
    uint8_t u8WalkDecTime;             //���߼���ʱ�䣺0-255 ��Ӧ0-25.5S 
} RegDef_Reg_AGVCtrlCmd2;
extern RegDef_Reg_AGVCtrlCmd2 Reg_AGVCtrlCmd2;
typedef struct
{
    uint8_t u8AGVSta;                  //Agv״̬��0-���ã�1-�ŵ磻2-���
    uint8_t u8InquiryBMSSta;           //��ѯλ
    uint8_t u8ChargerCtrl;             //��������λ: 0XA1-�Զ����BMS�ر������0XA2-�Զ����BMS������磬0XA3-�ֶ����BMS�������
    uint8_t u8Res4;                    //Ԥ��1
    uint8_t u8Res5;                    //Ԥ��2
    uint8_t u8Res6;                    //Ԥ��3
    uint8_t u8Res7;                    //Ԥ��4
    uint8_t u8Res8;                    //Ԥ��5
}RegDef_AGVBMSCtrlCmd;
extern RegDef_AGVBMSCtrlCmd Reg_AGVBMSCtrlCmd;

typedef struct
{
    uint8_t Staflag;
    uint8_t Soc;
    uint8_t CycleTimes[2];                  // 
    uint8_t CurrSum[2];                     //-30000~35535(ƫ��)  *0.1A               
    uint8_t MaxTemp;                        //����¶�  -40-215
    uint8_t u8FaultCode;                    //������
}RegDef_AGVBMSStaOrigDataFb;
extern RegDef_AGVBMSStaOrigDataFb Reg_AGVBMSStaOrigDataFb;

typedef struct
{
    int16_t s16WalkSpd;         //�����ٶȣ� 1=1rpm,-32767-32767
    uint8_t u8Res3;                    //Ԥ��1
    uint8_t u8Res4;                    //Ԥ��2
    uint8_t Soc;
    uint8_t u8WalkFaultCode;           //���߹�����
    uint8_t u8SteeringFaultCode;       //ת�������
    uint8_t SignalFb;                  //�źŷ���   
}RegDef_AGVDataFb1;
extern RegDef_AGVDataFb1 Reg_AGVDataFb1;
typedef struct
{
    int32_t s32WalkCnt;             //���߱���������,��ת��+����ת��-
    uint8_t SignalFb;                  //�źŷ�����bit0��1���� T22ǰ�����أ�bit1��1����T33���˿���
    uint8_t u8Res6;                    //Ԥ��1
    uint8_t u8Res7;                    //Ԥ��2
    uint8_t u8Res8;                    //Ԥ��3
}RegDef_AGVDataFb2;
extern RegDef_AGVDataFb2 Reg_AGVDataFb2;

typedef struct
{
    int16_t s16PumpSpd;         //���ٶȣ� 1=1rpm,0-32767
    uint8_t u8WalkFaultCode;           //�ÿع�����
    int16_t s16WalkCurrFb;      //���߿������������� 1=1A,0~32767
    int16_t s16PumpCurrFb;      //�ÿؿ������������� 1=1A,0~32767
    uint8_t u8Res8;                    //Ԥ��1
}RegDef_AGVPumpCtrFb;
extern RegDef_AGVPumpCtrFb Reg_AGVPumpCtrFb;
typedef struct
{
    S32WordDef s32MotorPosCmd;//
    uint8_t u8WorkMode;     
    U16HalfWordDef u16CtrlWord; 
}RegDef_Reg_AGVTurnCtrlCmd1;

typedef struct
{
    int32_t s32SpdLpSpdCmd;
    int32_t s32PosLpSpdCmd;
}RegDef_AGVTurnCtrlCmd2;

typedef struct
{
    int32_t s32MotorPosFb;
    TurnStaWord_REG u16StaWord;
}RegDef_AGVTurnFb;
typedef struct
{
    uint8_t u8SwCmd;
    uint8_t u8Data[7];
}RegDef_AGVChargeSWCmd;

typedef struct
{
    uint8_t u8SwStaFb;
    uint8_t u8Data[7];
}RegDef_AGVChargeSWFb;

typedef struct
{
    uint8_t u8WalkCmd;         
    uint8_t u8TurnCmd;                    //
    uint8_t u8LiftCmd;                    //
    uint8_t u8RotatingCmd;
    uint8_t u8TelescopicCmd;            //
    uint8_t u8HuggingCmd;               //
    uint8_t u8GlandCmd;                  //  
    uint8_t u8EmergencyStopCmd;
}RegDef_RCCtrlCmdFb;

typedef struct
{
    uint8_t FaultCode;
    uint8_t res1;
    uint8_t res2;
    uint8_t res3;
    uint8_t res4;
    uint8_t res5;
    uint8_t res6;
    uint8_t res7;
}RegDef_RCWarningFb;

extern  RegDef_RCCtrlCmdFb Reg_RCCtrlCmdFb;
extern  RegDef_RCWarningFb Reg_RCWarningFb;

extern RegDef_AGVChargeSWFb Reg_AGVChargeSWFb;
extern RegDef_AGVPumpCtrFb Reg_AGVPumpCtrFb;
extern RegDef_Reg_AGVTurnCtrlCmd1 Reg_AGVTurnCtrlCmd1;
extern RegDef_AGVTurnCtrlCmd2 Reg_AGVTurnCtrlCmd2;
extern RegDef_AGVChargeSWCmd Reg_AGVChargeSWCmd;
extern uint8_t AGVTurnCtrlCmdSendFlag1, AGVTurnCtrlCmdSendFlag2;
extern RegDef_AGVTurnFb Reg_AGVTurnFb;
/**/
extern CanTransferTypeDef CanTransferStruct;    //CAN���丨���ֶ�
extern RegDef_HARTCMD RegReal_HARTCMD;                //�Ĵ���0x1A3���ֲ���ָ��
extern int16_t VCULife;
void Can_VCU_Receive ( void );
void CanTaskProcess ( void );
void Hart_SolenoidCmd ( void);     //ң��Һѹ������ָ��
void Hart_SteerCmd ( void);         //ң��ת��ָ���ȡ
int int32DataSat(int data,int Max,int Min);
void Hart_SolenoidCmdInit ( void);     //ң��Һѹ������ָ���ʼ��

#endif

    






