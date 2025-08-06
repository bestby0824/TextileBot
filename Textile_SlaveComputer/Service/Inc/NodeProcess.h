/**
* ��Ȩ����(C)
*
* ********
*
* @file
* @brief
* @details
* @author MUNIU
* @version 1.0.0
* @date xxxx-xx-xx
* @par Description:
* @par Change History:
*
* <����> | <�汾> | <����> | <����>
*
* xxxx/xx/xx | 1.0.0 | MUNIU | �����ļ�
*
*/
//------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
#ifndef _NodeProcess_H_
#define _NodeProcess_H_

#include "stdint.h"
#include "AngleMath.h"

#define CMD_FIFO_SIZE                512
#define Reg_ValveLiftCmd             0x100          //����ָ��  --Һѹ��   
//#define Reg_ValveClawCmd             0x110          //����ָ��
#define Reg_ValveBowRiseCmd          0x102          //����ָ��
#define Reg_ValveRotateCmd           0x103          //��תָ��
#define Reg_ValveSideShiftCmd        0x104          //����ָ��
#define Reg_ValveLiftRaderCmd        0x105          //��������ָ��
#define Reg_ValveSensorCaliCmd       0x106          //�������궨ָ��
#define Reg_ValveFaultClearCmd       0x107          //�������ָ��
#define Reg_ValveFaultEnableCmd      0x108          //��������ָ��
#define Reg_ValveCtrlState           0x109          //��λ��״̬����

#define Reg_SteerPosCtrlCmd          0x180          //ת�����  --ת���
#define Reg_SteerSensorCaliCmd       0x181          //�������궨ָ��
#define Reg_SteerFaultClearCmd       0x182          //�������ָ��
#define Reg_SteerFaultEnableCmd      0x183          //��������ָ��
#define Reg_SteerCtrlState           0x184          //��λ��״̬����
#define Reg_LightCtrlState           0x101          //�ƹ����


#define Reg_BootModeFB               0x40
#define Reg_IMUState                 0x50           //IMU

//#define Reg_ValveSensorPos1FB        0x200          //Һѹ�屧��״̬����1
//#define Reg_ValveSensorPos2FB        0x201          //Һѹ�屧��״̬����2
//#define Reg_ValveCmd1SNFB            0x202          //Һѹ�屧��SN����1
//#define Reg_ValveCmd2SNFB            0x203          //Һѹ�屧��SN����2
//#define Reg_ValveCtrlMode1FB         0x204          //Һѹ�����ģʽ����1
//#define Reg_ValveCtrlMode2FB         0x205          //Һѹ�����ģʽ����2
//#define Reg_ValveState1FB            0x206          //Һѹ����ƽ������1
//#define Reg_ValvePressureFB          0x207          //Һѹ����ƽ������2
//#define Reg_ValveFaultCodeFB         0x208          //Һѹ����Ϸ���
//#define Reg_ValveCaliStateFB         0x209          //Һѹ��궨�������
//#define Reg_ValveState2FB            0x20A          //Һѹ������������

//#define Reg_SteerPosCtrlFB           0x300          //ת����Ʒ���1
//#define Reg_SteerSensorDataFB        0x305          //ת����Ʒ���2
//#define Reg_SteerFaultCodeFB         0x302          //ת����ƹ��Ϸ���
//#define Reg_SteerSensorCaliFB        0x303          //ת����ƴ������궨����
//#define Reg_SteerFaultCtrlFB         0x304          //�������/���η���

#define Reg_RadarSend                0x60          //�״�����ѡ��
#define Reg_RadarRev                 0x61
extern int16_t  RadarLife, DrawWireLife;
/*Textiles*/
/*����*/
#define Reg_RotatingMBDCmd                          0x110
#define Reg_TelescopicMBDCmd                        0x120
#define Reg_HuggingMBDCmd                           0x130
#define Reg_GlandMBDCmd                             0x140
/*���궨*/
#define Reg_RotatingMBDCalaCmd                      0x111
#define Reg_TelescopicMBDCalaCmd                    0x121
#define Reg_HuggingMBDCalaCmd                       0x131
#define Reg_GlandMBDCalaCmd                         0x141
/*�������*/
#define Reg_RotatingMBDFaultClearCmd                0x112
#define Reg_TelescopicMBDFaultClearCmd              0x122
#define Reg_HuggingMBDFaultClearCmd                 0x132
#define Reg_GlandMBDFaultClearCmd                   0x142
/*��������*/
#define Reg_RotatingMBDParamCmd                     0x113
#define Reg_TelescopicMBDParamCmd                   0x123
#define Reg_HuggingMBDParamCmd                      0x133
#define Reg_GlandMBDParamCmd                        0x143
/*����*/
#define Reg_RotatingMBDWatchDogCmd                  0x114
#define Reg_TelescopicMBDWatchDogCmd                0x124
#define Reg_HuggingMBDWatchDogCmd                   0x134
#define Reg_GlandMBDWatchDogCmd                     0x144
/*��λ*/
#define Reg_RotatingMBDResetCmd                     0x115
#define Reg_TelescopicMBDResetCmd                   0x125
#define Reg_HuggingMBDResetCmd                      0x135
#define Reg_GlandMBDResetCmd                        0x145

/*����ack*/
#define Reg_RotatingMBDCtrlFB                       0x200
#define Reg_TelescopicMBDCtrlFB                     0x300
#define Reg_HuggingMBDCtrlFB                        0x400
#define Reg_GlandMBDCtrlFB                          0x500
/*���궨ack*/
#define Reg_RotatingMBDCalaFB                       0x201
#define Reg_TelescopicMBDCalaFB                     0x301
#define Reg_HuggingMBDCalaFB                        0x401
#define Reg_GlandMBDCalaFB                          0x501
/*�������ack*/
#define Reg_RotatingMBDFaultClearFB                 0x202
#define Reg_TelescopicMBDFaultClearFB               0x302
#define Reg_HuggingMBDFaultClearFB                  0x402
#define Reg_GlandMBDFaultClearFB                    0x502
/*��������ack*/
#define Reg_RotatingMBDParamFB                      0x203
#define Reg_TelescopicMBDParamFB                    0x303
#define Reg_HuggingMBDParamFB                       0x403
#define Reg_GlandMBDParamFB                         0x503
/*����*/
#define Reg_RotatingMBDWatchDogFB                   0x204
#define Reg_TelescopicMBDWatchDogFB                 0x304
#define Reg_HuggingMBDWatchDogFB                    0x404
#define Reg_GlandMBDWatchDogFB                      0x504
/*��λ*/
#define Reg_RotatingMBDResetFB                      0x205
#define Reg_TelescopicMBDResetFB                    0x305
#define Reg_HuggingMBDResetFB                       0x405
#define Reg_GlandMBDResetFB                         0x505
/*����״̬*/
#define Reg_RotatingMBDFaultStaFB                   0x210
#define Reg_TelescopicMBDFaultStaFB                0x310
#define Reg_HuggingMBDFaultStaFB                    0x410
#define Reg_GlandMBDFaultStaFB                      0x510
/*������״̬*/
#define Reg_RotatingMBDSensorStaFB                  0x211
#define Reg_TelescopicMBDSensorStaFB               0x311
#define Reg_HuggingMBDSensorStaFB                   0x411
#define Reg_GlandMBDSensorStaFB                     0x511
/*����״̬1*/
#define Reg_RotatingMBDRunningSta1FB                0x212
#define Reg_TelescopicMBDRunningSta1FB             0x312
#define Reg_HuggingMBDRunningSta1FB                 0x412
#define Reg_GlandMBDRunningSta1FB                   0x512
/*����״̬2*/
#define Reg_RotatingMBDRunningSta2FB                0x213
#define Reg_TelescopicMBDRunningSta2FB             0x313
#define Reg_HuggingMBDRunningSta2FB                 0x413
#define Reg_GlandMBDRunningSta2FB                   0x513
/*����״̬3*/
#define Reg_RotatingMBDRunningSta3FB                0x214
#define Reg_TelescopicMBDRunningSta3FB             0x314
#define Reg_HuggingMBDRunningSta3FB                 0x414
#define Reg_GlandMBDRunningSta3FB                   0x514

#define Step_DrawWireSensor                     0x08
#define Step_TelescopicMBDRelayCmd                  0x126
#define Step_TelescopicMBDPushMotorCmd              0x127
#define Step_TelescopicMBDRelayCmdFB                0x306
typedef enum
{
    Para_READ = 0,
    Para_WRITE,
} E_PAPAM_OPT;
/*����*/
typedef struct//0x60
{  
   uint8_t u8RadarSw;
   uint8_t u8WorkDir;
   int16_t s16TurnDeg;
   uint8_t Res[4];
}RegDef_RadarSend;
extern RegDef_RadarSend  RegReal_RadarSend;
typedef struct//0x61
{  
   uint8_t u8LeftRadarInf;
   uint8_t u8RightRadarInf;
   uint8_t u8RadarInf;
   uint8_t Res[5];
}RegDef_RadarRev;
extern RegDef_RadarRev RegReal_RadarRev;
typedef struct//0x126
{
   uint16_t u16EN;
   uint16_t u16SN;
   uint16_t u16Cmd;
   uint16_t Res;
}RegDef_TelescopicMBDRelayCmd;
typedef struct//0x306
{
   uint16_t u16SN;      //Sn
   uint16_t u16Sta;     //�˶�״̬
   uint16_t u16RelayCmdFb;   //���״̬
   uint16_t u16TouchSwSta;//�Ӵ�����״̬
}RegDef_TelescopicMBDRelayCmdFB;
extern RegDef_TelescopicMBDRelayCmd RegReal_TelescopicMBDRelayCmd;
extern RegDef_TelescopicMBDRelayCmdFB RegReal_TelescopicMBDRelayCmdFB;
/*����*/
typedef struct//0x110
{
    uint16_t u16Cmd;
    uint16_t u16SN;
    uint16_t u16EN;
    uint16_t u16Ref;
} RegDef_MotorMBDCmd;
extern RegDef_MotorMBDCmd RegReal_RotatingMBDCmd;
extern RegDef_MotorMBDCmd RegReal_TelescopicMBDCmd;//0x120
extern RegDef_MotorMBDCmd RegReal_HuggingMBDCmd;//0x130
extern RegDef_MotorMBDCmd RegReal_GlandMBDCmd;//0x140
//typedef struct//0x120
//{
//    uint16_t u16Cmd;
//    uint16_t u16SN;
//    uint16_t u16EN;
//    uint16_t u16Ref;
//} RegDef_TelescopicMBDCmd;

//typedef struct
//{
//   uint16_t u16Cmd;
//    uint16_t u16SN;
//    uint16_t u16EN;
//    uint16_t u16Ref;
//} RegDef_HuggingMBDCmd;//0x130

//typedef struct
//{
//   uint16_t u16Cmd;
//    uint16_t u16SN;
//    uint16_t u16EN;
//    uint16_t u16Ref;
//} RegDef_GlandMBDCmd;//0x140
/*�궨*/
typedef struct//0x111
{
    uint16_t u16Cmd;
    uint16_t u16SN;
    uint16_t u16EN;
    uint16_t u16Ref;
} RegDef_MotorMBDCalaCmd;
//typedef struct//0x121
//{
//    uint16_t u16Cmd;
//    uint16_t u16SN;
//    uint16_t u16EN;
//    uint16_t u16Ref;
//}RegDef_TelescopicMBDCalaCmd;

//typedef struct//0x131
//{
//    uint16_t u16Cmd;
//    uint16_t u16SN;
//    uint16_t u16EN;
//    uint16_t u16Ref;
//} RegDef_HuggingMBDCalaCmd;

//typedef struct//0x141
//{
//    uint16_t u16Cmd;
//    uint16_t u16SN;
//    uint16_t u16EN;
//    uint16_t u16Ref;
//} RegDef_GlandMBDCalaCmd;
typedef struct//0x08
{
    uint8_t u8DataLen;
    uint8_t u8Id;
    uint8_t u8Cmd;
    uint8_t u8Databuf[5];
}RegDef_DrawWireSensor;
extern RegDef_DrawWireSensor Reg_DrawWireSensorFb;
typedef struct//0x112
{
    uint16_t u16EN;
    uint16_t u16SN;  
    uint32_t u32FaultBit;
} RegDef_MotorMBDFaultClearCmd;
//extern RegDef_RotatingFaultClearCmd  Reg_RotatingFaultClearCmd ;
//typedef struct//0x122
//{
//    uint16_t u16EN;
//    uint16_t u16SN;  
//    uint32_t u32FaultBit;
//} RegDef_TelescopicMBDFaultClearCmd;

//typedef struct//0x132
//{
//    uint16_t u16EN;
//    uint16_t u16SN;  
//    uint32_t u32FaultBit;
//} RegDef_HuggingMBDFaultClearCmd;

//typedef struct//0x142
//{
//    uint16_t u16EN;
//    uint16_t u16SN;  
//    uint32_t u32FaultBit;
//} RegDef_GlandMBDFaultClearCmd;

typedef struct//0x113
{
    uint16_t u16Addr;                       //�Ĵ�����ַ
    uint16_t u16SN;                         //ָ��SN
    E_PAPAM_OPT mod;                        //0-����1-д�����ŷ�״̬��Ч��
    uint16_t u16Value;                      //�Ĵ���ֵ
} RegDef_MotorMBDParamCmd;
//extern RegDef_RotatingParamCmd Reg_RotatingParamCmd;
//typedef struct//0x123
//{
//    uint16_t u16Addr;                       //�Ĵ�����ַ
//    uint16_t u16SN;                         //ָ��SN
//    E_PAPAM_OPT mod;                        //0-����1-д�����ŷ�״̬��Ч��
//    uint16_t u16Value;                      //�Ĵ���ֵ
//} RegDef_TelescopicMBDParamCmd;

//typedef struct//0x133
//{
//    uint16_t u16Addr;                       //�Ĵ�����ַ
//    uint16_t u16SN;                         //ָ��SN
//    E_PAPAM_OPT mod;                        //0-����1-д�����ŷ�״̬��Ч��
//    uint16_t u16Value;                      //�Ĵ���ֵ
//} RegDef_HuggingMBDParamCmd;

//typedef struct//0x143
//{
//    uint16_t u16Addr;                       //�Ĵ�����ַ
//    uint16_t u16SN;                         //ָ��SN
//    E_PAPAM_OPT mod;                        //0-����1-д�����ŷ�״̬��Ч��
//    uint16_t u16Value;                      //�Ĵ���ֵ
//} RegDef_GlandMBDParamCmd;

typedef struct//0x114
{
    uint16_t u16SN;                         //ָ��SN
    uint16_t res1;                          //Ԥ��
    uint16_t res2;                          //Ԥ��
    uint16_t res3;                          //Ԥ��
} RegDef_MotorMBDWatchDogCmd;

//extern RegDef_RotatingWatchDogCmd  Reg_RotatingWatchDogCmd ;
//typedef struct//0x124
//{
//    uint16_t u16SN;                         //ָ��SN
//    uint16_t res1;                          //Ԥ��
//    uint16_t res2;                          //Ԥ��
//    uint16_t res3;                          //Ԥ��
//} RegDef_TelescopicMBDWatchDogCmd;

//typedef struct//0x134
//{
//    uint16_t u16SN;                         //ָ��SN
//    uint16_t res1;                          //Ԥ��
//    uint16_t res2;                          //Ԥ��
//    uint16_t res3;                          //Ԥ��
//} RegDef_HuggingMBDWatchDogCmd;

//typedef struct//0x144
//{
//    uint16_t u16SN;                         //ָ��SN
//    uint16_t res1;                          //Ԥ��
//    uint16_t res2;                          //Ԥ��
//    uint16_t res3;                          //Ԥ��
//} RegDef_GlandMBDWatchDogCmd;

typedef struct//0x115
{
    uint16_t u16Delay;                      //��ʱ��λ����λmS
    uint16_t u16SN;                         //ָ��SN
    uint16_t u16Res1;                       //Ԥ��
    uint16_t u16Res2;                       //Ԥ��
} RegDef_MotorMBDResetCmd;
//extern RegDef_RotatingReset Reg_RotatingReset;
//typedef struct//0x125
//{
//    uint16_t u16Delay;                      //��ʱ��λ����λmS
//    uint16_t u16SN;                         //ָ��SN
//    uint16_t u16Res1;                       //Ԥ��
//    uint16_t u16Res2;                       //Ԥ��
//} RegDef_TelescopicMBDResetCmd;

//typedef struct//0x135
//{
//    uint16_t u16Delay;                      //��ʱ��λ����λmS
//    uint16_t u16SN;                         //ָ��SN
//    uint16_t u16Res1;                       //Ԥ��
//    uint16_t u16Res2;                       //Ԥ��
//} RegDef_HuggingMBDResetCmd;

//typedef struct//0x145
//{
//    uint16_t u16Delay;                      //��ʱ��λ����λmS
//    uint16_t u16SN;                         //ָ��SN
//    uint16_t u16Res1;                       //Ԥ��
//    uint16_t u16Res2;                       //Ԥ��
//} RegDef_GlandMBDResetCmd;

typedef struct//0x200
{
    uint16_t u16CmdFB;
    uint16_t u16SN;
    uint16_t u16Ack;
    uint16_t u16Sta;
} RegDef_RotatingMBDCtrlFB;
extern RegDef_RotatingMBDCtrlFB RegReal_RotatingMBDCtrlFB;
typedef struct//0x300
{
    uint16_t u16CmdFB;
    uint16_t u16SN;
    uint16_t u16Ack;
    uint16_t u16Sta;
} RegDef_TelescopicMBDCtrlFB;
extern RegDef_TelescopicMBDCtrlFB RegReal_TelescopicMBDCtrlFB;
typedef struct//0x400
{
    uint16_t u16CmdFB;
    uint16_t u16SN;
    uint16_t u16Ack;
    uint16_t u16Sta;
} RegDef_HuggingMBDCtrlFB;
extern RegDef_HuggingMBDCtrlFB RegReal_HuggingMBDCtrlFB;
typedef struct//0x500
{
    uint16_t u16CmdFB;
    uint16_t u16SN;
    uint16_t u16Ack;
    uint16_t u16Sta;
} RegDef_GlandMBDCtrlFB;
extern RegDef_GlandMBDCtrlFB RegReal_GlandMBDCtrlFB;
typedef struct//0x201
{
    uint16_t u16CmdFB;
    uint16_t u16SN;
    uint16_t u16Sta;
    uint16_t res1;
} RegDef_RotatingMBDCalaFB;
//extern RegDef_RotatingMBDCalaFB Reg_RotatingMBDCalaFB;
typedef struct//0x301
{
    uint16_t u16CmdFB;
    uint16_t u16SN;
    uint16_t u16Sta;
    uint16_t res1;
} RegDef_TelescopicMBDCalaFB;

typedef struct//0x401
{
    uint16_t u16CmdFB;
    uint16_t u16SN;
    uint16_t u16Sta;
    uint16_t res1;
} RegDef_HuggingMBDCalaFB;

typedef struct//0x501
{
    uint16_t u16CmdFB;
    uint16_t u16SN;
    uint16_t u16Sta;
    uint16_t res1;
} RegDef_GlandMBDCalaFB;

typedef struct//0x202
{
    uint16_t u16CmdFB;
    uint16_t u16SN;
    uint32_t u32FaultBitFB;
} RegDef_RotatingMBDFaultClearFB;
//extern RegDef_RotatingMBDFaultClearFB  Reg_RotatingMBDFaultClearFB ;
typedef struct//0x302
{
    uint16_t u16CmdFB;
    uint16_t u16SN;
    uint32_t u32FaultBitFB;
} RegDef_TelescopicMBDFaultClearFB;

typedef struct//0x402
{
    uint16_t u16CmdFB;
    uint16_t u16SN;
    uint32_t u32FaultBitFB;
} RegDef_HuggingMBDFaultClearFB;

typedef struct//0x502
{
    uint16_t u16CmdFB;
    uint16_t u16SN;
    uint32_t u32FaultBitFB;
} RegDef_GlandMBDFaultClearFB;

typedef struct//0x203
{
    uint16_t u16Addr;                       //�Ĵ�����ַ
    uint16_t u16SN;                         //ָ��SN
    E_PAPAM_OPT mod;                        //0-����1-д�����ŷ�״̬��Ч��
    uint16_t u16Value;                      //�Ĵ���ֵ
} RegDef_RotatingMBDParamFB;
//extern RegDef_RotatingParamFB Reg_RotatingParamFB;
typedef struct//0x303
{
    uint16_t u16Addr;                       //�Ĵ�����ַ
    uint16_t u16SN;                         //ָ��SN
    E_PAPAM_OPT mod;                        //0-����1-д�����ŷ�״̬��Ч��
    uint16_t u16Value;                      //�Ĵ���ֵ
} RegDef_TelescopicMBDParamFB;

typedef struct//0x403
{
    uint16_t u16Addr;                       //�Ĵ�����ַ
    uint16_t u16SN;                         //ָ��SN
    E_PAPAM_OPT mod;                        //0-����1-д�����ŷ�״̬��Ч��
    uint16_t u16Value;                      //�Ĵ���ֵ
} RegDef_HuggingMBDParamFB;

typedef struct//0x503
{
    uint16_t u16Addr;                       //�Ĵ�����ַ
    uint16_t u16SN;                         //ָ��SN
    E_PAPAM_OPT mod;                        //0-����1-д�����ŷ�״̬��Ч��
    uint16_t u16Value;                      //�Ĵ���ֵ
} RegDef_GlandMBDParamFB;

typedef struct//0x204
{
    uint16_t u16SN;                         //ָ��SN
    uint16_t res1;                          //Ԥ��
    uint16_t res2;                          //Ԥ��
    uint16_t res3;                          //Ԥ��
} RegDef_RotatingMBDWatchDogFB;
//extern RegDef_RotatingWatchDogFB Reg_RotatingWatchDogFB;
typedef struct//0x304
{
    uint16_t u16SN;                         //ָ��SN
    uint16_t res1;                          //Ԥ��
    uint16_t res2;                          //Ԥ��
    uint16_t res3;                          //Ԥ��
} RegDef_TelescopicMBDWatchDogFB;

typedef struct//0x404
{
    uint16_t u16SN;                         //ָ��SN
    uint16_t res1;                          //Ԥ��
    uint16_t res2;                          //Ԥ��
    uint16_t res3;                          //Ԥ��
} RegDef_HuggingMBDWatchDogFB;

typedef struct//0x504
{
    uint16_t u16SN;                         //ָ��SN
    uint16_t res1;                          //Ԥ��
    uint16_t res2;                          //Ԥ��
    uint16_t res3;                          //Ԥ��
} RegDef_GlandMBDWatchDogFB;

typedef struct//0x205
{
    uint16_t u16Delay;                      //��ʱ��λ����λmS
    uint16_t u16SN;                         //ָ��SN
    uint16_t u16Res1;                       //Ԥ��
    uint16_t u16Res2;                       //Ԥ��
} RegDef_RotatingMBDResetFB;
//extern RegDef_RotatingReset Reg_RotatingReset;
typedef struct//0x305
{
    uint16_t u16Delay;                      //��ʱ��λ����λmS
    uint16_t u16SN;                         //ָ��SN
    uint16_t u16Res1;                       //Ԥ��
    uint16_t u16Res2;                       //Ԥ��
} RegDef_TelescopicMBDResetFB;

typedef struct//0x405
{
    uint16_t u16Delay;                      //��ʱ��λ����λmS
    uint16_t u16SN;                         //ָ��SN
    uint16_t u16Res1;                       //Ԥ��
    uint16_t u16Res2;                       //Ԥ��
} RegDef_HuggingMBDResetFB;

typedef struct//0x505
{
    uint16_t u16Delay;                      //��ʱ��λ����λmS
    uint16_t u16SN;                         //ָ��SN
    uint16_t u16Res1;                       //Ԥ��
    uint16_t u16Res2;                       //Ԥ��
} RegDef_GlandMBDResetFB;

typedef struct//0x210
{
    uint32_t u32Fault;                    //������
    uint32_t u32Warning;                  //������
} RegDef_RotatingMBDFaultStaFB;
//extern RegDef_RotatingFaultReset Reg_RotatingFaultReset;
typedef struct//0x310
{
    uint32_t u32Fault;                    //������
    uint32_t u32Warning;                  //������
} RegDef_TelescopicMBDFaultStaFB;

typedef struct//0x410
{
    uint32_t u32Fault;                    //������
    uint32_t u32Warning;                  //������
} RegDef_HuggingMBDFaultStaFB;

typedef struct//0x510
{
    uint32_t u32Fault;                    //������
    uint32_t u32Warning;                  //������
} RegDef_GlandMBDFaultStaFB;
extern RegDef_RotatingMBDFaultStaFB RegReal_RotatingMBDFaultStaFB;//0x210
extern RegDef_TelescopicMBDFaultStaFB RegReal_TelescopicMBDFaultStaFB;//0x310
extern RegDef_HuggingMBDFaultStaFB RegReal_HuggingMBDFaultStaFB;//0x410
extern RegDef_GlandMBDFaultStaFB RegReal_GlandMBDFaultStaFB;//0x510
typedef struct//0x211
{
    uint16_t u16SensorSta1;
    uint16_t u16SensorSta2;
    uint16_t u16SensorSta3;
    uint16_t u16SensorSta4;
} RegDef_RotatingMBDSensorStaFB;
//extern RegDef_RotatingMBDSensorStaFB Reg_RotatingMBDSensorStaFB;
typedef struct//0x311
{
    uint16_t u16SensorSta1;
    uint16_t u16SensorSta2;
    uint16_t u16SensorSta3;
    uint16_t u16SensorSta4;
} RegDef_TelescopicMBDSensorStaFB;

typedef struct//0x411
{
    uint16_t u16SensorSta1;
    uint16_t u16SensorSta2;
    uint16_t u16SensorSta3;
    uint16_t u16SensorSta4;
} RegDef_HuggingMBDSensorStaFB;

typedef struct//0x511
{
    uint16_t u16SensorSta1;
    uint16_t u16SensorSta2;
    uint16_t u16SensorSta3;
    uint16_t u16SensorSta4;
} RegDef_GlandMBDSensorStaFB;

typedef struct//0x212
{
    uint16_t u16RunningSta1;
    uint16_t u16RunningSta2;
    uint16_t u16RunningSta3;
    uint16_t u16RunningSta4;
} RegDef_RotatingMBDRunningSta1FB;
//extern RegDef_RotatingMBDRunningSta1FB Reg_RotatingMBDRunningSta1FB;
typedef struct//0x312
{
    uint16_t u16RunningSta1;
    uint16_t u16RunningSta2;
    uint16_t u16RunningSta3;
    uint16_t u16RunningSta4;
} RegDef_TelescopicMBDRunningSta1FB;

typedef struct//0x412
{
    uint16_t u16RunningSta1;
    uint16_t u16RunningSta2;
    uint16_t u16RunningSta3;
    uint16_t u16RunningSta4;
} RegDef_HuggingMBDRunningSta1FB;

typedef struct//0x512
{
    uint16_t u16RunningSta1;
    uint16_t u16RunningSta2;
    uint16_t u16RunningSta3;
    uint16_t u16RunningSta4;
} RegDef_GlandMBDRunningSta1FB;
extern int16_t m_nRotatingMBLife, m_nTelescopicMBLife, m_nHuggingMBLife, m_nGlandMBLife;
extern int8_t ResetCheck ( void );
extern void ParamRead_Write ( void );
typedef enum{
    ValveLiftCmd = 0,
    ValveClawCmd,
    ValveBowRiseCmd,
    ValveRotateCmd,
    ValveSideShiftCmd,
    ValveLiftRaderCmd,
    ValveSensorCaliCmd,
    ValveFaultClearCmd,
    ValveFaultEnableCmd,
    ValveCtrlState,
    ValveCmdNum,
    SteerPosCtrlCmd,
    SteerSensorCaliCmd,
    SteerFaultClearCmd,
    SteerFaultEnableCmd,
    SteerCtrlState,
    SteerCmdNum,
    LightCmd,
    RaderCmd,
    StepS_None,
}CanTask2_StepS;

typedef struct
{
    uint32_t ID;
    uint32_t Len;
    uint8_t Buff[CMD_FIFO_SIZE];
} UserFrame2_t;

typedef struct {
    uint16_t u16Dev_ID;     //·�ɱ�ֵ
    uint16_t Opt;
    uint16_t u16Cnt;        //����
    uint16_t u16Ack;        //����
} BootRxStruct;
extern BootRxStruct BootRxHandle;

typedef struct
{
    uint16_t u16SN;                    //������ţ�ack��Ӧʹ��
    uint16_t unEN;                       //ʹ��
    uint16_t unLiftPos;                  //Ŀ��Lift�߶� 32767 = 6m
    uint16_t LiftMode;                   //����ģʽ��0��λ���ŷ���1���½�����
    
} RegDef_ValveLiftCmd;
extern RegDef_ValveLiftCmd RegReal_ValveLiftCmd;  //�Ĵ���0x100

typedef struct
{
    uint16_t u16SN;                    //������ţ�ack��Ӧʹ��
    uint16_t unEN;                       //ʹ��
    uint16_t unClawPos;                  //Ŀ��Clawλ��
    uint16_t unRes;                      //Ԥ��
} RegDef_ValveClawCmd;
extern RegDef_ValveClawCmd RegReal_ValveClawCmd;  //�Ĵ���0x101

typedef struct
{
    uint16_t u16SN;                    //������ţ�ack��Ӧʹ��
    uint16_t unEN;                       //ʹ��
    uint16_t unBowRisePos;               //Ŀ��λ��
    uint16_t unRes;                      //Ԥ��
} RegDef_ValveBowRiseCmd;
extern RegDef_ValveBowRiseCmd RegReal_ValveBowRiseCmd;  //�Ĵ���0x102

typedef struct
{
    uint16_t u16SN;                    //������ţ�ack��Ӧʹ��
    uint16_t unEN;                       //ʹ��
    uint16_t unRotatePos;                  //Ŀ��Rotateλ��
    uint16_t unRes;                      //Ԥ��
} RegDef_ValveRotateCmd;
extern RegDef_ValveRotateCmd RegReal_ValveRotateCmd;  //�Ĵ���0x103

typedef struct
{
    uint16_t u16SN;                    //������ţ�ack��Ӧʹ��
    uint16_t unEN;                       //ʹ��
    uint16_t unSideShiftPos;             //Ŀ��SideShiftλ��
    uint16_t SideMoveMode;                      //Ԥ��
} RegDef_ValveSideShiftCmd;                 //104�Ĵ���
extern RegDef_ValveSideShiftCmd RegReal_ValveSideShiftCmd;  //�Ĵ���0x104

typedef struct
{
    uint16_t u16SN;                    //������ţ�ack��Ӧʹ��
    uint16_t unEN;                       //ʹ��
    uint16_t unLiftRaderPos;             //Ŀ��LiftRaderλ��
    uint16_t unRes;                      //Ԥ��
} RegDef_ValveLiftRaderCmd;                 //105�Ĵ���
extern RegDef_ValveLiftRaderCmd RegReal_ValveLiftRaderCmd;  //�Ĵ���0x105

typedef struct
{
    uint16_t u16MotorCoderCaliSN;          //������̱궨SN
    uint16_t u16MotorCoderCaliCmd;         //������̱궨Cmd
    uint16_t u16WheelCoderCaliSN;          //�������̱궨SN
    uint16_t u16WheelCoderCaliCmd;         //�������̱궨Cmd
} RegDef_ValveSensorCaliCmd;
extern RegDef_ValveSensorCaliCmd RegReal_ValveSensorCaliCmd;  //�Ĵ���0x106

typedef struct
{
    uint16_t u16FaultClearCmd;           //������̱궨
    uint16_t u16Res1;                    //Ԥ��
    uint16_t u16Res2;                    //Ԥ��
    uint16_t u16Res3;                    //Ԥ��
} RegDef_ValveFaultClearCmd;
extern RegDef_ValveFaultClearCmd RegReal_ValveFaultClearCmd;  //�Ĵ���0x107

typedef struct
{
    uint16_t u16FaultClearCmd;           //������̱궨
    uint16_t u16Res1;                    //Ԥ��
    uint16_t u16Res2;                    //Ԥ��
    uint16_t u16Res3;                    //Ԥ��
} RegDef_ValveFaultEnableCmd;
extern RegDef_ValveFaultEnableCmd RegReal_ValveFaultEnableCmd;  //�Ĵ���0x108

typedef struct
{
    uint16_t u16CtrlMode;                //����ģʽ
    uint16_t u16Res1;                    //Ԥ��
    uint16_t u16Res2;                    //Ԥ��
    uint16_t u16Res3;                    //Ԥ��
} RegDef_ValveCtrlState;
extern RegDef_ValveCtrlState RegReal_ValveCtrlState;  //�Ĵ���0x109

typedef struct
{
    uint16_t u16Mode;                     //λ�ÿ���ģʽ
    uint16_t u16SN;                       //λ�ÿ���SN
    uint16_t u16EN;                       //λ�ÿ���ʹ��
    uint16_t u16PosRef;                   //λ�ø���ֵ
} RegDef_SteerPosCtrlCmd;
extern RegDef_SteerPosCtrlCmd RegReal_SteerPosCtrlCmd;  //�Ĵ���0x180

typedef struct
{
    uint16_t u16MotorCoderCaliSN;          //������̱궨SN
    uint16_t u16MotorCoderCaliCmd;         //������̱궨Cmd
    uint16_t u16WheelCoderCaliSN;          //�������̱궨SN
    uint16_t u16WheelCoderCaliCmd;         //�������̱궨Cmd
} RegDef_SteerSensorCaliCmd;
extern RegDef_SteerSensorCaliCmd RegReal_SteerSensorCaliCmd;  //�Ĵ���0x181

typedef struct
{
    uint16_t u16FaultClearCmd;           //������̱궨
    uint16_t u16Res1;                    //�������̱궨
    uint16_t u16Res2;                    //�������̱궨
    uint16_t u16Res3;                    //Ԥ��
} RegDef_SteerFaultClearCmd;
extern RegDef_SteerFaultClearCmd RegReal_SteerFaultClearCmd;  //�Ĵ���0x182

typedef struct
{
    uint16_t u16FaultClearCmd;           //������̱궨
    uint16_t u16CtrlMode;                //�������̱궨
    uint16_t u16Res2;                    //�������̱궨
    uint16_t u16Res3;                    //Ԥ��
} RegDef_SteerFaultEnableCmd;
extern RegDef_SteerFaultEnableCmd RegReal_SteerFaultEnableCmd;  //�Ĵ���0x183

typedef struct
{
    uint16_t u16CtrlMode;                //����ģʽ
    uint16_t u16Res1;                    //Ԥ��
    uint16_t u16Res2;                    //Ԥ��
    uint16_t u16Res3;                    //Ԥ��
} RegDef_SteerCtrlState;
extern RegDef_SteerCtrlState RegReal_SteerCtrlState;  //�Ĵ���0x184

typedef struct
{
    uint16_t unClampHight;               //���и߶�
    uint16_t unClampOpenDegree;          //���п���
    uint16_t unBowRiseDegree;            //����λ��
    uint16_t unRotateDegree;             //��תλ��
} RegDef_ValveSensorPos1FB;              //200�Ĵ���
extern RegDef_ValveSensorPos1FB RegReal_ValveSensorPos1FB;

typedef struct
{
    uint16_t unSideShiftDegree;          //����λ��
    uint16_t unLiftMotorHight;           //�ŷ�����λ��
    uint16_t res1;                       //Ԥ��
    uint16_t res2;                       //Ԥ��
} RegDef_ValveSensorPos2FB;              //201�Ĵ���

typedef struct
{
    uint16_t unLiftAck;                  //����SN
    uint16_t unClawAck;                  //����SN
    uint16_t unBowRiseAck;               //����SN
    uint16_t unRotateAck;                //��תSN
} RegDef_ValveCmd1SNFB;                  //202�Ĵ���

typedef struct
{
    uint16_t unSideShiftAck;             //����SN
    uint16_t unLiftMotorAck;             //�ŷ�����SN
    uint16_t res1;                       //Ԥ��
    uint16_t res2;                       //Ԥ��
} RegDef_ValveCmd2SNFB;                  //203�Ĵ���

typedef struct
{
    uint16_t unLiftCtrlMode;             //��������ģʽ
    uint16_t unClawCtrlMode;             //���Ͽ���ģʽ
    uint16_t unBowRiseCtrlMode;          //��������ģʽ
    uint16_t unRotateCtrlMode;           //��ת����ģʽ
} RegDef_ValveCtrlMode1FB;               //204�Ĵ���

typedef struct
{
    uint16_t unSideShiftCtrlMode;        //���ƿ���ģʽ
    uint16_t unLiftMotorCtrlMode;        //�ŷ���������ģʽ
    uint16_t res1;                       //Ԥ��
    uint16_t res2;                       //Ԥ��
} RegDef_ValveCtrlMode2FB;               //205�Ĵ���

typedef struct
{
    uint16_t u16Valve1Flag;              //Һѹ����״̬1���ϴ�����λ����֪�Ѿ��յ��� bit/0~3/4~7/8~11/12~15,�ֱ�������������У���������ת
    uint16_t u16Valve2Flag;              //Һѹ����״̬2���ϴ�����λ����֪�Ѿ��յ��� bit/0~3/4~7,�ֱ������ƣ��ŷ�����
    uint16_t u16ClawState;               //���б���״̬
    uint16_t u16SideShiftState;          //����ֹͣ״̬��0:��ʼ������յ���ָ�����㣻1����λֹͣ��2�����Ƶ���ֹͣ��3����ʱֹͣ��ָ�����
} RegDef_ValveState1FB;
extern RegDef_ValveState1FB RegReal_ValveState1FB;        //�Ĵ���0x206

typedef struct
{
    uint16_t u16Lift_Pressure;           //������ѹ
    uint16_t u16Claw_Pressure;           //������ѹ
    uint16_t u16SideShift_Pressure1;     //��������ѹ
    uint16_t u16SideShift_Pressure2;     //��������ѹ
} RegDef_ValvePressureFB;                //207�Ĵ���
extern RegDef_ValvePressureFB RegReal_ValvePressureFB;

typedef struct
{
    uint32_t u32FaultNum;               //���ϱ���
    uint32_t u32WarningNum;             //�������
} RegDef_ValveFaultCodeFB;
extern RegDef_ValveFaultCodeFB RegReal_ValveFaultCodeFB;        //�Ĵ���0x208

typedef struct
{
    uint16_t u16ClawCaliState;           //�������̱궨
    uint16_t u16RotateCaliState;         //��ת���̱궨
    uint16_t res1;                       //Ԥ��
    uint16_t res2;                       //Ԥ��
} RegDef_ValveCaliStateFB;               //209�Ĵ���

typedef struct
{
    uint16_t u16FaultClear;              //�������
    uint16_t u16FaultShield;             //��������
    uint16_t u16StaFlg;                  //��λ����
    uint16_t res1;                       //Ԥ��
} RegDef_ValveState2FB;                  //20A�Ĵ���

typedef struct
{
    uint16_t u16Mode;                     //λ�ÿ���ģʽ
    uint16_t u16SN;                       //λ�ÿ���SN
    uint16_t u16PosFbk;                   //λ�÷���ֵ
    uint16_t u16EndofCtrl;                //EndofCtrl״̬
} RegDef_SteerPosCtrlFB;
extern RegDef_SteerPosCtrlFB RegReal_SteerPosCtrlFB;  //�Ĵ���0x300

typedef struct
{
    uint16_t u16LeftEncoder;              //����ֱ�����ֵ
    uint16_t u16RightEncoder;             //�Һ��ֱ�����ֵ
    uint16_t u16Pressure;                 //ת����ѹ
    uint16_t u16Res;                      //Ԥ��
} RegDef_SteerSensorDataFB;
extern RegDef_SteerSensorDataFB RegReal_SteerSensorDataFB;  //�Ĵ���0x301

typedef struct
{
    uint32_t u32Fault;                    //������
    uint32_t u32Warning;                  //������
} RegDef_SteerFaultCodeFB;
extern RegDef_SteerFaultCodeFB RegReal_SteerFaultCodeFB;  //�Ĵ���0x302

typedef struct
{
    uint16_t u16MotorCoderCaliACK;        //����ֱ�����ֵ
    uint16_t u16MotorCoderCaliRes;             //�Һ��ֱ�����ֵ
    uint16_t u16WheelCoderCaliACK;                 //ת����ѹ
    uint16_t u16WheelCoderCaliRes;                      //Ԥ��
} RegDef_SteerSensorCaliFB;
extern RegDef_SteerSensorCaliFB RegReal_SteerSensorCaliFB;  //�Ĵ���0x303

typedef struct
{
    uint16_t u16FaultClearFB;             //�������FB
    uint16_t u16FaultEnableFB;            //��������EB
    uint16_t u16ResetCnt;                 //��λ����
    uint16_t u16Res;                      //Ԥ��
} RegDef_SteerFaultCtrlFB;
extern RegDef_RotatingMBDCtrlFB RegReal_RotatingMBDCtrlFB;//0x200
extern RegDef_TelescopicMBDCtrlFB RegReal_TelescopicMBDCtrlFB;//0x300
extern RegDef_HuggingMBDCtrlFB RegReal_HuggingMBDCtrlFB;//0x400
extern RegDef_GlandMBDCtrlFB RegReal_GlandMBDCtrlFB;//0x500
extern RegDef_RotatingMBDRunningSta1FB RegReal_RotatingMBDRunningSta2FB;//0x213
extern uint8_t u8BootFB_Flag;
//-------------------- public functions -------------------------------------
void CanTask2Process ( void );
void Can_Node_Receive ( void );
int Can2Task_Send ( uint32_t Cmd_ID );
void Can_Node_Receive ( void );
int8_t MotorCalaCheck ( void );
//�������
int8_t  FualtClear ( void );
#endif // _NodeProcess_H_

//-----------------------End of file------------------------------------------
/** @}*/






