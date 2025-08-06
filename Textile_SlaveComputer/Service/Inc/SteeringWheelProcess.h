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
#ifndef _SteerWheel_Process__H_
#define _SteerWheel_Process__H_
//-------------------- include files ----------------------------------------
#include "AngleMath.h"
#include "RS485_SteeringWheel.h"
#include "MODBusCRC.h"
//-------------------- public definitions -----------------------------------
#define RS485_TxBufLength_SteerWheel    64
#define RS485_RxBufLength_SteerWheel    64
#define EWAY_MODBUS_HEAD                0xFFFF
#define EWYA_MODBUS_BRC_ADDR            0xFE

typedef enum
{
    MB_ERROR          = 0,           //������
    MB_PING           = 0x01,        //ping����
    MB_REG_READ       = 0x02,        //���Ĵ���
    MB_REG_WRITE      = 0x03,        //д�Ĵ���
    MB_BRC_READ       = 0x04,        //�㲥��
    MB_BRC_WRITE      = 0x05,        //�㲥д
    MB_DYNAMIC_ADDR   = 0x06,        //��̬��ַ����

    MB_BUS_SLEEP      = 0x08,        //������߾�Ĭ�����ڽ����޸�

    MB_SYS_ZERO       = 0xE0,        //���������������ʼ��
    MB_SYS_RESTART    = 0xE1,        //ϵͳ����
    MB_SYS_INTT       = 0xE2,        //�ָ���������
    MB_BOOT_RESTART   = 0xE3,        //����ˢ�¹̼�����
    MB_BOOT_HANDSHAKE = 0xE4,        //����ˢ�¹̼�����
    MB_BOOT_TRANSFE   = 0xE5,        //����ˢ�¹̼���������
    MB_BOOT_FORECE_HANDSHAKE = 0xE6, //�̼������޸�����
    MB_BOOT_FORECE_TRANSFE   = 0xE7, //�̼������޸���������

    MB_SYS_CODER_ADJ       = 0xF0,   //���������У׼
    MB_SYS_MOTORCODER_INIT = 0xF1,   //��������̳�ʼ��
    MB_SYS_E2PROM_BACKUP   = 0xF2,   //E2PROM��������

} E_MB_FUNC;  //modbusͨ�Ź�����

typedef enum
{
  Index_HEAD0         = 0,
  Index_HEAD1         = 1,
  Index_ID            = 2,
  Index_LEN           = 3,
  Index_OPT           = 4,
  Index_PayLoad       = 5,
} PACKAGE_Index;
#define Index_Crc     (prbuf[Index_LEN]-2)

#define Reg_VersionGetCmd               2
#define Reg_StopCmd_Addr                81
#define Reg_GetPressure_Addr            82
#define Reg_TargetSpd_Addr              86
#define Reg_JointTargetPos_Addr         88
#define Reg_WheelTargetPos_Addr         90
#define Reg_TargetTorque_Addr           94
#define Reg_Fault1_Addr                 118
#define Reg_PositionOfJointMode_Addr    133
#define Reg_FaultAlarmClear_Addr        202
#define Reg_DogFeedMs_Addr              203

#define AngleMax        (_IQ(0.5) + _IQ(0.18)) //0.27   50��
#define AngleMin        (_IQ(0.5) - _IQ(0.18))
#define DogOutTimeMs    5000
//-------------------- public data ------------------------------------------
extern uint16_t g_u16SteeringWheelAbsPos; //0~4096
extern uint16_t g_u16SteeringWheelPressure;
extern uint8_t  SteerPreSta;           //ת��������ѹ����ָ�����ѹ״̬���յ���ѹ����ʱ������ѹ�������յ�����ѹ���źź���λ�ÿ��� 0x10��ʾ

//-------------------- public functions -------------------------------------
void SteeringWheel_SendPos ( _iq PosSend );
void SteeringWheel_SetStopFree ( void );
void SteeringWheel_DogFeed ( void );
void SteeringWheel_GetSta ( void );
void SteeringWheel_GetAbsPos ( void );
void SteeringWheel_GetVersion ( void );
uint8_t SteeringWheel_PktRev ( void );
void SteeringWheel_GetPressure ( void );
void SteeringWheel_FaultAlarmClear ( void );


#endif // _SteerWheel_Process__H_

//-----------------------End of file------------------------------------------
/** @}*/
