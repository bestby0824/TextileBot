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
#ifndef _SpeedCtrl__H_
#define _SpeedCtrl__H_
//-------------------- include files ----------------------------------------
#include"Driver.h"
#include "AngleMath.h"
#include "Pid.h"
//-------------------- public definitions -----------------------------------
int8_t SpdCtrl_Ctrl();
extern uint16_t g_unCurBrakPerc;
int8_t SpdCtrl_Init();
void Wheel_Pos_Ctrl_Init(void);
void Wheel_Pos_Ctrl(void);
void MotorInfo_PosAddCalc( _iq WheelPos);//��������Ȧת��Ȧ
void PosStop_SpdCtrl(_iq TargetSpeed);
void SlowSpeedCtrl(void);


_iq ramper(_iq in, _iq out, _iq rampDelta);
//-------------------- public data ------------------------------------------
extern PI_CONTROLLER PI_Spd;
extern _iq g_qCurWSpeed;//��ǰ�ٶȣ����������̼���ɵã� �����ٶȱ���ֵ
extern int16_t g_Car_Pos ;    //����ǰ��λ��(0-360,0-4096)
//extern _iq m_qTargetSpeed;//Ŀ���ٶ� �����ٶȱ���ֵ
extern int16_t s16TargetSpeed;
extern int16_t PosCtrl_InitFlag,PI_Pos_up,SlowCtrl_InitFlag;    //λ�ÿ��Ƴ�ʼ��
extern PI_CONTROLLER PI_Pos;
extern uint16_t Break_Range;
extern uint16_t VCU_Speed;
extern int16_t PosModeState;
extern _iq VCUSpeedCtl;
extern uint16_t StopFlag,LastStopFlag;
extern int16_t  g_WheelPos;
extern uint16_t  Last_Pos_To_Claw,SlowCtrl_Pos;             //����λ�ÿ��ƣ��ۺ�������λ��
extern uint32_t SlowCtrlOverTimecnt;
extern uint16_t SlowStopStep;     //����ͣ���쳣�ͽ�����־

//-------------------- public functions -------------------------------------


#endif // _SpeedCtrl__H_

//-----------------------End of file------------------------------------------
/** @}*/
