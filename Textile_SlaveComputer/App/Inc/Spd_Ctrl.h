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
void MotorInfo_PosAddCalc( _iq WheelPos);//编码器单圈转多圈
void PosStop_SpdCtrl(_iq TargetSpeed);
void SlowSpeedCtrl(void);


_iq ramper(_iq in, _iq out, _iq rampDelta);
//-------------------- public data ------------------------------------------
extern PI_CONTROLLER PI_Spd;
extern _iq g_qCurWSpeed;//当前速度（由左右码盘计算可得） 轮子速度标幺值
extern int16_t g_Car_Pos ;    //整车前轮位置(0-360,0-4096)
//extern _iq m_qTargetSpeed;//目标速度 轮子速度标幺值
extern int16_t s16TargetSpeed;
extern int16_t PosCtrl_InitFlag,PI_Pos_up,SlowCtrl_InitFlag;    //位置控制初始化
extern PI_CONTROLLER PI_Pos;
extern uint16_t Break_Range;
extern uint16_t VCU_Speed;
extern int16_t PosModeState;
extern _iq VCUSpeedCtl;
extern uint16_t StopFlag,LastStopFlag;
extern int16_t  g_WheelPos;
extern uint16_t  Last_Pos_To_Claw,SlowCtrl_Pos;             //车轮位置控制，综合两车轮位置
extern uint32_t SlowCtrlOverTimecnt;
extern uint16_t SlowStopStep;     //定点停车异常和结束标志

//-------------------- public functions -------------------------------------


#endif // _SpeedCtrl__H_

//-----------------------End of file------------------------------------------
/** @}*/
