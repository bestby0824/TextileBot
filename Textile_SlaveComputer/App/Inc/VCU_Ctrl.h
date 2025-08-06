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
#ifndef _VCUCtrl__H_
#define _VCUCtrl__H_
//-------------------- include files ----------------------------------------
#include"Driver.h"
#include"IQmath.h"
#include"string.h"
#include "VCUProcess.h"
//-------------------- public definitions -----------------------------------
#define   Forward               1
#define   Reverse               2
#define   Up               3
#define   Dn               4
#define   RC_Debug            5
#define   TurnCtrlPowerON  3
#define   TurnEncoder        65536
#define   TurnSpdCLc(Spd)    _IQdiv(Spd * 512,1875)
//#define   Reduction_Ratio(Deg)    _IQdiv(127 * 28 * Deg, (18 * 360))
#define   LiftPosMin           900      //9cm
#define   LiftPosMax           6000      //60cm
#define   LiftPumpMaxSpd        255
#define   LiftDnMaxSpd          200
#define   INVWalkspdKs                 9251         //IQ(0.5)对应2m/s(4625.5rpm)
#define   INVWalkspdClc(Spd)       _IQdiv(Spd, INVWalkspdKs)      //rpmtodata(65536)
#define   WalkspdMax                3470            //694
#define   LengthOf_1_Circle        _IQ(0.729791963)
#define   WalkPosClc(Pos)       _IQdiv(Pos, ENCLines)      //rpmtodata(65536)

//#define   ReductionRatio(Deg)    _IQdiv(127 * 28 * Deg, (18 * 360))

#define   Reduction_Ratio(Deg)    _IQdiv( 360 * Deg, _IQ(1.8227))//90°-3235990
#define   INVReduction_Ratio(Deg)  _IQdiv(_IQmpy(Deg, _IQ(1.8227)), _IQ(360.0))



//-------------------- public data ------------------------------------------
extern int32_t TurnDeg, TurnDegFb, TurnSpd;
extern uint8_t AGVCtrlMode, AGVTurnCtrlMode;
extern int16_t s16AGVWalkSpd;
extern int16_t s16AGVWalkBrakePosFB, s16WalkSpd;
extern _iq iqPosRef_m, iqPosition_m;
extern _iq iq_PosAddCircles;
extern _iq LiftSpd;
extern _iq iqAGVWalkPosFb;
extern uint32_t u32LiftPos;
extern uint16_t AGVLiftEndOfCtrlCnt;
extern uint16_t AGVWalkBrakeCnt,AGVSlowSpdPosFB, AGVSlowSpdPosCnt, BreakCanelCnt;
//-------------------- public functions -------------------------------------
void VCU_RegState_Ctrl ( void );
void PosAddCalc ( void );
int16_t AGVWalkPosLp ( _iq ref );
extern uint16_t RunState_RC;
extern _iq LiftSpdCalc ( uint16_t CalcFreq );
void AGVTurnCtrl(void);
#endif // _SpeedCtrl__H_

//-----------------------End of file------------------------------------------
/** @}*/
