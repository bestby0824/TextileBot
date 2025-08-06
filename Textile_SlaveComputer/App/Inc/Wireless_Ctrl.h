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
#ifndef _Wireless_Ctrl__H_
#define _Wireless_Ctrl__H_
//-------------------- include files ----------------------------------------
#include "Debug_Com.h"
#include "DataBaseProcess.h"
#include "Monitor.h"
#include "DidoProcess.h"
//-------------------- public data ------------------------------------------
/*textile*/
#define RC_AGVDriveCmd                  0x0001
#define RC_AGVSteeringCmd               0x0002
#define RC_AGVLiftCmd                   0x0004
#define RC_RotatingCmd                  0x0008
#define RC_TelescopicCmd                0x0010
#define RC_HuggingCmd                   0x0020
#define RC_GlandCmd                     0x0040
#define RC_ESTOPCmd                     0x0080
#define RC_WarningCmd                   0x0100
#define RC_RunningCmd                   0x017F
#define PairingReqCmd                   0xFFFF
#define PairingClcCmd                   0xEEEE

//-------------------- public functions -------------------------------------
void WirelessCtrl();
extern uint8_t u8FaultClear_Flag;
#endif /* Wireless_Ctrl_H_ */