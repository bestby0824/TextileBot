/**
* 版权所有(C)
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
* <日期> | <版本> | <作者> | <描述>
*
* xxxx/xx/xx | 1.0.0 | MUNIU | 创建文件
*
*/
//------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
#ifndef _RELAYCMD_Ctrl_H_
#define _RELAYCMD_Ctrl_H_

#include "stm32f4xx_hal.h"
#include "IQmath.h"
#include "RelayProcess.h"
#include "HostProcess.h"

typedef enum
{
    Relay_EndOfCtrl = 0,
    Relay_CmdRuning,
    Relay_CmdTimeOut,
    Relay_CmdRes1,
    Relay_CmdInvaid,
    Relay_CmdFault = 14,
} E_RELAY_RUN_STATE;

typedef struct {
    E_RELAY_RUN_STATE eRunState;
    RELAY_STA eCmdMode;
    RELAY_STA eFbMode;
    uint16_t u16LastSN;
    _iq iqTimeOut;
    _iq iqTimeLimit;
    _iq iqRuningTime;
} S_RELAY_CMD;
extern S_RELAY_CMD gRelayCmd;

typedef struct {
    uint16_t u16TxType;
    RELAY_STA eTxMode;    
} S_RELAYTX_STRUCT;

void RelayCmd_Ctrl ( RegDef_RelayCmd *RelayCmd );
#endif

//-----------------------End of file------------------------------------------
/** @}*/




