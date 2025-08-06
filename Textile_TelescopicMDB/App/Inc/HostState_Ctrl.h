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
* xxxx/xx/xx | 1.0.0 | MUNIU | 创建文件
*/
//------------------------------------------------------------------------------
//-----------------------------------------------------------------------------
#ifndef _HostState_Ctrl__H_
#define _HostState_Ctrl__H_
//-------------------- include files ----------------------------------------
#include "stm32f4xx_hal.h"

//-------------------- public definitions -----------------------------------
#define ParamAck                         1
#define ParamNack                        2
#define ParamSuccess                     3
#define ParamFail                        4
#define PacketDataLen                    6

typedef enum {
    HostCom_Idle = 0,
    HostCom_Running,
    HostCom_ToIAP,
    HostCom_UpDataWait,
    HostCom_ParamManage,
} HostComStates;
extern HostComStates m_eHostComState;

typedef enum {
    ParamManage_Idle = 0,
    ParamManage_Start,
    ParamManage_Program,
    ParamManage_Save,
    ParamManage_Success,
    ParamManage_Fail,
} ParamManageStates;

typedef enum {
    PARAM_CMD_LINK = 1,
    PARAM_CMD_WRITE,
    PARAM_CMD_END,
} ParamCmd_TypeDef;

typedef struct {
    uint8_t u8Cmd;   //命令字
    uint8_t u8Cnt;   //计数
    uint8_t u8FB;    //反馈
    uint8_t u8Data[5];
} ParamTxStruct;
extern ParamTxStruct ParamTxHandle;
//-------------------- public functions -------------------------------------
void HostState_Ctrl ( HostComStates HostState );
#endif // _HostState_Ctrl_H_

//-----------------------End of file------------------------------------------
/** @}*/
