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

//-------------------- include files ----------------------------------------
#include "RelayCmd_Ctrl.h"
#include "HostProcess.h"
#include "RelayProcess.h"

//-------------------- private functions declare ----------------------------

//-------------------- local definitions ------------------------------------
#define RelayCtrl_TimeOut          10000
#define RelayCtrl_Running          1000

//-------------------- private data -----------------------------------------

//-------------------- private functions declare ----------------------------

//-------------------- public data ------------------------------------------
S_RELAY_CMD gRelayCmd;
S_RELAYTX_STRUCT gRelayTxHandle;

//-------------------- public functions -------------------------------------
/**
 * @brief       继电器控制函数
 * @param       输入：继电器控制指令
 * @retval      无
 * @note
 */
void RelayCmd_Ctrl ( RegDef_RelayCmd *RelayCmd )
{
    static uint16_t u16RelayInit = 0, u16Cnt_Init = 0; 
    static uint16_t u16Cnt_50mS = 0, u16Cnt_100mS = 0;
    
    if ( u16RelayInit == 0 )
    {
        if ( u16Cnt_Init < 6000 ) u16Cnt_Init++;
        else
        {
            u16Cnt_Init = 0;
            u16RelayInit = 1;
        }
        gRelayTxHandle.u16TxType = RelayCtrl_Write;
        gRelayTxHandle.eTxMode = OPEN;
    }
    else
    {
        if ( RelayCmd->u16EN == 0 ) gRelayCmd.u16LastSN = 0;  //DISABLE之后清除SN
        
        if ( ( RelayCmd->u16SN != gRelayCmd.u16LastSN ) && ( RelayCmd->u16EN ) )
        {
            gRelayCmd.u16LastSN = RelayCmd->u16SN;
            if ( RelayCmd->u16SN == 0 )
            {
                gRelayCmd.eRunState = Relay_CmdInvaid;
                gRelayCmd.iqRuningTime = 0;
            }
            else
            {
                gRelayCmd.eRunState = Relay_CmdRuning;
                gRelayCmd.iqTimeOut = RelayCtrl_TimeOut;//( RelayCmd->u16Time > _IQdiv2 ( RelayCtrl_TimeOut ) ) ? _IQmpy2 ( RelayCmd->u16Time ) : RelayCtrl_TimeOut;
                if ( RelayCmd->u16Mode == OPEN )
                {
                    gRelayCmd.iqTimeLimit = RelayCtrl_Running*4;
                }
                else
                {
                    gRelayCmd.iqTimeLimit = RelayCtrl_Running;//RelayCmd->u16Time;;
                }
                gRelayCmd.iqRuningTime = 0;
                gRelayCmd.eCmdMode = RelayCmd->u16Mode;
                gRelayCmd.eFbMode = RELAY_NONE;
            }
        }

        if ( gRelayCmd.eRunState == Relay_CmdRuning )
        {
            if ( gRelayCmd.iqTimeOut > 0 )
            {
                gRelayCmd.iqTimeOut--;
                if ( gRelayCmd.eCmdMode != gRelayCmd.eFbMode )
                {
                    if ( gRelayCmd.iqTimeOut % 100 == 0 )
                    {
                        gRelayTxHandle.u16TxType = RelayCtrl_Write;
                        gRelayTxHandle.eTxMode = gRelayCmd.eCmdMode;
                    }
                    else if ( gRelayCmd.iqTimeOut % 100 == 50 )
                    {
                        gRelayTxHandle.u16TxType = RelayCtrl_Read;
                        gRelayTxHandle.eTxMode = RELAY_NONE;
                    }
                    else
                    {
                        ;
                    }
                }
                else
                {
                    gRelayCmd.iqRuningTime++;
                    gRelayTxHandle.u16TxType = RelayCtrl_Read;
                    gRelayTxHandle.eTxMode = RELAY_NONE;
                    if ( gRelayCmd.iqRuningTime > gRelayCmd.iqTimeLimit )
                    {
                        gRelayCmd.eRunState = Relay_EndOfCtrl;
                    }
                }
            }
            else
            {
                gRelayTxHandle.u16TxType = RelayCtrl_Read;
                gRelayTxHandle.eTxMode = RELAY_NONE;
                gRelayCmd.eRunState = Relay_CmdTimeOut;
            }
        }
        else
        {
            if ( u16Cnt_100mS < 100 ) u16Cnt_100mS++;
            else u16Cnt_100mS = 0;
            
            if ( u16Cnt_100mS % 100 == 0 )
            {
                gRelayTxHandle.u16TxType = RelayCtrl_Read;
                gRelayTxHandle.eTxMode = RELAY_NONE;
            }
            else if ( u16Cnt_100mS % 100 == 50 )
            {
                if ( gRelayCmd.eFbMode == OPEN )
                {
                    gRelayTxHandle.u16TxType = RelayCtrl_Write;
                    gRelayTxHandle.eTxMode = STOP;
                }
            }
            else
            {
                ;
            }    
        }
    }
    
    if ( u16Cnt_50mS < 50 ) {
        u16Cnt_50mS++;
    } else {
        u16Cnt_50mS = 0;
        RelayCtrl ( gRelayTxHandle.u16TxType, gRelayTxHandle.eTxMode );
    }
}
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */
