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
#include "HostState_Ctrl.h"
#include "HostProcess.h"
#include "Can_Host.h"
#include "Monitor.h"
#include "main.h"
#include "DataBaseProcess.h"
#include "string.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
static ParamStruct_Steer ParamRxHandle_Steer;

//-------------------- private functions declare ----------------------------
static void ParamProcess ( void );

//-------------------- public data ------------------------------------------
HostComStates m_eHostComState = HostCom_Idle;
ParamManageStates ParamManage_Opt = ParamManage_Start;
ParamTxStruct ParamTxHandle;

//-------------------- public functions -------------------------------------
/**
* @brief       CAN总线上转向驱动板节点状态控制
 * @param       无
 * @retval      无
 */
void HostState_Ctrl ( HostComStates HostState )
{
    switch ( HostState )
    {
    case HostCom_Idle:
    {
        if ( 1 == u8Standby_Flag )
        {
            m_eHostComState = HostCom_Running;  //延时启动报文发送模块
        }
    }
    break;
    case HostCom_Running:
    {
        CanFBDataObtain ( );
        CanTaskProcess ( );
        CanBusOff_Reset ( );
    }
    break;
    case HostCom_ToIAP:
    {
        BootProcess ( );  //写升级标志，并跳IAP
    }
    break;
    case HostCom_UpDataWait:  //关闭报文发送及电机驱动，并报故障
    {
        SetBoardUpdata_Fault ( );  //下位机板或者液压驱动板升级时，该位置1
    }
    break;
    case HostCom_ParamManage:
    {
        ParamProcess ( );
        SetBoardUpdata_Fault ( );
    }
    break;
    default:
        break;
    }
}
/**
 * @brief       参数管理函数
 * @param       无
 * @retval      无
 */
static void ParamProcess ( void )
{

}
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


