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
* @brief       CAN������ת��������ڵ�״̬����
 * @param       ��
 * @retval      ��
 */
void HostState_Ctrl ( HostComStates HostState )
{
    switch ( HostState )
    {
    case HostCom_Idle:
    {
        if ( 1 == u8Standby_Flag )
        {
            m_eHostComState = HostCom_Running;  //��ʱ�������ķ���ģ��
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
        BootProcess ( );  //д������־������IAP
    }
    break;
    case HostCom_UpDataWait:  //�رձ��ķ��ͼ������������������
    {
        SetBoardUpdata_Fault ( );  //��λ�������Һѹ����������ʱ����λ��1
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
 * @brief       ����������
 * @param       ��
 * @retval      ��
 */
static void ParamProcess ( void )
{

}
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


