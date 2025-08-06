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

//-------------------- include files ----------------------------------------
#include "pullrod_Ctrl.h"
#include "StateCtrl.h"
#include "Uart_Dbug.h"
#include "Host_Com.h"
#include "Xint.h"
#include "VCUProcess.h"
#include "NodeProcess.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------


//-------------------- public functions -------------------------------------
uint16_t Cnt = 0;
/**
  * @brief  Һѹ���ת������----100hz
  * @param  none
  * @retval None
  */
#if 0
void Solenoid_Ctrl ( void ) //��ͨѶƵ�����ƣ�10mSִ��һ��
{
    uint16_t RUNSta;
    _iq qTemp = 0;

    RUNSta = GetRUNSta();
//    HostCom_ReadModeCommand(&s16ValveMode[RodID_Lift], &u16RefValvePos[RodID_Lift]);
    if ( m_eCtrlMode == PC_MODE )     //�Զ�ģʽ��ȡ��λ��ָ��
    {
        HostCom_ReadLiftCommand ();
        HostCom_ReadClampCommand ();
        HostCom_ReadBowRiseCommand ();
        HostCom_ReadRotateCommand ();
        HostCom_ReadSideShiftCommand();

    }
    else if ( m_eCtrlMode == RC_MODE ) //ң��ģʽ��ȡң��ָ��
    {
        Cnt++;
        if ( Cnt % 10 == 0 )   //ң��ָ��100ms��ȡһ��
        {
            Hart_SolenoidCmd();         //����ң��ָ��
        }
    }

    if ( SolenoidReport.SolenoidEndOfCtrl == 1 )     //�յ�Һѹ������ִ�еķ���
    {
        if ( SolenoidReport.unLiftAck != 0 )
        {
            SolenoidCtrlCmd.CmdSet.bit.Lift = 0;
        }
        else if ( SolenoidReport.unClampAck != 0 )
        {
            SolenoidCtrlCmd.CmdSet.bit.Claw = 0;
        }
        else if ( SolenoidReport.unBowRiseDegree != 0 )
        {
            SolenoidCtrlCmd.CmdSet.bit.BowRise = 0;
        }
        else if ( SolenoidReport.unRotateDegree != 0 )
        {
            SolenoidCtrlCmd.CmdSet.bit.Rotate = 0;
        }
        else if ( SolenoidReport.unSideShiftDegree != 0 )
        {
            SolenoidCtrlCmd.CmdSet.bit.SideShift = 0;
        }
    }
    if ( SolenoidReport.u16StaFlg == 1 )
    {
        SolenoidCtrlCmd.u16StaFlg = 1;
    }
    else if ( SolenoidReport.u16StaFlg == 0 )
    {
        SolenoidCtrlCmd.u16StaFlg = 0;
    }
    SetSolenoidPos ();//(����)
    return;
}
#endif
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


