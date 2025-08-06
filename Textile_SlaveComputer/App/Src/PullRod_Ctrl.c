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
  * @brief  液压板和转向板控制----100hz
  * @param  none
  * @retval None
  */
#if 0
void Solenoid_Ctrl ( void ) //受通讯频率限制，10mS执行一次
{
    uint16_t RUNSta;
    _iq qTemp = 0;

    RUNSta = GetRUNSta();
//    HostCom_ReadModeCommand(&s16ValveMode[RodID_Lift], &u16RefValvePos[RodID_Lift]);
    if ( m_eCtrlMode == PC_MODE )     //自动模式读取上位机指令
    {
        HostCom_ReadLiftCommand ();
        HostCom_ReadClampCommand ();
        HostCom_ReadBowRiseCommand ();
        HostCom_ReadRotateCommand ();
        HostCom_ReadSideShiftCommand();

    }
    else if ( m_eCtrlMode == RC_MODE ) //遥控模式读取遥控指令
    {
        Cnt++;
        if ( Cnt % 10 == 0 )   //遥控指令100ms读取一次
        {
            Hart_SolenoidCmd();         //解析遥控指令
        }
    }

    if ( SolenoidReport.SolenoidEndOfCtrl == 1 )     //收到液压板正在执行的反馈
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
    SetSolenoidPos ();//(不用)
    return;
}
#endif
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


