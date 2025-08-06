/**
* ~{!c~}??~{!'~}?~{(4~}??(C)
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
* xxxx/xx/xx | 1.0.0 | HWW | ???~{!'~}????
*
*/
//------------------------------------------------------------------------------

//-------------------- include files ----------------------------------------
#include "StateCtrl.h"
#include "Xint.h"
#include "Iwdg.h"
#include "VCUProcess.h"
#include "NodeProcess.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
RunStates m_eRunState = RunSta_Reset2Idle;
ActStates m_eActState = Act_Brake;
static uint32_t m_u32ActWaitCnt_10mS = 0;
uint8_t  Status_Text;
uint16_t u16Status_1, u16Status_2;

//-------------------- private functions declare ----------------------------
static void GotoIdleState ( void );
//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
void RunStateInit ( void )
{
    m_eRunState = RunSta_Reset2Idle;//RunSta_Idle;//
    m_eActState = Act_Brake;//Act_Brake;
}

void StateResetSystem()
{
    m_eActState = Act_Brake;
    m_eRunState = RunSta_Reset2Idle;
}
/***************实时检测系统状态，对系统进行保护*无用******************/
void RunStateCtrl ( void ) //调用频率100Hz
{
    int16_t s16SetRunSta;
    int8_t s8Ret;

    if ( ( FaultCtrl.all == 0 ) && ( m_eRunState == RunSta_Fault ) )
    {
        StateResetSystem();
    }
    switch ( m_eRunState )
    {
    case RunSta_Reset2Idle:
    {
        RegReal_ChassisParam.u8ChassisEnabled.bit.SteeringEnabled = 1;////转向使能
        GotoIdleState();
        if ( 0 != FaultCtrl.all )
        {
            m_eRunState = RunSta_Fault;
        }
        else
        {
            NextStateCheck();
        }
    }
    break;
    case RunSta_Idle:
    {
        Status_Text = RunSta_Idle;
        RegReal_ChassisParam.u8ChassisEnabled.bit.SteeringEnabled = 1;//转向使能

        if ( 0 ) //待完善，每3S检测一次，不符合空闲条件则复位
        {
        }
        else
        {
            NextStateCheck();
        }
    }
    break;
    case RunSta_Run_D:
    {
        DoSetFBSW ( Shift_D );
        NextStateCheck();
    }
    break;
    case RunSta_Run_R:
    {
        DoSetFBSW ( Shift_R );
        NextStateCheck();

    }
    break;
    case RunSta_Run_N:
    {
        DoSetFBSW ( Shift_N );
        NextStateCheck();
    }
    break;
    case RunSta_Fault:
    {
        DoSetFBSW ( Shift_N );
        NextStateCheck();
    }
    case RunSta_Run_P:
    {
        NextStateCheck();
    }
    break;
    }
}
RunStates GetRUNSta ( void )
{
    return m_eRunState;
}
ActStates GetPreSta ( void )
{
    return m_eActState;
}
//初始化动作，由main函数 100Hz调用
//1.刹车
static void GotoIdleState ( void )
{
    //等待预设时间，各个动作自行设定
    if ( m_u32ActWaitCnt_10mS > 0 ) {
        m_u32ActWaitCnt_10mS--;
    }
    switch ( m_eActState )
    {
    case Act_Brake:
    {
        DoSetFBSW ( Shift_N );
        m_u32ActWaitCnt_10mS = 10;//等待100ms
        m_eActState = Act_Shift_N;
    }

    break;
    case Act_Shift_N:
    {
        if ( m_u32ActWaitCnt_10mS == 0 ) {
            m_eActState = Act_Lift;
        }

    }
    break;
    case Act_Lift:
    {
        m_eActState = Act_Claw;
    }
    break;
    case Act_Claw:
    {
        m_eActState = Act_BowRise;
    }
    break;
    case Act_BowRise:
    {
        m_eActState = Act_Rotate;
    }
    break;
    case Act_Rotate:
    {
        m_eActState = Act_Shift_N_Bump_ON;
        m_u32ActWaitCnt_10mS = 300;
    }
    break;

    case Act_Shift_N_Bump_ON:
    {
        if ( m_u32ActWaitCnt_10mS == 0 )
        {
            DoSetFBSW ( Shift_D );
            m_eActState = Act_Shift_D;
            m_u32ActWaitCnt_10mS = 300; //3S
        }
    }
    break;

    case Act_Shift_D:
    {
        if ( m_u32ActWaitCnt_10mS == 0 )
        {
            m_eActState = Act_Finish;
            m_u32ActWaitCnt_10mS = 100; //3S
        }
    }
    break;
    case Act_Finish:
    {
        if ( m_u32ActWaitCnt_10mS == 0 )
        {
            DoSetFBSW ( Shift_N );
            m_eRunState = RunSta_Idle;
        }
    }
    break;
    default:
    {
        ;
    }
    break;
    }
}
void Delay_mS ( uint32_t Delay )
{
    HAL_Delay ( Delay );
    IwdgReload();
    DO_ToggleByIndex ( DO_CH_LED );
}
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


