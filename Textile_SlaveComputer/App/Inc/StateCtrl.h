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

//-----------------------------------------------------------------------------
#ifndef _StateCtrl__H_
#define _StateCtrl__H_
//-------------------- include files ----------------------------------------
#include "Driver.h"
#include "DidoProcess.h"
#include "SteeringWheel_Ctrl.h"
#include "Monitor.h"
//-------------------- public definitions -----------------------------------
typedef enum 
 {
    RunSta_Reset2Idle = 0,// ��ʼ״̬
    RunSta_Idle,  // 1:����״̬����ʼ���������
    RunSta_Run_D, // 2: ����D��
    RunSta_Run_R, // 3: ����R��
    RunSta_Run_N, // 4:���ҿյ�
    RunSta_Fault, // �й���
    RunSta_Run_P, // 6:P��

} RunStates;
typedef enum {
    Act_Brake = 0, // 1:����ɲ��
    Act_Shift_N,   // 1:�յ�
    Act_Lift,      // 1:�����Ƹ˻���
    Act_Claw,      // 1:�Ƹ˻���
    Act_BowRise,   // 1:�Ƹ˻���
    Act_Rotate,    // 1:�Ƹ˻���
    Act_Shift_N_Bump_ON, // �յ�
    Act_Shift_D,   // ��D��������0
    Act_Steering,  // ת�����ӻ���
    Act_Finish,    // ��ʼ����ɣ��޹���
} ActStates;//����״̬����¼��ǰ����ִ�еĶ���
extern uint16_t RunState;//��ǰ��λ�������ĵ�λ
extern RunStates m_eRunState;
#define Lim_ON    ((LimStaNow == LimH)||(LimStaNow == LimM)||(LimStaNow == LimL))

#define NextStateCheck()  {                                             \
        s8Ret = HostCom_ReadDrvModeCommand(&s16SetRunSta);              \
        if((Err_None == s8Ret)&&(s16SetRunSta < RunSta_Fault))          \
        {                                                               \
            m_eRunState = s16SetRunSta;                                 \
            RunState = s16SetRunSta;                                    \
            if(RunSta_Reset2Idle == m_eRunState){StateResetSystem();}   \
        }                                                               \
        if(0 != FaultCtrl.all)                            \
        {                                                               \
            m_eRunState = RunSta_Fault;                                 \
        }                                                               \
    }
//-------------------- public data ------------------------------------------
//-------------------- public functions -------------------------------------
void StateResetSystem();
void RunStateInit ( void );
void RunStateCtrl ( void );
RunStates GetRUNSta ( void );
ActStates GetPreSta ( void );   
void Delay_mS ( uint32_t Delay );
#endif // _XXX_H_

//-----------------------End of file------------------------------------------
/** @}*/
