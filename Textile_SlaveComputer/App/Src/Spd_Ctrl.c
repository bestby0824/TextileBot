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
#include "Spd_Ctrl.h"
#include "Host_Com.h"
#include "EncoderProcess.h"
#include "Accelerator.h"
#include "StateCtrl.h"
#include "Pid.h"
#include "pullrod_Ctrl.h"
#include "DataBaseProcess.h"
#include "Monitor.h"
#include "xint.h"
#include "VCUProcess.h"
#include "Xint.h"


//-------------------- local definitions ------------------------------------
#define Break_Range_Init    300

//-------------------- private data -----------------------------------------
//-------------------- private functions declare ----------------------------
PI_CONTROLLER PI_Spd = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER PI_Pos = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER PI_PosStopSpd = PI_CONTROLLER_DEFAULTS;
//-------------------- public data ------------------------------------------
//-------------------- public functions -------------------------------------

/*! \fn
 *  \brief .
 *  \param
 *  \return
 */
_iq VCUSpeedCtl; //目标速度 轮子速度标幺值  角速度（6065代表0.46273689转/s  表示1m/s）  、加积分速度控制
int16_t s16TargetSpeed = 0;
float Line_TargetSpeed = 0;//目标速度 轮子速度标幺值  线速度（单位 m/s）
float SpeedKi = 0.04;
_iq g_qCurWSpeed = 0;//当前速度（由左右码盘计算可得） 轮子速度标幺值
uint32_t Spd_Err_Cnt = 0;
uint16_t Last_Pos_To_Claw = 0;              //上位机发送最后到纸卷的距离(0-10000代表整圈，距离2.16142m)
uint16_t SlowCtrl_Pos = 0;
int16_t  g_WheelPos = 0, PI_Pos_up;         //车轮位置，综合两车轮
int16_t left_WheelPos, Right_WheelPos, Last_left_WheelPos, Last_Right_WheelPos; //左右车轮增量
int16_t PosCtrl_InitFlag = 1, SlowCtrl_InitFlag = 1;
int16_t PosModeState = 0;
uint16_t Break_Range = 60, BreakAdd = 500;             //BreakAdd = 600        2024.05.23 由600修改为400
uint16_t SlowCtrlBreak_Range = 60, Break_RangeFlag = 0;
uint16_t PosModeStateCnt = 0;
uint8_t u8LowSpdCnt = 0;
_iq Pos_StopTSpeed = 0;//定点停车速度
float MotorCurnum[6] = {0};
uint16_t RefKp = 10, StopToMoveFlag = 0, StopToMoveCnt = 0;
static _iq iqLastPosAbsOutAxle;
_iq   iqPosAdd  ;
#define GasKp_Stop       _IQ ( 3 )
#define GasKi_Stop       _IQ ( 0.12 )
#define PosStopIdle            0           //正在定点停车
#define PosStopOverTime        1           //停车超时
#define PosStopNormal          2           //正常进入刹车，开始刹车
#define PosStopQuit            3           //刹车完成，停车并退出定点停车状态

extern _iq _iqSteerPosRef;

int Brake_State, Break_Cnt, BreakPosToStopCnt = 0;
float SpeedKs = 0.000171;
uint16_t VCU_Speed = 0;
uint16_t StopCnt = 0;           //上次的倒车雷达状态
uint16_t FM = 4;
//50hz
//***********************************
/*关于车速的说明和计算
行车电机：车轮：编码器 = 138：7：14
车轮直径68.8mm  轮周长2161.42mm
收到上位机车速 (21614)2161mm/s时，轮速60Rpm,编码器速度120Rpm,行走电机速度1182.86Rpm
车速与行走电机在数值上的对应关系可以简化成一个系数Ks = 1.8273
*/
//******************************//
void Wheel_Pos_Ctrl_Init ( void )
{
    Last_left_WheelPos = g_nLWheelPos;
    Last_Right_WheelPos = g_nRWheelPos;
    PI_Pos.Ref = 0;
    PI_Pos.Fbk = 0;

}
_iq LastPosUp = 0;
uint16_t PosStopStep = 0, SlowStopStep = 0;    //定点停车异常和结束标志
uint32_t PosMoveCheckCnt = 0, PosStopOverTimecnt = 0, SlowCtrlOverTimecnt;
/**
* @brief  位置控制--1k
* @param  none
* @retval None
*/
void Wheel_Pos_Ctrl ( void )
{
    if ( PosCtrl_InitFlag ) //参数初始化
    {
        Wheel_Pos_Ctrl_Init();
        PosCtrl_InitFlag = 0;
        PosModeState = 1;           //上报上位机状态正在运行定点停车
        LastPosUp = 0;
        PosStopStep = PosStopIdle;
        PosStopOverTimecnt = 0;
        PosMoveCheckCnt = 0;
        StopToMoveCnt = 0;
        PosModeStateCnt = 0;
        u8LowSpdCnt = 0;
    }
    PosStopOverTimecnt++;
//    left_WheelPos = g_nLWheelPos - Last_left_WheelPos;
    Right_WheelPos = g_nRWheelPos - Last_Right_WheelPos;

//    if ( left_WheelPos > ENCLinesHalf ) //过零处理
//    {
//        left_WheelPos = left_WheelPos - ENCLines;
//    }
//    else if ( left_WheelPos < -ENCLinesHalf )
//    {
//        left_WheelPos = left_WheelPos + ENCLines;
//    }
    if ( Right_WheelPos > ENCLinesHalf ) //过零处理
    {
        Right_WheelPos = Right_WheelPos - ENCLines;
    }
    else if ( Right_WheelPos < -ENCLinesHalf )
    {
        Right_WheelPos = Right_WheelPos + ENCLines;
    }
    g_WheelPos = Right_WheelPos;//( left_WheelPos + Right_WheelPos ) * 0.5;

    PI_Pos.Ref = Last_Pos_To_Claw;
    PI_Pos.Fbk = g_WheelPos;

    PI_Pos_up =  PI_Pos.Ref - PI_Pos.Fbk;

    if ( PosStopOverTimecnt > 4000 ) //定点停车超时4s
    {
        PosStopStep = PosStopOverTime;//
    }
    if ( PI_Pos_up > Break_Range )//刹车区
    {
        s16TargetSpeed =  900;  //速度0.15m/s
    }

    if ( PosStopStep == PosStopIdle )//正在定点停车确定刹车区
    {
        Break_Range = BreakAdd + ( abs ( g_qCurWSpeed ) - 1200 ) / FM ;     //2400 = 0.2m/s

        if ( Break_Range > 1500 )
        {
            Break_Range = 1500;
        }
        else if ( Break_Range < 300 )
        {
            Break_Range = 300;
        }
    }
    if ( ( PI_Pos_up <= Break_Range ) && ( PosStopStep == PosStopIdle ) )//位置小于刹车区
    {
        PosStopStep = PosStopNormal;      //正常刹车
    }
    if ( ( PosStopStep == PosStopOverTime ) || ( PosStopStep == PosStopNormal ) ) //位置小于刹车区或定点停车超时未动
    {
        BreakPos = BrakeDeep;//刹车制动50
        BreakFlag = 4;//给上位机上报

        if ( abs ( g_qCurWSpeed ) < 50 ) //稳定速度
        {
            u8LowSpdCnt++;
            if ( u8LowSpdCnt > 50 )//0.05s
            {
                PosStopStep = PosStopQuit;
                PosModeFlag = 0;     //为0退出定点停车模式
                u8LowSpdCnt = 0;
                PosModeState = 0;
            }
        }
        else
        {
            if ( u8LowSpdCnt > 0 ) u8LowSpdCnt--;
        }
        PosModeStateCnt++;
        if ( PosModeStateCnt > 1500 )//1.5s
        {
            if ( PosStopStep == PosStopOverTime )
            {
                PosModeState = 2;    //异常上报EndOfCtrl---给上位机上报
            }
            else
            {
                PosModeState = 0;    //正常上报EndOfCtrl---给上位机上报
            }
            PosModeFlag = 0;     //为0退出定点停车模式
            s16TargetSpeed =  0;
            PosModeStateCnt = 0;
        }
    }
}
/**
* @brief  低速控制
* @param  none
* @retval None
*/
void SlowSpeedCtrl ( void )
{
    if ( SlowCtrl_InitFlag ) //参数初始化
    {
        Wheel_Pos_Ctrl_Init();
        SlowCtrl_InitFlag = 0;
        SlowCtrlOverTimecnt = 0;
        SlowStopStep = 0;
//        PI_Pos_up = 0;
    }
    SlowCtrlOverTimecnt++;
    if ( SlowDir == 1 )
    {
        left_WheelPos = g_nLWheelPos - Last_left_WheelPos;
        Right_WheelPos = g_nRWheelPos - Last_Right_WheelPos;

//        if ( left_WheelPos > 10000 ) //过零处理
//        {
//            left_WheelPos = left_WheelPos - ENCLines;
//        }
//        else if ( left_WheelPos < -10000 )
//        {
//            left_WheelPos = left_WheelPos + ENCLines;
//        }
        if ( Right_WheelPos > ENCLinesHalf ) //过零处理
        {
            Right_WheelPos = Right_WheelPos - ENCLines;
        }
        else if ( Right_WheelPos < -ENCLinesHalf )
        {
            Right_WheelPos = Right_WheelPos + ENCLines;
        }
        m_eRunState = RunSta_Run_D;

    }
    else if ( SlowDir == 2 )
    {
        left_WheelPos =  Last_left_WheelPos - g_nLWheelPos;
        Right_WheelPos = Last_Right_WheelPos - g_nRWheelPos;
//        if ( left_WheelPos > 10000 ) //过零处理
//        {
//            left_WheelPos = left_WheelPos - ENCLines;
//        }
//        else if ( left_WheelPos < -10000 )
//        {
//            left_WheelPos = left_WheelPos + ENCLines;
//        }
        if ( Right_WheelPos > ENCLinesHalf ) //过零处理
        {
            Right_WheelPos = Right_WheelPos - ENCLines;
        }
        else if ( Right_WheelPos < -ENCLinesHalf )
        {
            Right_WheelPos = Right_WheelPos + ENCLines;
        }
        m_eRunState = RunSta_Run_R;

    }
    s16TargetSpeed =  600;  //速度0.1m/s

    g_WheelPos = ( left_WheelPos + Right_WheelPos ) * 0.5;

    PI_Pos.Ref = SlowCtrl_Pos;
    PI_Pos.Fbk = g_WheelPos;
    PI_Pos_up =  PI_Pos.Ref - PI_Pos.Fbk;
    if ( PI_Pos_up <= SlowCtrlBreak_Range )   //Break_Range
    {
        SlowStopStep = PosStopNormal;      //正常刹车
    }
    if ( SlowCtrlOverTimecnt > 12000 )
    {
        SlowStopStep = PosStopOverTime;      //超时
    }
    if ( ( SlowStopStep == PosStopNormal ) || ( SlowStopStep == PosStopOverTime ) ) //位置小于刹车区或定点停车超时未动
    {
        BreakPos = BrakeDeep;
        BreakFlag = 5;

        if ( abs ( g_qCurWSpeed ) < 50 ) //稳定速度
        {
            u8LowSpdCnt++;
            if ( u8LowSpdCnt > 50 )
            {
                SlowStopStep = PosStopQuit;
                SlowModeFlag = 0;     //为0退出定点停车模式
                u8LowSpdCnt = 0;
            }
        }
        else
        {
            if ( u8LowSpdCnt > 0 ) u8LowSpdCnt--;
        }
        s16TargetSpeed =  0;

        PosModeStateCnt++;
    }
}
uint8_t i = 0, j = 0;
float Cur_K = 0;
/**
* @brief  速度控制
* @param  none
* @retval None
*/
int8_t SpdCtrl_Ctrl()
{
    RunStates nDriveMode = RunSta_Idle;
    nDriveMode = GetRUNSta();//从状态机取当前状态
//避障雷达逻辑
    if ( RadarSwitch )
    {
        if ( nDriveMode != RunSta_Run_D )  //安全避障 前进时不生效
        {
            if ( StopFlag == 1 )//减速区
            {
                if ( abs ( s16TargetSpeed ) > _IQ ( 0.06 ) )
                {
                    s16TargetSpeed = _IQsat ( s16TargetSpeed, _IQ ( 0.06 ), _IQ ( 0 ) );
                }
            }
            else if ( StopFlag == 2 )//停车区
            {
                if ( abs ( s16TargetSpeed ) > _IQ ( 0.03 ) )
                {
                    s16TargetSpeed = _IQsat ( s16TargetSpeed, _IQ ( 0.03 ), _IQ ( 0 ) );
                }
            }
            else if ( StopFlag == 3 )//急停区
            {
                s16TargetSpeed = _IQ ( 0 );
//                DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_STOP2 );  //2类急停
                LastStopFlag = StopFlag;
                BreakPos = BrakeDeep;
                BreakFlag = 6;

            }
            else
            {
                StopCnt = 0;
                HostCom_ReadGasCommand ( &s16TargetSpeed ); //读取指令，无新指令则使用上次设置
                if ( LastStopFlag == 3 )
                    BreakPos = 0;
            }
            LastStopFlag = StopFlag;
        }
        else
        {
            StopFlag = 0;
            if ( ( m_eCtrlMode == RC_MODE ) && ( s16TargetSpeed > _IQ ( 0.03 ) ) )//遥控模式
            {
                BreakPos = 0;
            }
        }
    }

    PI_Spd.Fbk = abs ( g_qCurWSpeed );
    if ( PosModeFlag )//位置模式
    {
        PI_Spd.Kp = GasKp_Stop;
        PI_Spd.Ki = GasKi_Stop;
        PosStop_SpdCtrl ( s16TargetSpeed );
        PI_Spd.Ref = PI_PosStopSpd.Out * SpeedKs;//没用？
        VCUSpeedCtl = s16TargetSpeed;
    }
    else//速度控制
    {
        if ( 0 == s16TargetSpeed )
        {
            PI_Spd.Ref = 0;
        }
        else
        {
            PI_Spd.Ref = s16TargetSpeed;
            Line_TargetSpeed = ( float ) s16TargetSpeed * SpeedKs; //转换成线速度m/s   s16TargetSpeed / 32767*150/60*2.161*1.0357; (最大线速度5.595m/s)
            PI_Spd.Fbk = abs ( g_qCurWSpeed );
        }
        PI_PosStopSpd.ui = 0;

        //启动速度，加积分启动
        PI_Spd.up = PI_Spd.Ref - PI_Spd.Fbk;
        if ( ( PI_Spd.up > ( PI_Spd.Ref * 0.5 ) ) && ( PI_Spd.Fbk < 1213 ) )
        {
            MotorCurnum[i] = VCU_ParamHandle.s16ChassisMotorCurrent;
            i++;
            if ( i > 5 )
            {
                i = 0;
            }
            Cur_K = ( ( ( MotorCurnum[5] - MotorCurnum[2] ) + ( MotorCurnum[4] - MotorCurnum[1] ) + ( MotorCurnum[3] - MotorCurnum[0] ) ) / 3 ) / 1000;
            if ( Cur_K < 0.1 )
            {
                PI_Spd.ui = PI_Spd.ui + PI_Spd.up * SpeedKi;
            }
            VCUSpeedCtl = s16TargetSpeed + PI_Spd.ui;
            VCUSpeedCtl = _IQsat ( VCUSpeedCtl, _IQ ( 0.0555 ), 0 ); //限速0.6m/s
        }
        else
        {
            PI_Spd.ui = 0;
            i = 0;
            VCUSpeedCtl = s16TargetSpeed;
        }
    }
    VCU_Speed = VCUSpeedCtl * 0.09 ;     //转换成VCU电机端角速度rpm  VCU_Speed = s16TargetSpeed /32767 *150 /7 * 138 ;
    return Err_None;
}
/**
* @brief  速度闭环
* @param  none
* @retval None
*/
void PosStop_SpdCtrl ( _iq TargetSpeed )
{
    PI_PosStopSpd.Ref = TargetSpeed;

    PI_PosStopSpd.up = PI_PosStopSpd.Ref - PI_PosStopSpd.Fbk;

    PI_PosStopSpd.ui += PI_PosStopSpd.up * PI_PosStopSpd.Ki;

    PI_PosStopSpd.Out = _IQmpy ( PI_PosStopSpd.up, PI_PosStopSpd.Kp ) + PI_PosStopSpd.ui;

    PI_PosStopSpd.Out  = _IQsat ( PI_PosStopSpd.Out, PI_PosStopSpd.Umax, PI_PosStopSpd.Umin );
}
/**
* @brief  
* @param  none
* @retval None
*/
int8_t SpdCtrl_Init()
{
    Accl_Init ();
//    PI_Spd.Kp = _IQdiv ( _IQ ( ParamHandle_Control.Reg_Gas_Kp ), _IQ ( 10 ) );
//    PI_Spd.Ki = _IQdiv ( _IQ ( ParamHandle_Control.Reg_Gas_Ki ), _IQ ( 1000 ) );
    PI_Spd.Kd = 0;
//    PI_Spd.Umax = ParamHandle_Control.Reg_GasMax - ParamHandle_Control.Reg_GasMin;    //油门输出限制
//    PI_Spd.Umin = - ParamHandle_Control.Reg_GasMin;
    PI_Pos.Umax = _IQ ( 1 );
    PI_Pos.Umin = _IQ ( 0.01 );

    PI_PosStopSpd.Kp =  _IQ ( 0.5 );
    PI_PosStopSpd.Ki =  2;
    PI_PosStopSpd.Umax  = 150;
    PI_PosStopSpd.Umin = 0;
    Pos_StopTSpeed = 100;
    return Err_None;
}

/*****************************************************************
* @brief  出轴累加位置计算
* @param  电机状态结构体指针(输出到此全局变量)
* @param  出轴码盘值
* @retval None
******************************************************************/
void MotorInfo_PosAddCalc ( _iq WheelPos )
{
    _iq iqPositionDelta;
    _iq iqPosition = WheelPos;

    iqPositionDelta = iqPosition - iqLastPosAbsOutAxle;

    if ( iqPositionDelta < -5000 )
    {
        iqPositionDelta += 10000;
    }
    if ( iqPositionDelta > 5000 )
    {
        iqPositionDelta -= 10000;
    }

    iqLastPosAbsOutAxle = iqPosition;

    iqPosAdd += iqPositionDelta;
}
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


