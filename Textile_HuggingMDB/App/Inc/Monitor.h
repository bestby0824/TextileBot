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
#ifndef _Monitor__H_
#define _Monitor__H_
//-------------------- include files ----------------------------------------
#include "stm32f4xx_hal.h"

//-------------------- public definitions -----------------------------------
#define NoFault                            0
#define Fault                              1
#define FullLife_2s                        2000
#define FullLife_500ms                     500
#define FullLife_100ms                     100
#define V_SUPPLY_Max                       3000
#define V_SUPPLY_Min                       1500
#define V_SUPPLY_A2D(x)                    x*4845/4096     //x*3.3/4096/2.2*32.3*100
#define HighTempErrValue                   8000
#define HighTempWarnValue                  8000
#define NormalSpdLoseCtrl                  (abs(pi_spd.Ref - pi_spd.Fbk) > _IQ(0.25))
#define ZeroSpdLoseCtrl                    ((abs(pi_spd.Ref) < _IQ(0.05)) && (abs(pi_spd.Fbk) > _IQ(0.1)))
#define ProtectCur                         _IQ(13.2)  //_IQ(1)代表1A

struct Error_BITS              //转向驱动板故障码
{
    uint32_t    uE2promErr                     : 1; //E2p故障
    uint32_t    uDBVersion                     : 1; //数据库版本不一致
    uint32_t    uLackUVW                       : 1; //缺相
    uint32_t    uReangleLack                   : 1; //电机轴编码器离线
    
    uint32_t    uCoderTama1_Lack               : 1; //出轴1号编码器离线
    uint32_t    uCoderTama2_Lack               : 1; //出轴1号编码器离线
    uint32_t    uCmdDogLose                    : 1; //心跳超时
    uint32_t    uSpdCircle                     : 1; //速度跟踪严重不良
    
    uint32_t    uSeveralLodeOver               : 1; //多次过载
    uint32_t    uTempOver                      : 1; //过温
    uint32_t    uVolOver                       : 1; //过压
    uint32_t    uBoardUpdata                   : 1; //存在板卡升级/参数修改
    
    uint32_t    CANComError                    : 1; //CAN通信异常
    uint32_t    rsvd                           : 19;
};

typedef union
{
    uint32_t    all;
    struct      Error_BITS   bit;
} Fault_REG;

struct Warning_BITS
{
    uint32_t    uLowPressure                   : 1; //低油压
    uint32_t    uVolUnder                      : 1; //欠压
    uint32_t    uMotorAxleInvalid              : 1; //电机未标定
    uint32_t    uCoderCaliWarnning             : 1; //左右码盘不匹配（0点与初始标定值差变3°以上）
    
    uint32_t    uLodeOver                      : 1; //过载
    uint32_t    uTempWarnning                  : 1; //过温
    uint32_t    CAN_Board_BUSOFF               : 1; //板卡can总线进入BUSOFF
    uint32_t    uPosCoderInvalid               : 1; //位置零点未标定
    
    uint32_t    rsvd                           : 24;
};

typedef union
{
    uint32_t    all;
    struct      Warning_BITS   bit;
} Warning_REG;

#define BIT(n)            (0x01<<n)
#define ErrBit(n)         (0x01<<n)
extern Fault_REG g_u32FaultNum;
extern Warning_REG g_u32WarningNum;
extern uint8_t Foc_EN_Flag;
extern int16_t s16CtrlLife,s16ReangleLife,s16CoderTama1Life,s16CoderTama2Life;
extern uint32_t FaultCheckEnable, WarnCheckEnable;

uint16_t FaultNum_Get ( Fault_REG FaultNum );
void MonitorInit ( void );
void MonitorProcess(void);
void SetE2prom_Fault ( void );
void SetDBVersion_Fault ( void );
void SetBoardUpdata_Fault ( void );
void FaultClearProcess ( void );


#endif // _XXX_H_

//-----------------------End of file------------------------------------------
/** @}*/
