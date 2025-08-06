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

//-----------------------------------------------------------------------------
#ifndef _DidoProcess__H_
#define _DidoProcess__H_
//-------------------- include files ----------------------------------------
#include "Dido.h"
#include "StateCtrl.h"
//#include "Host_Com.h"
//-------------------- public definitions -----------------------------------
typedef enum{
    Shift_N=0,//前进&左转向&无刹车
    Shift_D=1,//空挡
    Shift_R=2,//后退&右转向
}ShiftSta;
typedef enum {
    Sta_L=0,//左转向
    Sta_N,//空挡
    Sta_R,//右转向
    Sta_A,//
} LightSta;
typedef enum {
    Brake_NC=0,//无刹车
    Brake_ON1=1,//触发SW1
    Brake_ON2=2,//触发SW2
} BrakeSta;
typedef enum {
    OFF=0,//断开
    ON=1,//接通
} SWSta;
typedef enum {
    LimH=0,//上
    LimM=1,//中
    LimL=2,//下
    LimN=3,//无触发
    LimErr=4,
} PosLim;

typedef enum {
    Led_OFF,    // 关闭
    Led_R,    // 红灯
    Led_G,  // 绿灯
    Led_Y    // 黄灯
} TriColorSta;

typedef enum 
{
    PCPower,
    Motor1Power,
    Motor2Power,
    SensorPower,
    AllPower
} PowerOptDef;
typedef enum 
{
    PC_MODE = 0,  //遥控模式
    RC_MODE,        //初始模式
    Idle_MODE
//    PC_MODE,        //自动模式
//    HAND_MODE,      //手动模式
} CtrlMode;

typedef struct 
{
    uint8_t PC_5VSta;
    uint8_t RST_MCUSta;
    uint8_t PowerSWSta;
    uint8_t STOP_0Sta;
    uint8_t CtrlModeOpt;
    uint8_t TouchEdgeSta;
}RegDef_InputIO;
//-------------------- public data ------------------------------------------
#define ADDRESS_NRSTIMES            ADDR_FLASH_SECTOR_3
extern CtrlMode m_eCtrlMode ;
extern uint16_t LastMode;
extern RegDef_InputIO InputIOInfor;
//-------------------- public functions -------------------------------------
void UpdateIOSta(void);
void DidoProcessInit(void);
int8_t DoSetFBSW (ShiftSta Sta);
ShiftSta DoGetFBSW();

int8_t DoSetPower_SW1 (SWSta Sta);//(5v,9v,12v)
int8_t DoSetPower_SW2 (SWSta Sta);//电机24V
int8_t DoSetPower_SW3 (SWSta Sta);
int8_t DoSetTAG_LED1_ON ( SWSta Sta );
int8_t DoSetTAG_LED2_ON ( SWSta Sta );
int8_t DoSet0_STOP_CTRL_ON ( SWSta Sta );
int8_t DoSet2_STOP_CTRL_ON ( SWSta Sta );
int8_t DoSeWhistle (SWSta Sta);
int8_t DoSetLED5 ( SWSta Sta );
int8_t DoSetLED6 ( SWSta Sta );
int8_t DoSetRadar  ( uint16_t *StaL ,uint16_t *StaR);
void CtrlModeFeedback ( void );
CtrlMode GetCtrlMode ( void );
void Set_NRSTIMES ( void );
void NRST_Erase ( void );

extern SWSta Reg_SWSta;
extern LightSta Reg_LightSta;
extern TriColorSta Reg_TriColorSta;
extern E_DO_CH Reg_E_DO_CH;
void PowerCtrl_SW ( E_DO_CH PowerOpt, SWSta Sta );
void TurnSignalCtrl ( LightSta Sta );
void Buzzer_Beep(uint32_t on_time, uint32_t off_time);
void TriColorLedCtrl ( TriColorSta Sta );
void DoSetSteeringLampSW ( LightSta Sta, uint32_t on_time, uint32_t off_time ) ;
void PowerCtrl( PowerOptDef PowerOpt, SWSta PowerSta);
/*textile*/
void DoSetMagnetPower ( SWSta Sta );
void TurnSignalCtrl ( LightSta Sta );
void STOP_0_Ctrl ( SWSta Sta ); 
#endif // _Dido_Process__H_

//-----------------------End of file------------------------------------------
/** @}*/
