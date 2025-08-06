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
#ifndef _IMUCom__H_
#define _IMUCom__H_
//-------------------- include files ----------------------------------------
#include "Driver.h"
#include "ErrorCode.h"
#include "Uart_Dbug.h"
#include "Uart_PCDbug.h"

//-------------------- local definitions ------------------------------------
#define Reg_IMUState                 0x50

//-------------------- public definitions -----------------------------------
typedef struct {
    int16_t nImuRZ;                     //pu值-32767~32767 对应 -pi~+pi
    int16_t nImuRY;                     //
    int16_t nImuRX;                     //
    int16_t nImuRZSpeed;                //pu值-32767~32767 对应 -2pi~+2pi弧度/s
    int16_t nImuRYSpeed;                //
    int16_t nImuRXSpeed;                //
    int16_t nImuZAcc;                   //pu值-32767~32767 对应 -2g~+2g
    int16_t nImuYAcc;                   //
    int16_t nImuXAcc;                   //
} IMUData;

#define IMUDataDefault {                  \
    0,    /* nImuRZ;                     */ \
    0,    /* nImuRY;                     */ \
    0,    /* nImuRX;                     */ \
    0,    /* nImuRZSpeed;                */ \
    0,    /* nImuRYSpeed;                */ \
    0,    /* nImuRXSpeed;                */ \
    0,    /* nImuZAcc;                   */ \
    0,    /* nImuYAcc;                   */ \
    0,    /* nImuXAcc;                   */ \
};
//-------------------- public data ------------------------------------------
extern IMUData g_sImuData;
extern uint16_t u16IMUCRCErrCnt,u16IMUCRCErrPerCnt;
extern uint8_t g_bIMURxBuf[8];
extern uint8_t u8IMUInitRes;
extern uint8_t u8IMUDataRxFlag;

//-------------------- public functions -------------------------------------
//初始化
int8_t IMU_AngleCal();
//处理接收数据
int8_t IMUCom_ProcRecvData();
void IMUData_Receive ( void );
extern uint8_t DbugCom_Tx[8];
#endif // _IMUCom__H_

//-----------------------End of file------------------------------------------
/** @}*/
