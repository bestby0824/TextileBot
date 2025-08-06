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
#include "IMU_Com.h"
#include "string.h"
#include "StateCtrl.h"
#include "Xint.h"
#include "CAN_Node.h"

//-------------------- local definitions ------------------------------------
#define IMUCom_FullLife  500                              //IMU通信最大心跳

//-------------------- private data -----------------------------------------
uint8_t g_bIMURxBuf[8];
uint8_t u8IMUDataRxFlag = 0;

//-------------------- private functions declare ----------------------------

//-------------------- public data ------------------------------------------
IMUData g_sImuData = IMUDataDefault;
int16_t  IMULife = -1;                                   //IMU通信心跳
uint8_t u8IMUInitRes = 0xFF;

//-------------------- public functions -------------------------------------
int8_t IMU_AngleCal ( void );
int8_t IMUCom_ProcRecvData ( void );
void IMUData_Receive ( void );

/**
 * @brief       IMU上电初始化
 * @param       无
 * @retval      无
 */
int8_t IMU_AngleCal ( void )
{
    uint8_t bZeroHeadingAngle[5] = {0xFF, 0xAA, 0x01, 0x04, 0x00};   //航向角置零
    uint8_t bSetAngleReference[5] = {0xFF, 0xAA, 0x01, 0x08, 0x00};  //设置角度参考

    if ( Can_Node_send_msg ( 0x50, ( uint8_t * ) &bZeroHeadingAngle, 5 ) != 0 )
    {
        return 1;
    }
    Delay_mS ( 10 );
    if ( Can_Node_send_msg ( 0x50, ( uint8_t * ) &bSetAngleReference, 5 ) != 0 )
    {
        return 1;
    }

    return 0;
}
/**
 * @brief       IMU数据更新---直接上传上位机
 * @param       无
 * @retval      无
 */
int8_t IMUCom_ProcRecvData ( void )
{
    int16_t s16Tmp;
    int32_t s32Tmp;
    static uint16_t u16ZeroCheckCnt = 0, u16ZeroCheckRes = 1;

//    if ( u16ZeroCheckRes )
//    {
//        if ( u8IMUInitRes == 0 )
//        {
//            if ( u16ZeroCheckCnt > 3000 )  //3s回读超时报错
//            {
////                FaultCtrl.bit.IMUInitErr = 1;
//                return Err_IMUInit;
//            }
//            else
//            {
//                u16ZeroCheckCnt++;
//                if ( abs ( g_sImuData.nImuRZ ) < 20 )  //零位回读
//                {
//                    u16ZeroCheckRes = 0;
//                }
//            }
//        }
//        else
//        {
////            FaultCtrl.bit.IMUInitErr = 1;
//            return Err_IMUInit;
//        }
//    }

    if ( u8IMUDataRxFlag == 1 )  //收到数据，进行处理
    {
        u8IMUDataRxFlag = 0;
    }
    else
    {
        return Err_NotData;
    }

    if ( g_bIMURxBuf[1] == 0x51 )
    {
        s16Tmp = ( ( int16_t ) g_bIMURxBuf[3] << 8 ) + g_bIMURxBuf[2];
        g_sImuData.nImuXAcc = _IQsat ( _IQmpy8 ( s16Tmp ), 32767, -32767 );
        s16Tmp = - ( ( ( int16_t ) g_bIMURxBuf[5] << 8 ) + g_bIMURxBuf[4] );
        g_sImuData.nImuYAcc = _IQsat ( _IQmpy8 ( s16Tmp ), 32767, -32767 );
        s16Tmp = ( ( int16_t ) g_bIMURxBuf[7] << 8 ) + g_bIMURxBuf[6];
        g_sImuData.nImuZAcc = _IQsat ( _IQmpy8 ( s16Tmp ), 32767, -32767 );
    }
    else if ( g_bIMURxBuf[1] == 0x52 )
    {
        s16Tmp = ( ( int16_t ) g_bIMURxBuf[3] << 8 ) + g_bIMURxBuf[2];
        g_sImuData.nImuRXSpeed = _IQsat ( _IQmpy8 ( s16Tmp ), 32767, -32767 );
        s16Tmp = - ( ( ( int16_t ) g_bIMURxBuf[5] << 8 ) + g_bIMURxBuf[4] );
        g_sImuData.nImuRXSpeed = _IQsat ( _IQmpy8 ( s16Tmp ), 32767, -32767 );
        s16Tmp = ( ( int16_t ) g_bIMURxBuf[7] << 8 ) + g_bIMURxBuf[6];
        g_sImuData.nImuRXSpeed = _IQsat ( _IQmpy8 ( s16Tmp ), 32767, -32767 );
    }
    else if ( g_bIMURxBuf[1] == 0x53 )
    {
        if ( g_bIMURxBuf[2] == 0x01 )
        {
            s32Tmp = ( int32_t ) ( ( g_bIMURxBuf[7] << 24 )  + ( g_bIMURxBuf[6] << 16 ) + ( g_bIMURxBuf[5] << 8 ) + g_bIMURxBuf[4] );
            s16Tmp = s32Tmp / 10;
            g_sImuData.nImuRX = _IQsat ( _IQ15div ( s16Tmp, 18000 ), 32767, -32767 );
        }
        else if ( g_bIMURxBuf[2] == 0x02 )
        {
            s32Tmp = ( int32_t ) ( ( g_bIMURxBuf[7] << 24 )  + ( g_bIMURxBuf[6] << 16 ) + ( g_bIMURxBuf[5] << 8 ) + g_bIMURxBuf[4] );
            s16Tmp = s32Tmp / 10;
            g_sImuData.nImuRY = _IQsat ( _IQ15div ( s16Tmp, 18000 ), 32767, -32767 );
        }
        else if ( g_bIMURxBuf[2] == 0x03 )
        {
            s32Tmp = ( int32_t ) ( ( g_bIMURxBuf[7] << 24 )  + ( g_bIMURxBuf[6] << 16 ) + ( g_bIMURxBuf[5] << 8 ) + g_bIMURxBuf[4] );
            s16Tmp = s32Tmp / 10;
            g_sImuData.nImuRZ = _IQsat ( _IQ15div ( s16Tmp, 18000 ), 32767, -32767 );
            DbugCom_Tx[4] =  g_bIMURxBuf[4];
            DbugCom_Tx[5] =  g_bIMURxBuf[5];
            DbugCom_Tx[6] =  g_bIMURxBuf[6];
            DbugCom_Tx[7] =  g_bIMURxBuf[7];
        }
    }
    else
    {
        ;
    }

    return Err_None;
}
/**
 * @brief       IMU数据接收
 * @param       无
 * @retval      无
 */
void IMUData_Receive ( void )
{
    static uint32_t Rx_ID;
    static uint8_t Rx_Buffer[8];

    Rx_ID = Can_Node_receive_msg ( Rx_Buffer );
    if ( Reg_IMUState == Rx_ID )
    {
        __disable_irq();         //关闭总中断
        memcpy ( &g_bIMURxBuf, &Rx_Buffer, 8 );
        __enable_irq();          // 开启总中断
        IMULife = IMUCom_FullLife;                  //收到IMU数据后将心跳等待时间放到最大
        u8IMUDataRxFlag = 1;
    }
}

//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


