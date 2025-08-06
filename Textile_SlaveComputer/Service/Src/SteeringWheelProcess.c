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
#include "SteeringWheelProcess.h"
#include "Debug_Com.h"
#include "DataBaseProcess.h"
#include "string.h"
#include "Monitor.h"
//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
static uint8_t RS485_TxBuf_SteerWheel[RS485_TxBufLength_SteerWheel];
static uint8_t RS485_RxBuf_SteerWheel[RS485_RxBufLength_SteerWheel];
static uint16_t RS485_RxLen_SteerWheel, RS485_TxLen_SteerWheel;
static uint8_t SteerWheelVersionGet[64];
//-------------------- private functions declare ----------------------------
static void FillBufHead ( void );
static uint8_t IsRightCheck ( uint8_t *prbuf, uint16_t rleg );

//-------------------- public data ------------------------------------------
uint16_t g_u16SteeringWheelAbsPos = 16384;
uint16_t g_u16SteeringWheelPressure = 0;
uint16_t Pos12bit = 0, u16Crc = 0;
uint8_t  SteerPreSta = 0;           //ת��������ѹ����ָ�����ѹ״̬���յ���ѹ����ʱ������ѹ�������յ�����ѹ���źź���λ�ÿ��� 0x10��ʾ
//-------------------- public functions -------------------------------------
/**
* @brief  ת��巢��λ����������֡(û��)
  * @param  none
  * @retval None
  */
void SteeringWheel_SendPos ( _iq PosSend )
{
    //���ʣ�ಿ�ֹ�����������֡

    Pos12bit = _IQmpy ( _IQsat ( PosSend, AngleMax, AngleMin ), 4096 ); //_IQ(0.5)ת���ֻ�����ת��Ƕ����ƣ�����50�㣻
    RS485_TxLen_SteerWheel = 11;
    FillBufHead();
    RS485_TxBuf_SteerWheel[Index_PayLoad] = Reg_JointTargetPos_Addr;
    RS485_TxBuf_SteerWheel[Index_PayLoad + 1] = 2;
    RS485_TxBuf_SteerWheel[Index_PayLoad + 2] = ( Pos12bit & 0x00FF );
    RS485_TxBuf_SteerWheel[Index_PayLoad + 3] = ( ( Pos12bit >> 8 ) & 0x00FF );
    u16Crc = Modbus_GetCRC16 ( RS485_TxBuf_SteerWheel, RS485_TxLen_SteerWheel - 2 );
    RS485_TxBuf_SteerWheel[RS485_TxLen_SteerWheel - 2] = ( u16Crc >> 8 );
    RS485_TxBuf_SteerWheel[RS485_TxLen_SteerWheel - 1] = u16Crc;
    RS485_SteeringWheel_Send ( RS485_TxBuf_SteerWheel, RS485_TxLen_SteerWheel );
}
/**
  * @brief  (û��)
  * @param  none
  * @retval None
  */
void SteeringWheel_SetStopFree ( void )
{
    uint16_t u16Crc = 0;
    RS485_TxLen_SteerWheel = 10;
    FillBufHead();
    RS485_TxBuf_SteerWheel[Index_PayLoad] = Reg_StopCmd_Addr;
    RS485_TxBuf_SteerWheel[Index_PayLoad + 1] = 2;
    RS485_TxBuf_SteerWheel[Index_PayLoad + 2] = 2; //1����ֹͣ��2����ֹͣ

    u16Crc = Modbus_GetCRC16 ( RS485_TxBuf_SteerWheel, RS485_TxLen_SteerWheel - 2 );
    RS485_TxBuf_SteerWheel[RS485_TxLen_SteerWheel - 2] = ( u16Crc >> 8 );
    RS485_TxBuf_SteerWheel[RS485_TxLen_SteerWheel - 1] = u16Crc;

    RS485_SteeringWheel_Send ( RS485_TxBuf_SteerWheel, RS485_TxLen_SteerWheel );
}
/**
  * @brief  (û��)
  * @param  none
  * @retval None
  */
void SteeringWheel_DogFeed ( void )
{
    uint16_t u16Crc = 0;
    RS485_TxLen_SteerWheel = 11;
    FillBufHead();
    RS485_TxBuf_SteerWheel[Index_PayLoad] = Reg_DogFeedMs_Addr;
    RS485_TxBuf_SteerWheel[Index_PayLoad + 1] = 2;
    RS485_TxBuf_SteerWheel[Index_PayLoad + 2] = ( DogOutTimeMs & 0x00FF );
    RS485_TxBuf_SteerWheel[Index_PayLoad + 3] = ( ( DogOutTimeMs >> 8 ) & 0x00FF );

    u16Crc = Modbus_GetCRC16 ( RS485_TxBuf_SteerWheel, RS485_TxLen_SteerWheel - 2 );
    RS485_TxBuf_SteerWheel[RS485_TxLen_SteerWheel - 2] = ( u16Crc >> 8 );
    RS485_TxBuf_SteerWheel[RS485_TxLen_SteerWheel - 1] = u16Crc;

    RS485_SteeringWheel_Send ( RS485_TxBuf_SteerWheel, RS485_TxLen_SteerWheel );
}
/**
  * @brief  (û��)
  * @param  none
  * @retval None
  */
void SteeringWheel_GetSta ( void )
{
    uint16_t u16Crc = 0;
    RS485_TxLen_SteerWheel = 9;
    FillBufHead();
    RS485_TxBuf_SteerWheel[Index_OPT] = MB_REG_READ;
    RS485_TxBuf_SteerWheel[Index_PayLoad] = Reg_Fault1_Addr;
    RS485_TxBuf_SteerWheel[Index_PayLoad + 1] = 6;

    u16Crc = Modbus_GetCRC16 ( RS485_TxBuf_SteerWheel, RS485_TxLen_SteerWheel - 2 );
    RS485_TxBuf_SteerWheel[RS485_TxLen_SteerWheel - 2] = ( u16Crc >> 8 );
    RS485_TxBuf_SteerWheel[RS485_TxLen_SteerWheel - 1] = u16Crc;

    RS485_SteeringWheel_Send ( RS485_TxBuf_SteerWheel, RS485_TxLen_SteerWheel );
}
/**
  * @brief  (û��)
  * @param  none
  * @retval None
  */
void SteeringWheel_GetAbsPos ( void )
{
    uint16_t u16Crc = 0;
    RS485_TxLen_SteerWheel = 9;
    FillBufHead();
    RS485_TxBuf_SteerWheel[Index_OPT] = MB_REG_READ;
    RS485_TxBuf_SteerWheel[Index_PayLoad] = Reg_PositionOfJointMode_Addr;
    RS485_TxBuf_SteerWheel[Index_PayLoad + 1] = 2;

    u16Crc = Modbus_GetCRC16 ( RS485_TxBuf_SteerWheel, RS485_TxLen_SteerWheel - 2 );
    RS485_TxBuf_SteerWheel[RS485_TxLen_SteerWheel - 2] = ( u16Crc >> 8 );
    RS485_TxBuf_SteerWheel[RS485_TxLen_SteerWheel - 1] = u16Crc;

    RS485_SteeringWheel_Send ( RS485_TxBuf_SteerWheel, RS485_TxLen_SteerWheel );
}
/**
  * @brief  (û��)
  * @param  none
  * @retval None
  */
void SteeringWheel_GetVersion ( void )
{
    uint16_t u16Crc = 0;
    RS485_TxLen_SteerWheel = 9;
    FillBufHead();
    RS485_TxBuf_SteerWheel[Index_OPT] = MB_REG_READ;
    RS485_TxBuf_SteerWheel[Index_PayLoad] = Reg_VersionGetCmd;
    RS485_TxBuf_SteerWheel[Index_PayLoad + 1] = 5;

    u16Crc = Modbus_GetCRC16 ( RS485_TxBuf_SteerWheel, RS485_TxLen_SteerWheel - 2 );
    RS485_TxBuf_SteerWheel[RS485_TxLen_SteerWheel - 2] = ( u16Crc >> 8 );
    RS485_TxBuf_SteerWheel[RS485_TxLen_SteerWheel - 1] = u16Crc;

    RS485_SteeringWheel_Send ( RS485_TxBuf_SteerWheel, RS485_TxLen_SteerWheel );
}
/**
  * @brief  (û��)
  * @param  none
  * @retval None
  */
void SteeringWheel_GetPressure ( void )
{
    uint16_t u16Crc = 0;
    RS485_TxLen_SteerWheel = 9;
    FillBufHead();
    RS485_TxBuf_SteerWheel[Index_OPT] = MB_REG_READ;
    RS485_TxBuf_SteerWheel[Index_PayLoad] = Reg_GetPressure_Addr;
    RS485_TxBuf_SteerWheel[Index_PayLoad + 1] = 3;

    u16Crc = Modbus_GetCRC16 ( RS485_TxBuf_SteerWheel, RS485_TxLen_SteerWheel - 2 );
    RS485_TxBuf_SteerWheel[RS485_TxLen_SteerWheel - 2] = ( u16Crc >> 8 );
    RS485_TxBuf_SteerWheel[RS485_TxLen_SteerWheel - 1] = u16Crc;

    RS485_SteeringWheel_Send ( RS485_TxBuf_SteerWheel, RS485_TxLen_SteerWheel );
}
/**
  * @brief  (û��)
  * @param  none
  * @retval None
  */
void SteeringWheel_FaultAlarmClear ( void )
{
    uint16_t u16Crc = 0;
    RS485_TxLen_SteerWheel = 10;
    FillBufHead();
    RS485_TxBuf_SteerWheel[Index_PayLoad] = Reg_FaultAlarmClear_Addr;
    RS485_TxBuf_SteerWheel[Index_PayLoad + 1] = 1;
    RS485_TxBuf_SteerWheel[Index_PayLoad + 2] = 1; //1-���Ͼ�������

    u16Crc = Modbus_GetCRC16 ( RS485_TxBuf_SteerWheel, RS485_TxLen_SteerWheel - 2 );
    RS485_TxBuf_SteerWheel[RS485_TxLen_SteerWheel - 2] = ( u16Crc >> 8 );
    RS485_TxBuf_SteerWheel[RS485_TxLen_SteerWheel - 1] = u16Crc;

    RS485_SteeringWheel_Send ( RS485_TxBuf_SteerWheel, RS485_TxLen_SteerWheel );
}
/**
  * @brief  (û��)
  * @param  none
  * @retval None
  */
#if 0
uint8_t SteeringWheel_PktRev ( void )
{
    uint8_t ret = 0;
    ret = RS485_SteeringWheel_Receive ( RS485_RxBuf_SteerWheel, &RS485_RxLen_SteerWheel );
    if ( ret )
    {
        if ( IsRightCheck ( RS485_RxBuf_SteerWheel, RS485_RxLen_SteerWheel ) || ( * ( uint16_t  * ) RS485_RxBuf_SteerWheel == BOOTHEAD ) || ( * ( uint16_t  * ) RS485_RxBuf_SteerWheel == PARAHEAD ) )
        {
            if ( u8BootFlg_SteeringWheel == 0 )
            {
                switch ( RS485_RxBuf_SteerWheel[Index_OPT] )
                {
                case MB_REG_READ:
                {
                    switch ( RS485_RxBuf_SteerWheel[Index_LEN] )
                    {
                    case 16:
                    {
                        switch ( RS485_RxBuf_SteerWheel[Index_PayLoad] )
                        {
                        case Reg_VersionGetCmd:
                        {
                            memcpy ( &SteerWheelVersionGet[0], &RS485_RxBuf_SteerWheel[6], 8 );
                        }
                        break;
                        case ( Reg_VersionGetCmd + 1 ) :
                        {
                            memcpy ( &SteerWheelVersionGet[8], &RS485_RxBuf_SteerWheel[6], 8 );
                        }
                        break;
                        case ( Reg_VersionGetCmd + 2 ) :
                        {
                            memcpy ( &SteerWheelVersionGet[16], &RS485_RxBuf_SteerWheel[6], 8 );
                        }
                        break;
                        case ( Reg_VersionGetCmd + 3 ) :
                        {
                            memcpy ( &SteerWheelVersionGet[24], &RS485_RxBuf_SteerWheel[6], 8 );
                        }
                        break;
                        case ( Reg_VersionGetCmd + 4 ) :
                        {
                            memcpy ( &SteerWheelVersionGet[32], &RS485_RxBuf_SteerWheel[6], 2 );
                        }
                        break;
                        default:
                            break;
                        }
                        memcpy ( &ParamHandle_Steer, &SteerWheelVersionGet, sizeof ( ParamHandle_Steer ) );
                    }
                    break;
                    case 14://Reg_Fault1_Addr:
                    {
//                    if ( RS485_RxBuf_SteerWheel[Index_PayLoad + 1] == 4 )
//                        {
                            Fault_Handle1.Fault_Steer.all = ( uint32_t ) RS485_RxBuf_SteerWheel[Index_PayLoad + 1] + ( ( uint32_t ) RS485_RxBuf_SteerWheel[Index_PayLoad + 2] << 8 ) \
                                                      + ( ( uint32_t ) RS485_RxBuf_SteerWheel[Index_PayLoad + 3] << 16 ) + ( ( uint32_t ) RS485_RxBuf_SteerWheel[Index_PayLoad + 4] << 24 );
                            //����ͣ��������
//                        }
                    }
                    break;
                    case 10://Reg_PositionOfJointMode_Addr:
                    {
                        g_u16SteeringWheelAbsPos = ( uint16_t ) RS485_RxBuf_SteerWheel[Index_PayLoad + 1] + ( ( uint32_t ) RS485_RxBuf_SteerWheel[Index_PayLoad + 2] << 8 );
                        SteerPreSta = RS485_RxBuf_SteerWheel[Index_PayLoad + 3];
                    }
                    break;
                    case 11://Reg_PositionOfJointMode_Addr:
                    {
                        g_u16SteeringWheelPressure = ( uint16_t ) RS485_RxBuf_SteerWheel[Index_PayLoad + 1] + ( ( uint32_t ) RS485_RxBuf_SteerWheel[Index_PayLoad + 2] << 8 );
                    }
                    break;
                    case 13://Reg_PositionOfJointMode_Addr:
                    {
                        ParamHandle_Steer.Reg_HW_Ver = RS485_RxBuf_SteerWheel[10];
                        ParamHandle_Steer.Reg_SW_Ver = _IQsat ( ( ( uint32_t ) ( ParamHandle_Steer.Reg_HW_Ver * 1000000 ) + ( ( RS485_RxBuf_SteerWheel[6] - 3 ) * 1000 ) + RS485_RxBuf_SteerWheel[7] ), 255255255, 0 );
                    }
                    break;
                    }
                }
                break;
                case MB_REG_WRITE:
                {

                }
                break;
                }
                return 0;
            }
            else {
                //͸��ģʽ
                Uart_Dbug_Send ( RS485_RxBuf_SteerWheel, RS485_RxLen_SteerWheel );
                return 0;
            }
        }
        else {
            return 2;
        }
    } else {
        return 1;
    }
}
#endif
/**
  * @brief  
  * @param  none
  * @retval None
  */
static void FillBufHead ( void )
{
    RS485_TxBuf_SteerWheel[Index_HEAD0] = ( uint8_t ) ( EWAY_MODBUS_HEAD >> 8 );
    RS485_TxBuf_SteerWheel[Index_HEAD1] = ( uint8_t ) EWAY_MODBUS_HEAD;
    RS485_TxBuf_SteerWheel[Index_ID] = 1;
    RS485_TxBuf_SteerWheel[Index_LEN] = RS485_TxLen_SteerWheel;
    RS485_TxBuf_SteerWheel[Index_OPT] = MB_REG_WRITE;
}
/**
  * @brief  
  * @param  none
  * @retval None
  */
static uint8_t IsRightCheck ( uint8_t *prbuf, uint16_t rleg )
{
    uint8_t ret = 0;
    uint16_t u16Hander;         //֡ͷ
    uint8_t  u8Addr;            //��ַ
    uint16_t u16Crc;            //crcУ��ֵ
    uint8_t  u8len;

    if ( ( rleg < 5 ) || ( rleg > 255 ) ) return 0; //����̫�̻�̫��ֱ�ӷ���,��ֹ�ڴ����

    u16Hander  = prbuf[Index_HEAD0] << 8 | prbuf[Index_HEAD1];
    u8Addr     = prbuf[Index_ID];
    u8len      = prbuf[Index_LEN];

    if ( u16Hander != EWAY_MODBUS_HEAD )
    {
        ret = 0;
    }
    else if ( ( u8Addr != 1 ) && ( u8Addr != EWYA_MODBUS_BRC_ADDR ) ) //����֡��ַ������ڱ�����ַ���߹㲥��ַ
    {
        ret = 0;
    }
    else
    {
        u16Crc = prbuf[Index_Crc] << 8 | prbuf[Index_Crc + 1];

        if ( u16Crc == Modbus_GetCRC16 ( prbuf, Index_Crc ) )
        {
            ret = 1;
        }
        else
        {
            ret = 0;
        }
    }

    return ret;
}
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


