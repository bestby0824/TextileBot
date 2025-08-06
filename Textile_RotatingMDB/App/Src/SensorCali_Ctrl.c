/**
* ��Ȩ����(C)
*
* ********
*
* @file
* @brief
* @details
* @author MUNIU
* @version 1.0.0
* @date xxxx-xx-xx
* @par Description:
* @par Change History:
*
* <����> | <�汾> | <����> | <����>
*
* xxxx/xx/xx | 1.0.0 | MUNIU | �����ļ�
*
*/
//------------------------------------------------------------------------------

//-------------------- include files ----------------------------------------
#include "SensorCali_Ctrl.h"
#include "DataBaseProcess.h"
#include "HostProcess.h"
#include "MotorCmd_Ctrl.h"
#include "string.h"
#include "Monitor.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
SensorCali_TypeDef SensorCali_Handle;
RegDef_SensorCaliFB SensorCaliFB;

//-------------------- private functions declare ----------------------------

//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
/**
* @brief       �������궨����
 * @param       ��
 * @retval      ��
 */
void SensorCali_Ctrl ( void )
{
    static uint16_t u16_SensorCaliCmd_Last;
    static uint16_t u16Cnt_ResetDelay = 0, u16Cnt_WriteTime = 0;

    if ( RegReal_SensorCaliCmd.u16SN != u16_SensorCaliCmd_Last ) {  //��ָ��
        u16_SensorCaliCmd_Last = RegReal_SensorCaliCmd.u16SN;
        switch ( RegReal_SensorCaliCmd.u16Dev ) {
            case Sensor_MotorCoder: {
                if ( RegReal_SensorCaliCmd.u16EN == 0x0001 ) {  //����������궨ʹ��
                    SensorCali_Handle.u16MotorCaliEnable = 1;
                } else {
                    SensorCali_Handle.u16MotorCaliEnable = 0;
                }
            }
            break;
            default:
                break;                    
        }
    }
    
    if ( SensorCali_Handle.u16MotorCaliEnable ) {
        u16StopFlag = 1;
        SensorCaliFB.u16Dev = RegReal_SensorCaliCmd.u16Dev;
        SensorCaliFB.u16SN = RegReal_SensorCaliCmd.u16SN;
        SensorCaliFB.u16EndofCtrl = 1;  //�궨��
        if ( u16Cnt_ResetDelay < 100 ) u16Cnt_ResetDelay++;
        else {
            ParamHandle_Steer.Reg_ReverseFlag = 0;
            if ( Param2Eeprom ( ParamHandle_Steer ) == 0 ) {
                SoftReset( );
            } else {
                if ( u16Cnt_WriteTime < 3 ) {
                    u16Cnt_WriteTime++;
                    u16Cnt_ResetDelay = 0;
                } else {
                    SensorCaliFB.u16EndofCtrl = 2;  //�궨ʧ��
                    SensorCali_Handle.u16MotorCaliEnable = 0;
                    u16Cnt_ResetDelay = 0;
                    u16Cnt_WriteTime = 0;
                }
            }
        }
    } else {
        u16StopFlag = 0;
        u16Cnt_ResetDelay = 0;
        u16Cnt_WriteTime = 0;

        if ( Foc_EN_Flag ) {
            SensorCaliFB.u16EndofCtrl = 0;  //�궨�ɹ�
        } else {
            SensorCaliFB.u16EndofCtrl = 2;  //�궨ʧ��
        }
    }
    
    if ( g_u16SensorCaliStep == 1 ) g_u16SensorCaliStep = 2;
}
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


