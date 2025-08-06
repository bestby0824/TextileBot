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
*
*/
//------------------------------------------------------------------------------

//-------------------- include files ----------------------------------------
#include "DataBaseProcess.h"
#include "IQmath.h"
#include "HostProcess.h"
#include "Monitor.h"
#include "Dido.h"
#include "ADC.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------

//-------------------- private functions declare ----------------------------
int8_t Param2Eeprom ( ParamStruct_Steer Param );
static int8_t Eeprom2Param ( void );
static void SteerDataBaseInit ( void );
//-------------------- public data ------------------------------------------
ParamStruct_Steer ParamHandle_Steer = ParamDefault;

//-------------------- public functions -------------------------------------
/*! \fn				void DataBaseInit( void )
 *  \brief 		参数初始化
 *  \param 		none
 *  \return 	none
 */
void DataBaseInit ( void )
{
    int8_t res = 0, FLASHWriteTimes = 0, FLASHWriteResult = 0;
    uint16_t u16HardVer_A = 0, u16HardVer_B = 0, u16Value;
    uint16_t EepromParamSavingFlag = 0;

    Eeprom_Init();
    SteerDataBaseInit();
    EepromParamSavingFlag = ( * ( uint16_t* ) ( ADDRESS_EepromParamSaving_FLAG ) );    //读取FLASH中参数保存标志
    if ( EepromParamSaving_SteerResult == EepromParamSavingFlag )    //E2P中有参数，则读取E2P中参数；反正，将默认参数写入E2P
    {
        res = Eeprom2Param ();
        if ( res )
        {
            SetE2prom_Fault ( );
        }
        else
        {
            if ( ( ParamHandle_Steer.Reg_Tab_Ver != Tab_Ver ) || ( ParamHandle_Steer.Reg_Tab_Len != Tab_Len ) || \
                    ( ParamHandle_Steer.Reg_Dev_Ver != Dev_Ver ) )    //设备编号、数据长度或者数据库编号不对，判定为数据库版本异常
            {
                SetDBVersion_Fault ( );
            }
        }
    }
    else
    {
        res = Param2Eeprom ( ParamHandle_Steer );
        if ( res )
        {
            SetE2prom_Fault ( );
        }
        else
        {
            FLASHWriteTimes = 3;
            EepromParamSavingFlag = EepromParamSaving_SteerResult;
            FLASHWriteResult = Flash_HalfWordWrite ( ADDRESS_EepromParamSaving_FLAG, & ( EepromParamSavingFlag ), 1 );
            while ( FLASHWriteResult && FLASHWriteTimes )
            {
                FLASHWriteTimes--;
                FLASHWriteResult = Flash_HalfWordWrite ( ADDRESS_EepromParamSaving_FLAG, & ( EepromParamSavingFlag ), 1 );
            }
            if ( FLASHWriteResult )
            {
                SetE2prom_Fault ( );
            }
            else
            {
                __disable_irq();
                __NVIC_SystemReset();
            }
        }
    }

    ParamHandle_Steer.Reg_SW_Ver = _IQsat ( ( ( uint32_t ) ( SoftVer_A * 1000000 ) + ( SoftVer_B * 1000 ) + SoftVer_C ), 255255255, 0 );
    DI_ReadAll ( &u16Value );
    u16HardVer_A = u16Value & 0x0007;
    u16Value = ADC_GetValue ( ADC_CH_HW_Ver );
    u16HardVer_B = HardVerCrc ( u16Value );
    ParamHandle_Steer.Reg_HW_Ver = _IQsat ( ( uint32_t ) ( u16HardVer_A * 100 + u16HardVer_B ), 712, 0 );  //A:0-7,B:0-12

    ParamHandle_Steer.Reg_SW_NewApp = _IQsat ( ( * ( uint32_t* ) ( ADDRESS_VerNewApp ) ), 255255255, 0 );
    ParamHandle_Steer.Reg_SW_Factory = _IQsat ( ( * ( uint32_t* ) ( ADDRESS_VerFactory ) ), 255255255, 0 );
    ParamHandle_Steer.Reg_SW_Boot = _IQsat ( ( * ( uint32_t* ) ( ADDRESS_VerBoot ) ), 255255255, 0 );
    ParamHandle_Steer.Reg_SN = HAL_GetUIDw0();
}
/*! \fn				void ParameterUpData( ParamStruct_Steer Param )
 *  \brief 		更新参数表
 *  \param 		none
 *  \return 	0：更新成功，1：更新失败
 */
int DataUpParameter ( void )
{
    int8_t res;

    if ( Eeprom2Param ( ) == 0 )
    {
        res = 0;
    }
    else
    {
        SetE2prom_Fault ( );  //写入失败
        res = 1;
    }

    return res;
}
//-------------------- private functions ------------------------------------
//从本地变量上传数据到E2PROM
int8_t Param2Eeprom ( ParamStruct_Steer Param )
{
    int8_t res;

    if ( ( Param.Reg_Tab_Ver != Tab_Ver ) || ( Param.Reg_Tab_Len != Tab_Len ) || ( Param.Reg_Dev_Ver != Dev_Ver ) )
    {
        res = 1;
    }
    else
    {
        Param.Reg_CRC = Modbus_GetCRC16 ( ( uint8_t * ) &Param, sizeof ( Param ) - 2 );
        res = EEPROM_Write ( E2PROM_PAGE_DB_Start, ( uint8_t * ) &Param, sizeof ( Param ) );
    }

    return res;

}
//从E2PROM下载数据到本地变量
static int8_t Eeprom2Param ( void )
{
    int8_t res;
    res = EEPROM_Read ( E2PROM_PAGE_DB_Start, ( uint8_t * ) &ParamHandle_Steer, sizeof ( ParamHandle_Steer ) );
    if ( ( ERR_NONE == res )
            && ( ParamHandle_Steer.Reg_CRC !=  Modbus_GetCRC16 ( ( uint8_t * ) &ParamHandle_Steer, sizeof ( ParamHandle_Steer ) - 2 ) ) ) {
        res = ERR_CRC;
    }

    return res;
}
/*! \fn				static void SteerDataBaseInit( void )
 *  \brief 		非版本类参数初始化
 *  \param 		none
 *  \return 	none
 */
static void SteerDataBaseInit ( void )
{
    ParamHandle_Steer.Reg_PI_POS_KP = _IQ ( 24 );
    ParamHandle_Steer.Reg_PI_POS_KI = _IQ ( 0 );
    ParamHandle_Steer.Reg_PI_SPD_KP = _IQ ( 20 );
    ParamHandle_Steer.Reg_PI_SPD_KI = _IQ ( 0.002 );
    ParamHandle_Steer.Reg_PI_ID_KP = _IQ ( 0.04 );
    ParamHandle_Steer.Reg_PI_ID_KI = _IQ ( 0.025 );
    ParamHandle_Steer.Reg_PI_IQ_KP = _IQ ( 0.04 );
    ParamHandle_Steer.Reg_PI_IQ_KI = _IQ ( 0.025 );
    ParamHandle_Steer.Reg_AngleMax = _IQ ( 0.1667 );
    ParamHandle_Steer.Reg_SpeedMax = 1500;
    ParamHandle_Steer.Reg_CurrentMax = 1000;
    ParamHandle_Steer.Reg_IdIqMax = _IQ ( 0.5 );
    ParamHandle_Steer.Reg_FaultCheckEnable = 0x00000FFF;
    ParamHandle_Steer.Reg_CtrlDeadZone = _IQ ( 0.00138 );
    ParamHandle_Steer.Reg_PosDeadZone = _IQ ( 0.00138 );
    ParamHandle_Steer.Reg_PosMax = _IQ ( 0.5 );
    ParamHandle_Steer.Reg_PosMin = _IQ ( 0.25 );
    ParamHandle_Steer.Reg_TorqueMax = 0;
    ParamHandle_Steer.Reg_TorqueMin = 0;
    ParamHandle_Steer.Reg_CaliTimeOut = 0;
    ParamHandle_Steer.Reg_RunTimeOut = 12000;
    ParamHandle_Steer.Reg_RC_Spdmax = 0;
}
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


