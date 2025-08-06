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
#include "DataBaseProcess.h"
#include "Debug_Com.h"
#include "IQmath.h"
#include "TimerEncoder.h"
#include "Monitor.h"
#include "AdcProcess.h"


//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------

//-------------------- private functions declare ----------------------------
int8_t Param2Eeprom ( void );
static int8_t Eeprom2Param ( void );
//-------------------- public data ------------------------------------------
ParamStruct_Control ParamHandle_Control = ParamDefault;
//ParamStruct_Solenoid ParamHandle_Solenoid;
//ParamStruct_Steer ParamHandle_Steer;
//ParamStruct_Hub485 ParamHandle_Hub485_Fork;
//ParamStruct_Hub485 ParamHandle_Hub485_Body;
//-------------------- public functions -------------------------------------

void CtrlDataBaseInit ( void )
{
//    ParamHandle_Control.Reg_Gas_Kp = 20;
//    ParamHandle_Control.Reg_Gas_Ki = 120;
//    ParamHandle_Control.Reg_GasMax = _IQ ( 0.8 );
//    ParamHandle_Control.Reg_GasMin = _IQ ( 0.1 );
//    ParamHandle_Control.Reg_MileFormula = _IQ ( 0.14287 );
//    ParamHandle_Control.Reg_GasSpeedBound = _IQ ( 0.1 );
//    ParamHandle_Control.Reg_HighSpdGasMax = _IQ ( 0.8 );
//    ParamHandle_Control.Reg_LowSpdGasMax = _IQ ( 0.5 );
//    ParamHandle_Control.Reg_BreakSpeedBound = _IQ ( 0.02 );
//    ParamHandle_Control.Reg_HighSpdBreakRange = 170;
//    ParamHandle_Control.Reg_LowSpdBreakRange = 300;
//    ParamHandle_Control.Reg_LoadBreakAdd = 30;
//    ParamHandle_Control.Reg_GasBias = _IQ ( 0.3 );
}

void DataBaseInit ( void )
{
    int8_t ret = 0, FLASHWriteTimes = 0, FLASHWriteResult = 0;
    uint16_t u16Value = 0, EepromParamSavingFlag = 0;

    Eeprom_Init();
//    CtrlDataBaseInit();
    EepromParamSavingFlag = ( * ( uint16_t* ) ( ADDRESS_EepromParamSaving_FLAG ) );    //读取FLASH中参数保存标志
    if ( EepromParamSaving_ControlResult == EepromParamSavingFlag )//0)//
    {
        ret = Eeprom2Param ();
        if ( ( ret )
                || ( ParamHandle_Control.Reg_Tab_Ver != Tab_Ver )
                || ( ParamHandle_Control.Reg_Tab_Len != Tab_Len )
                || ( ParamHandle_Control.Reg_Dev_Ver != Dev_Ver ) )
        {
            FaultCtrl.bit.EepromReadErr = 1;
        }
    }
    else
    {
        ret = Param2Eeprom ();
        if ( ret )
        {
            FaultCtrl.bit.EepromWriteErr = 1;
        }
        else
        {
            FLASHWriteTimes = 3;
            EepromParamSavingFlag = EepromParamSaving_ControlResult;
            FLASHWriteResult = Flash_HalfWordWrite ( ADDRESS_EepromParamSaving_FLAG, & ( EepromParamSavingFlag ), 1 );
            while ( FLASHWriteResult && FLASHWriteTimes )
            {
                FLASHWriteTimes--;
                FLASHWriteResult = Flash_HalfWordWrite ( ADDRESS_EepromParamSaving_FLAG, & ( EepromParamSavingFlag ), 1 );
            }
            if ( FLASHWriteResult )
            {
//                FaultCtrl.bit.FLASHWriteErr = 1;
            }
            else
            {
                __disable_irq();
                __NVIC_SystemReset();
            }
        }
    }

//    ParamHandle_Control.Reg_SW_Ver = ( uint32_t ) OS_HOUR + OS_DAY * 100 + OS_MONTH * 10000 + OS_YEAR * 1000000;
    ParamHandle_Control.Reg_SW_Ver = _IQsat ( ( ( uint32_t ) ( SoftVer_A * 1000000 ) + ( SoftVer_B * 1000 ) + SoftVer_C ), 255255255, 0 );
    DI_ReadAll ( &u16Value );
    ParamHandle_Control.Reg_HW_Ver = ( ( ( u16Value >> DI_CH_VID0 ) & 0x0007 ) * 100) + Get_Version_AD ( );

    ParamHandle_Control.Reg_SW_NewApp = _IQsat ( ( * ( uint32_t* ) ( ADDRESS_VerNewApp ) ), 255255255, 0 );
    ParamHandle_Control.Reg_SW_Factory = _IQsat ( ( * ( uint32_t* ) ( ADDRESS_VerFactory ) ), 255255255, 0 );
    ParamHandle_Control.Reg_SW_Boot = _IQsat ( ( * ( uint32_t* ) ( ADDRESS_VerBoot ) ), 255255255, 0 );
    ParamHandle_Control.Reg_SN = HAL_GetUIDw0();
    //ParamHandle_Control.Reg_ENCLines = ENCLines;ParamHandle_Control
}

//从本地变量上传数据到E2PROM
int8_t Param2Eeprom ( void )
{
    int8_t res;

    if ( ( ParamHandle_Control.Reg_Tab_Ver != Tab_Ver ) || ( ParamHandle_Control.Reg_Tab_Len != Tab_Len ) || ( ParamHandle_Control.Reg_Dev_Ver != Dev_Ver ) )
    {
        res = 1;
    }
    else
    {
        ParamHandle_Control.Reg_CRC = Modbus_GetCRC16 ( ( uint8_t * ) &ParamHandle_Control, sizeof ( ParamHandle_Control ) - 4 );
        res = EEPROM_Write ( E2PROM_PAGE_DB_Start, ( uint8_t * ) &ParamHandle_Control, sizeof ( ParamHandle_Control ) );
    }

    return res;

}
//从E2PROM下载数据到本地变量
int8_t Eeprom2Param ( void )
{
    int8_t res;
    res = EEPROM_Read ( E2PROM_PAGE_DB_Start, ( uint8_t * ) &ParamHandle_Control, sizeof ( ParamHandle_Control ) );
    if ( ( ERR_NONE == res )
            && ( ParamHandle_Control.Reg_CRC !=  Modbus_GetCRC16 ( ( uint8_t * ) &ParamHandle_Control, sizeof ( ParamHandle_Control ) - 4 ) ) ) {
        res = ERR_CRC;
    }

    return res;
}
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */


