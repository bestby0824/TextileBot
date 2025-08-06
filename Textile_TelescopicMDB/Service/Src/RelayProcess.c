/**
* 版权所有(C)
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
* <日期> | <版本> | <作者> | <描述>
*
* xxxx/xx/xx | 1.0.0 | MUNIU | 创建文件
*
*/
//------------------------------------------------------------------------------

//-------------------- include files ----------------------------------------
#include "RelayProcess.h"

//-------------------- private functions declare ----------------------------

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------

//-------------------- private functions declare ----------------------------


//-------------------- public data ------------------------------------------


//-------------------- public functions -------------------------------------
/**
 * @brief       继电器服务
 * @param       无
 * @retval      无
 * @notes       控制继电器张开、闭合、停止
 */
void RelayCtrl ( uint8_t Feat, RELAY_STA Cmd )
{
    ModbusTypeDef ModbusVars;
    
    ModbusVars.Id = ( uint8_t ) RelayCtrl_ID;
    ModbusVars.Featurecodes = Feat;
    ModbusVars.RegAddr = ( uint16_t ) RelayCtrl_Addr;
    
    if ( RelayCtrl_Read == Feat ) {
        ModbusVars.RegData = 0x0001;
        RelayCtrl_Send ( &ModbusVars );
    } else if ( RelayCtrl_Write == Feat ) {
        switch ( Cmd ) {
            case OPEN:
                ModbusVars.RegData = ( uint16_t ) OPEN;
                RelayCtrl_Send ( &ModbusVars );
                break;
            case CLOSE:
                ModbusVars.RegData = ( uint16_t ) CLOSE;
                RelayCtrl_Send ( &ModbusVars );
                break;
            case STOP:
                ModbusVars.RegData = ( uint16_t ) STOP;
                RelayCtrl_Send ( &ModbusVars );
                break;
            default:
                break;
        }
    } else {
        ;
    }

}
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */
