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
#include "RelayProcess.h"

//-------------------- private functions declare ----------------------------

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------

//-------------------- private functions declare ----------------------------


//-------------------- public data ------------------------------------------


//-------------------- public functions -------------------------------------
/**
 * @brief       �̵�������
 * @param       ��
 * @retval      ��
 * @notes       ���Ƽ̵����ſ����պϡ�ֹͣ
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
