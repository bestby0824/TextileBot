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

//-----------------------------------------------------------------------------
#ifndef _WindowProcess_H_
#define _WindowProcess_H_

#include "AngleMath.h"
#include "CoderTama.h"

#define RelayCtrl_ID                1
#define RelayCtrl_Addr              0
#define RelayCtrl_Read              3
#define RelayCtrl_Write             6

//-------------------- public data ------------------------------------------
typedef enum
{
    STOP = 0,
    OPEN,
    CLOSE,
    RELAY_NONE,
}RELAY_STA;


//-------------------- public functions -------------------------------------

void RelayCtrl ( uint8_t Feat, RELAY_STA Cmd );

#endif  /*End*/

//-----------------------End of file------------------------------------------
/** @}*/




