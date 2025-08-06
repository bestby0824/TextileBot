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
#include "Electromagnet_Ctrl.h"
//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------


//-------------------- private functions declare ----------------------------

//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
/*
* @brief       电磁铁打开函数
* @param       无
* @retval      无
*
*/
void EMCtrlON(uint16_t NPNSta)
{
        DoSetMagnetPower(NPNSta);
}