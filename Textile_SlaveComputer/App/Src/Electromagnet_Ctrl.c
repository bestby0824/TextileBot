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
#include "Electromagnet_Ctrl.h"
//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------


//-------------------- private functions declare ----------------------------

//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
/*
* @brief       ������򿪺���
* @param       ��
* @retval      ��
*
*/
void EMCtrlON(uint16_t NPNSta)
{
        DoSetMagnetPower(NPNSta);
}