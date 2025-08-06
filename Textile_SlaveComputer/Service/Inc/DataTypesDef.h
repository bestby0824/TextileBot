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

//-----------------------------------------------------------------------------
#ifndef _DataTypesDef_H_
#define _DataTypesDef_H_

#include "stdint.h"

typedef union
{
    uint8_t bit[2];
    int16_t all;
} S16HalfWordDef;

typedef union
{
    uint8_t bit[4];
    int32_t all;
} S32WordDef;

typedef union
{
    uint8_t bit[2];
    uint16_t all;
} U16HalfWordDef;

typedef union {
    uint8_t bytes[4];    // 4字节数组
    uint32_t value;      // 32位整数
} U32WordDef;

// 32位大小端转换宏
#define ENDIAN_SWAP32(value) \
    ((((value) & 0x000000FFU) << 24) | \
     (((value) & 0x0000FF00U) << 8)  | \
     (((value) & 0x00FF0000U) >> 8)  | \
     (((value) & 0xFF000000U) >> 24))

// 16位大小端转换宏
#define ENDIAN_SWAP16(value) \
    ((((value) & 0x00FFU) << 8) | \
     (((value) & 0xFF00U) >> 8))
#endif