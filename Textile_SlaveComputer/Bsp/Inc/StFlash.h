
//------------------------------------------------------------------------------

#ifndef _ST_FLASH_H_
#define _ST_FLASH_H_
//-------------------- include files ----------------------------------------
#include "Driver.h"

//-------------------- public definitions -----------------------------------
#define STM32_FLASH_BASE 0x08000000     //STM32 FLASH起始地址
#define FLASH_WAITETIME  50000          //FLASH等待超时时间

#define FLASH_LENTH        ((uint32_t)0x100000) //1MB

//FLASH 各扇区的起始地址
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) || defined(STM32F412Zx) ||\
    defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx)

#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000)     //扇区0起始地址,  16 Kbytes
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000)     //扇区0起始地址,  16 Kbytes
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000)     //扇区0起始地址,  16 Kbytes
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000)     //扇区0起始地址,  16 Kbytes
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000)     //扇区0起始地址,  64 Kbytes
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000)     //扇区0起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000)     //扇区0起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000)     //扇区0起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000)     //扇区0起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000)     //扇区0起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000)     //扇区0起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000)     //扇区0起始地址, 128 Kbytes
#endif

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx)|| defined(STM32F439xx) ||\
    defined(STM32F469xx) || defined(STM32F479xx)


//#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000)     //扇区0起始地址, 16 Kbytes
//#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000)     //扇区0起始地址, 16 Kbytes
//#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000)     //扇区0起始地址, 16 Kbytes
//#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000)     //扇区0起始地址, 16 Kbytes
//#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000)     //扇区0起始地址, 64 Kbytes
//#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000)     //扇区0起始地址, 128 Kbytes
//#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000)     //扇区0起始地址, 128 Kbytes
//#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000)     //扇区0起始地址, 128 Kbytes
//#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000)     //扇区0起始地址, 128 Kbytes
//#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000)     //扇区0起始地址, 128 Kbytes
//#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000)     //扇区0起始地址, 128 Kbytes
//#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000)     //扇区0起始地址, 128 Kbytes
//#define ADDR_FLASH_SECTOR_12    ((uint32_t)0x08100000)     //扇区0起始地址, 16 Kbytes
//#define ADDR_FLASH_SECTOR_13    ((uint32_t)0x08104000)     //扇区0起始地址, 16 Kbytes
//#define ADDR_FLASH_SECTOR_14    ((uint32_t)0x08108000)     //扇区0起始地址, 16 Kbytes
//#define ADDR_FLASH_SECTOR_15    ((uint32_t)0x0810C000)     //扇区0起始地址, 16 Kbytes
//#define ADDR_FLASH_SECTOR_16    ((uint32_t)0x08110000)     //扇区0起始地址, 64 Kbytes
//#define ADDR_FLASH_SECTOR_17    ((uint32_t)0x08120000)     //扇区0起始地址, 128 Kbytes
//#define ADDR_FLASH_SECTOR_18    ((uint32_t)0x08140000)     //扇区0起始地址, 128 Kbytes
//#define ADDR_FLASH_SECTOR_19    ((uint32_t)0x08160000)     //扇区0起始地址, 128 Kbytes
//#define ADDR_FLASH_SECTOR_20    ((uint32_t)0x08180000)     //扇区0起始地址, 128 Kbytes
//#define ADDR_FLASH_SECTOR_21    ((uint32_t)0x081A0000)     //扇区0起始地址, 128 Kbytes
//#define ADDR_FLASH_SECTOR_22    ((uint32_t)0x081C0000)     //扇区0起始地址, 128 Kbytes
//#define ADDR_FLASH_SECTOR_23    ((uint32_t)0x081E0000)     //扇区0起始地址, 128 Kbytes
#endif


//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
uint8_t Flash_WordWrite(uint32_t StartAddr,uint32_t *pBuff,uint32_t Lenth);
uint8_t Flash_HalfWordWrite(uint32_t StartAddr,uint16_t *pBuff,uint32_t Lenth);
uint8_t Flash_ByteWrite(uint32_t StartAddr,uint8_t *pBuff,uint32_t Lenth);

uint32_t Flash_WordRead(uint32_t faddr);
uint16_t Flash_HalfWordRead(uint32_t faddr);
uint8_t Flash_ByteRead(uint32_t faddr);

uint8_t Flash_Erase(uint32_t addr,uint8_t sectorNUM);
//-------------------- inline functions -------------------------------------

#endif /* _ST_FLASH_H_ */
//-----------------------End of file------------------------------------------
/** @}*/

