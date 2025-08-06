/**
*

*/
//------------------------------------------------------------------------------

//-------------------- include files ----------------------------------------
#include "StFlash.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------

//-------------------- private functions declare ----------------------------
static uint8_t STMFLASH_GetFlashSector ( uint32_t addr );

//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
/*! \fn                uint8_t Flash_WordWrite(uint32_t StartAddr,uint32_t *pBuff,uint32_t Lenth)
 *  \brief
 *  \param      StartAddr
 *  \param      data buf
 *  \param      data lenth
 *  \return   0:success  other;failed
 */
uint8_t Flash_WordWrite ( uint32_t StartAddr, uint32_t *pBuff, uint32_t Lenth )
{
    uint32_t u32Addr;
    uint32_t u32EndAddr;
    uint32_t SectorError;
    FLASH_EraseInitTypeDef FlashEraseInit;

    if ( StartAddr < STM32_FLASH_BASE || StartAddr % 4 )     return 1; //闪存非法地址

    u32Addr = StartAddr;
    u32EndAddr = StartAddr + Lenth * 4;

    if ( u32EndAddr > STM32_FLASH_BASE + FLASH_LENTH )   return 1; //闪存地址越界

    HAL_FLASH_Unlock();             //闪存解锁


    //闪存写操作
    u32Addr = StartAddr;
    while ( u32Addr < u32EndAddr )         //闪存数据写入
    {
        if ( HAL_FLASH_Program ( FLASH_TYPEPROGRAM_WORD, u32Addr, *pBuff ) != HAL_OK )
        {
            return 3;//闪存写入失败
        }
        else
        {
            u32Addr += 4;
            pBuff ++;
        }
    }

    HAL_FLASH_Lock();           //闪存加锁

    return 0;    //闪存成功
}


/*! \fn                uint8_t Flash_HalfWordWrite(uint32_t StartAddr,uint16_t *pBuff,uint32_t Lenth)
 *  \brief
 *  \param         StartAddr
 *  \param         data buf
 *  \param      data lenth
 *  \return   0:success  other;failed
 */
uint8_t Flash_HalfWordWrite ( uint32_t StartAddr, uint16_t *pBuff, uint32_t Lenth )
{
    uint32_t u32Addr;
    uint32_t u32EndAddr;
    uint32_t SectorError;
    FLASH_EraseInitTypeDef FlashEraseInit;

    if ( StartAddr < STM32_FLASH_BASE || StartAddr % 2 )     return 1; //闪存非法地址

    u32Addr = StartAddr;
    u32EndAddr = StartAddr + Lenth * 2;

    if ( u32EndAddr > STM32_FLASH_BASE + FLASH_LENTH )   return 1; //闪存地址越界

    HAL_FLASH_Unlock();             //闪存解锁


    //闪存写操作
    u32Addr = StartAddr;
    while ( u32Addr < u32EndAddr )         //闪存数据写入
    {
        if ( HAL_FLASH_Program ( FLASH_TYPEPROGRAM_HALFWORD, u32Addr, *pBuff ) != HAL_OK )
        {
            return 3;           //闪存写入失败
        }
        else
        {
            u32Addr += 2;
            pBuff ++;
        }
    }

    HAL_FLASH_Lock();           //闪存加锁

    return 0;                   //闪存成功
}


/*! \fn                uint8_t Flash_ByteWrite(uint32_t StartAddr,uint8_t *pBuff,uint32_t Lenth)
 *  \brief
 *  \param         StartAddr
 *  \param         data buf
 *  \param      data lenth
 *  \return   0:success  other;failed
 */
uint8_t Flash_ByteWrite ( uint32_t StartAddr, uint8_t *pBuff, uint32_t Lenth )
{
    uint32_t u32Addr;
    uint32_t u32EndAddr;
    uint32_t SectorError;
    FLASH_EraseInitTypeDef FlashEraseInit;

    u32Addr = StartAddr;
    u32EndAddr = StartAddr + Lenth;

    if ( u32EndAddr > STM32_FLASH_BASE + FLASH_LENTH )   return 1;

    HAL_FLASH_Unlock();

    u32Addr = StartAddr;
    while ( u32Addr < u32EndAddr )
    {
        if ( HAL_FLASH_Program ( FLASH_TYPEPROGRAM_BYTE, u32Addr, *pBuff ) != HAL_OK )
        {
            return 3;
        }
        else
        {
            u32Addr += 1;
            pBuff ++;
        }
    }

    HAL_FLASH_Lock();
    return 0;
}

/*! \fn                uint32_t Flash_WordRead(uint32_t faddr)
 *  \brief
 *  \param         Addr
 *  \return   data
 */
uint32_t Flash_WordRead ( uint32_t faddr )
{
    return * ( __IO uint32_t * ) faddr;
}

/*! \fn                uint16_t Flash_HalfWordRead(uint32_t faddr)
 *  \brief
 *  \param         Addr
 *  \return   data
 */
uint16_t Flash_HalfWordRead ( uint32_t faddr )
{
    return * ( __IO uint16_t * ) faddr;
}

/*! \fn                uint8_t Flash_ByteRead(uint32_t faddr)
 *  \brief
 *  \param         Addr
 *  \return   data
 */
uint8_t Flash_ByteRead ( uint32_t faddr )
{
    return * ( __IO uint8_t * ) faddr;
}

uint8_t Flash_Erase ( uint32_t addr, uint8_t sectorNUM )
{
    uint32_t SectorError;
    FLASH_EraseInitTypeDef FlashEraseInit;

    if ( addr < STM32_FLASH_BASE )     return 1; //闪存非法地址

    if ( addr > STM32_FLASH_BASE + FLASH_LENTH )   return 1; //闪存地址越界

    HAL_FLASH_Unlock();             //闪存解锁

    FlashEraseInit.TypeErase    = FLASH_TYPEERASE_SECTORS;              //闪存擦除类型:扇区擦除
    FlashEraseInit.Sector       = STMFLASH_GetFlashSector ( addr );     //闪存擦除起始扇区
    FlashEraseInit.NbSectors    = sectorNUM;                            //闪存擦除一个扇区
    FlashEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;                //闪存电压范围:VCC=2.7~3.6V
    if ( HAL_FLASHEx_Erase ( &FlashEraseInit, &SectorError ) != HAL_OK )
    {
        return 2;  //闪存擦除失败
    }

    if ( FLASH_WaitForLastOperation ( FLASH_WAITETIME ) != HAL_OK )          //闪存等待上次操作完成
    {
        return 2;//闪存擦除失败
    }

    HAL_FLASH_Lock();           //闪存加锁

    return 0;    //闪存成功
}
//-------------------- private functions ------------------------------------
static uint8_t STMFLASH_GetFlashSector ( uint32_t addr )
{
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) || defined(STM32F412Zx) ||\
    defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx)

    if ( addr < ADDR_FLASH_SECTOR_1 ) return FLASH_SECTOR_0;
    else if ( addr < ADDR_FLASH_SECTOR_2 ) return FLASH_SECTOR_1;
    else if ( addr < ADDR_FLASH_SECTOR_3 ) return FLASH_SECTOR_2;
    else if ( addr < ADDR_FLASH_SECTOR_4 ) return FLASH_SECTOR_3;
    else if ( addr < ADDR_FLASH_SECTOR_5 ) return FLASH_SECTOR_4;
    else if ( addr < ADDR_FLASH_SECTOR_6 ) return FLASH_SECTOR_5;
    else if ( addr < ADDR_FLASH_SECTOR_7 ) return FLASH_SECTOR_6;
    else if ( addr < ADDR_FLASH_SECTOR_8 ) return FLASH_SECTOR_7;
    else if ( addr < ADDR_FLASH_SECTOR_9 ) return FLASH_SECTOR_8;
    else if ( addr < ADDR_FLASH_SECTOR_10 ) return FLASH_SECTOR_9;
    else if ( addr < ADDR_FLASH_SECTOR_11 ) return FLASH_SECTOR_10;
    return FLASH_SECTOR_11;

#endif

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx)|| defined(STM32F439xx) ||\
    defined(STM32F469xx) || defined(STM32F479xx)

    if ( addr < ADDR_FLASH_SECTOR_1 ) return FLASH_SECTOR_0;
    else if ( addr < ADDR_FLASH_SECTOR_2 ) return FLASH_SECTOR_1;
    else if ( addr < ADDR_FLASH_SECTOR_3 ) return FLASH_SECTOR_2;
    else if ( addr < ADDR_FLASH_SECTOR_4 ) return FLASH_SECTOR_3;
    else if ( addr < ADDR_FLASH_SECTOR_5 ) return FLASH_SECTOR_4;
    else if ( addr < ADDR_FLASH_SECTOR_6 ) return FLASH_SECTOR_5;
    else if ( addr < ADDR_FLASH_SECTOR_7 ) return FLASH_SECTOR_6;
    else if ( addr < ADDR_FLASH_SECTOR_8 ) return FLASH_SECTOR_7;
    else if ( addr < ADDR_FLASH_SECTOR_9 ) return FLASH_SECTOR_8;
    else if ( addr < ADDR_FLASH_SECTOR_10 ) return FLASH_SECTOR_9;
    else if ( addr < ADDR_FLASH_SECTOR_11 ) return FLASH_SECTOR_10;
    else if ( addr < ADDR_FLASH_SECTOR_12 ) return FLASH_SECTOR_11;
    else if ( addr < ADDR_FLASH_SECTOR_13 ) return FLASH_SECTOR_12;
    else if ( addr < ADDR_FLASH_SECTOR_14 ) return FLASH_SECTOR_13;
    else if ( addr < ADDR_FLASH_SECTOR_15 ) return FLASH_SECTOR_14;
    else if ( addr < ADDR_FLASH_SECTOR_16 ) return FLASH_SECTOR_15;
    else if ( addr < ADDR_FLASH_SECTOR_17 ) return FLASH_SECTOR_16;
    else if ( addr < ADDR_FLASH_SECTOR_18 ) return FLASH_SECTOR_17;
    else if ( addr < ADDR_FLASH_SECTOR_19 ) return FLASH_SECTOR_18;
    else if ( addr < ADDR_FLASH_SECTOR_20 ) return FLASH_SECTOR_19;
    else if ( addr < ADDR_FLASH_SECTOR_21 ) return FLASH_SECTOR_20;
    else if ( addr < ADDR_FLASH_SECTOR_22 ) return FLASH_SECTOR_21;
    else if ( addr < ADDR_FLASH_SECTOR_23 ) return FLASH_SECTOR_22;
    return FLASH_SECTOR_23;
#endif

}

//-----------------------End of file------------------------------------------
/** @} */ /* End of group */



