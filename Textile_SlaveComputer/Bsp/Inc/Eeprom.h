
//------------------------------------------------------------------------------

#ifndef EEPROM_H_
#define EEPROM_H_
//-------------------- include files ----------------------------------------
#include "Driver.h"

//-------------------- public definitions -----------------------------------
#define EEPROM_SDA_PIN       GPIO_PIN_9
#define EEPROM_SCL_PIN       GPIO_PIN_8
#define EEPROM_WP_PIN        GPIO_PIN_7

#define EEPROM_SDA_PORT     GPIOB
#define EEPROM_SCL_PORT     GPIOB
#define EEPROM_WP_PORT      GPIOB


#define AT_ADDR     0xa0

#define AT_CHECK_ADDR   0X00
#define AT_CHECK_VALUE  0X52


//函数返回的错误状态码定义
#define ERR_NONE                    0
#define ERR_INPUT_PARAMETERS        -1
#define ERR_IS_BUSY                 -2
#define ERR_DATA_OVERFLOW           -3
#define ERR_GET_RESULT_FAILED       -4
#define ERR_PROCESS_FAILED          -5
#define ERR_UART_TX_BUF_FULL        -6
#define ERR_POINTER_NULL            -7
#define ERR_ADD_HEADER              -8
#define ERR_UART_NULL               -9
#define ERR_DATA_NOT_RDY            -10
#define ERR_RX_DATA_HEADER          -11
#define ERR_RX_DATA_ADDR            -12
#define ERR_RX_DATA_CRC             -13
#define ERR_RX_DATA_LEN             -14
#define ERR_RESULT_NOT_EQUAL        -15
#define ERR_PROCESS_FAILED_Alloc    -16
#define ERR_PROCESS_FAILED_BufTake  -17
#define ERR_CHECK_SUM               -18
#define ERR_CHECK_DATA_INVALID      -19
#define ERR_BUFFER_EMPTY            -20
#define ERR_CRC                     -100

#define SystemInfo_EpromStartAddr    (0)       //!< 系统参数存储在eeprom的起始地址
#define EEPROM_ADDRESS_MAX         (0x7FF)   //!< (MB85RC16 2KByte 2的11次方)
#define EEPROM_PAGE_CAPACITY       (0x0010)    //!< 每页16字节
#define EEPROM_CAPACITY_MAX        (0x0800)

#define MB85RC16_WRITE_ADDR         0xA0
#define MB85RC16_READ_ADDR          0xA1

//------------------------页分配（添加备份区）----------------------------
#define E2PROM_PAGE_DB_Start             0x0000      //实际使用参数
#define E2PROM_PAGE_DB_Copy_Start        0x0400      //参数备份（恢复出厂设置使用）
#define E2PROM_PAGE_LEN                  0x0300      //有效长度，剩余位置用作其他（如E2PROM检测等）

//------------------------特殊功能区--------------------------------------
#define ROM_ADDR_FUNBYTE_START       1792                 //(E2PROM_PAGE_DB_Copy_Start+E2PROM_PAGE_LEN)=0x0700

#define ROM_ADDR_BackReadyFlg        2039                 //1Byte
#define ROM_ADDR_BootFlg             2040                 //1Byte
#define EEPROM_CHECK_ADDR            EEPROM_ADDRESS_MAX-4 //4Bytes，2043-2047
//------------------------2047----------------------------------------------
#define ROM_Back_Success             0x77



//------------------------------------------------------------------------

//-------------------- public data ------------------------------------------
extern uint8_t BootFlg_Read;
//-------------------- public functions -------------------------------------
void Eeprom_Init(void);
int8_t EEPROM_Write(uint16_t addr,uint8_t* pdat,uint16_t nums);
int8_t EEPROM_Read(uint16_t addr,uint8_t* pdat,uint16_t nums);
void BootFlg_Success(void);
void DB_Backup(void);
uint8_t DB_Recovery(void);

//-------------------- inline functions -------------------------------------

#endif /* EEPROM_H_ */
//-----------------------End of file------------------------------------------
/** @}*/
