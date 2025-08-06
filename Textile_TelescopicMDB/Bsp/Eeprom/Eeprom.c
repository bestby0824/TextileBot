
//------------------------------------------------------------------------------

//-------------------- include files ----------------------------------------
#include "Eeprom.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------

//-------------------- private functions declare ----------------------------
static void SetSDATA ( uint8_t state );
static void SetSCLK ( uint8_t state );
static uint8_t GetSDATA ( void );
static void Start ( void );
static void Stop ( void );
//static void SendACK(void);
static int8_t Write ( uint8_t data_out );
static int8_t Read ( uint8_t *data_in, uint8_t send_ack );
static int8_t I2C_Read ( uint16_t address, uint8_t *data_in, uint16_t nums );
static int8_t I2C_Write ( uint16_t address, uint8_t *data_out, uint16_t nums );
//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
void Eeprom_Init ( void )
{
    GPIO_InitTypeDef   GPIO_InitStructure;

    GPIO_InitStructure.Mode     = GPIO_MODE_OUTPUT_OD;;
    GPIO_InitStructure.Pull     = GPIO_NOPULL;
    GPIO_InitStructure.Speed    = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Pin      = EEPROM_SDA_PIN;
    HAL_GPIO_Init ( EEPROM_SDA_PORT, &GPIO_InitStructure );
    HAL_GPIO_WritePin ( EEPROM_SDA_PORT, EEPROM_SDA_PIN, GPIO_PIN_SET );

    GPIO_InitStructure.Mode     = GPIO_MODE_OUTPUT_OD;;
    GPIO_InitStructure.Pull     = GPIO_NOPULL;
    GPIO_InitStructure.Speed    = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Pin      = EEPROM_SCL_PIN;
    HAL_GPIO_Init ( EEPROM_SCL_PORT, &GPIO_InitStructure );
    HAL_GPIO_WritePin ( EEPROM_SCL_PORT, EEPROM_SCL_PIN, GPIO_PIN_SET );

    GPIO_InitStructure.Mode     = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull     = GPIO_PULLUP;
    GPIO_InitStructure.Speed    = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Pin      = EEPROM_WP_PIN;
    HAL_GPIO_Init ( EEPROM_WP_PORT, &GPIO_InitStructure );
    HAL_GPIO_WritePin ( EEPROM_WP_PORT, EEPROM_WP_PIN, GPIO_PIN_SET );
}


int8_t EEPROM_Write ( uint16_t addr, uint8_t* pdat, uint16_t nums )
{
    int8_t res = ERR_NONE;

    //写溢出
    if ( ( addr + nums ) > EEPROM_ADDRESS_MAX )
    {
        return ERR_INPUT_PARAMETERS;
    }

    HAL_GPIO_WritePin ( EEPROM_WP_PORT, EEPROM_WP_PIN, GPIO_PIN_RESET );

    if ( ( addr / EEPROM_PAGE_CAPACITY ) == ( ( addr + nums ) / EEPROM_PAGE_CAPACITY ) ) //!< 所有待写入的数据在相同的页内，直接读并返回结果
    {
        res = I2C_Write ( addr, pdat, nums );
    }
    else                    //!< 待写入的数据跨页了
    {
        int16_t NumBy1Time, i, Page_N;
        //写整页前数据
        NumBy1Time = EEPROM_PAGE_CAPACITY - ( addr % EEPROM_PAGE_CAPACITY ) ;
        res = I2C_Write ( addr, pdat, NumBy1Time );
        addr += NumBy1Time;
        pdat += NumBy1Time;
        nums -= NumBy1Time;

        //写整页数据
        Page_N = nums / EEPROM_PAGE_CAPACITY;
        for ( i = 0; i < Page_N; i++ )
        {
            res = I2C_Write ( addr, pdat, EEPROM_PAGE_CAPACITY );
            addr += EEPROM_PAGE_CAPACITY;
            pdat += EEPROM_PAGE_CAPACITY;
            nums -= EEPROM_PAGE_CAPACITY;
        }
        //写整页后数据
        NumBy1Time = nums % EEPROM_PAGE_CAPACITY;
        if ( NumBy1Time > 0 )
        {
            res = I2C_Write ( addr, pdat, NumBy1Time );
        }
    }

    HAL_GPIO_WritePin ( EEPROM_WP_PORT, EEPROM_WP_PIN, GPIO_PIN_SET );
    return res;
}


int8_t EEPROM_Read ( uint16_t addr, uint8_t* pdat, uint16_t nums )
{
    int8_t res = ERR_NONE;

    //!< 判断地址是否超限
    if ( ( addr > EEPROM_ADDRESS_MAX ) || ( nums > EEPROM_CAPACITY_MAX ) )
    {
        return ERR_INPUT_PARAMETERS;
    }

    HAL_GPIO_WritePin ( EEPROM_WP_PORT, EEPROM_WP_PIN, GPIO_PIN_RESET );

    if ( ( addr / EEPROM_PAGE_CAPACITY ) == ( ( addr + nums ) / EEPROM_PAGE_CAPACITY ) ) //!< 所读的数据在相同的页内，直接读并返回结果
    {
        res = I2C_Read ( addr, pdat, nums );
    }
    else                                                                            //!< 所读的数据跨页了
    {
        int16_t NumBy1Time, i, Page_N;
        //读整页前数据
        NumBy1Time = EEPROM_PAGE_CAPACITY - ( addr % EEPROM_PAGE_CAPACITY ) ;
        res = I2C_Read ( addr, pdat, NumBy1Time );
        addr += NumBy1Time;
        pdat += NumBy1Time;
        nums -= NumBy1Time;

        //读整页数据
        Page_N = nums / EEPROM_PAGE_CAPACITY;
        for ( i = 0; i < Page_N; i++ )
        {
            res = I2C_Read ( addr, pdat, EEPROM_PAGE_CAPACITY );
            addr += EEPROM_PAGE_CAPACITY;
            pdat += EEPROM_PAGE_CAPACITY;
            nums -= EEPROM_PAGE_CAPACITY;
        }
        //读整页后数据
        NumBy1Time = nums % EEPROM_PAGE_CAPACITY;
        if ( NumBy1Time > 0 )
        {
            res = I2C_Read ( addr, pdat, NumBy1Time );
        }
    }

    HAL_GPIO_WritePin ( EEPROM_WP_PORT, EEPROM_WP_PIN, GPIO_PIN_SET );

    return res;
}


//-------------------- private functions ------------------------------------

static void SetSDATA ( uint8_t state )
{
    uint32_t i;
    if ( state )
    {
        EEPROM_SDA_PORT->BSRR = EEPROM_SDA_PIN;                 //!<  Set SDATA.
    } else
    {
        EEPROM_SDA_PORT->BSRR = EEPROM_SDA_PIN << 16 ;          //!<  reset SDA
    }
    for ( i = 0; i < 0x90; i++ ) {
        __asm ( "nop;" ); //!<  Delay
    }
}


static void SetSCLK ( uint8_t state )
{
    uint32_t i;

    if ( state )
    {
        EEPROM_SCL_PORT->BSRR = EEPROM_SCL_PIN ;                //!<  Set SCLK high.

    } else
    {
        EEPROM_SCL_PORT->BSRR = EEPROM_SCL_PIN << 16 ;          //!<  Set SCLK low.
    }
    for ( i = 0; i < 0x90; i++ )
    {
        __asm ( "nop;" );
    }
}


static uint8_t GetSDATA ( void )
{
    return ( ( EEPROM_SDA_PORT->IDR & EEPROM_SDA_PIN ) ? 1 : 0 );
}


static void Start ( void )
{
    SetSCLK ( 1 );                      //!<  Set SCLK high
    SetSDATA ( 0 );                     //!<  Set SDATA output/low
    SetSCLK ( 0 );                      //!<  Set SCLK low
}


static void Stop ( void )
{
    SetSDATA ( 0 );                //!<  Set SDATA output/low
    SetSCLK ( 1 );                 //!<  Set SCLK high
    SetSDATA ( 1 );                //!<  Set SDATA as input/high
}


static int8_t Write ( uint8_t data_out )
{
    uint8_t index;

    for ( index = 0; index < 8; index++ )
    {
        //!<  Output the data bit to the device
        SetSDATA ( ( ( data_out & 0x80 ) ? 1 : 0 ) );

        data_out <<= 1;                         //!<  Shift the byte by one bit
        SetSCLK ( 1 );                          //!<  Set SCLK high
        SetSCLK ( 0 );                          //!<  Set SCLK low
    }

    SetSDATA ( 1 );                             //!<  Set SDATA input/high
    SetSCLK ( 1 );                              //!<  Set SCLK high

    if ( !GetSDATA() )
    {
        SetSCLK ( 0 );                          //!<  Set SCLK low
        return ERR_NONE;                        //!<  ACK from slave
    } else
    {
        SetSCLK ( 0 );                          //!<  Set SCLK low
        return ERR_PROCESS_FAILED;              //!<  NACK from slave
    }
}

static int8_t Read ( uint8_t *data_in, uint8_t send_ack )
{
    uint8_t index;

    *data_in = 0x00;

    SetSDATA ( 1 );                             //!<  Set SDATA input/high
    SetSCLK ( 0 );                              //!<  Set SCLK low

    //!<  Get 8 bits from the device
    for ( index = 0; index < 8; index++ )
    {
        *data_in <<= 1;                         //!<  Shift the data right 1 bit
        SetSCLK ( 1 );                          //!<  Set SCLK high
        //!<  Set SCLK low
        *data_in |= GetSDATA();                 //!<  Read the data bit

        SetSCLK ( 0 );                          //!<  Set SCLK low
    }

    if ( send_ack )
        SetSDATA ( 0 );        //!<  Set data pin to output/low to ACK the read
    else
        SetSDATA ( 1 );        //!<  Set data pin to input/high to NACK the read

    SetSCLK ( 1 );                              //!<  Set SCLK high
    SetSCLK ( 0 );                              //!<  Set SCLK low
    SetSDATA ( 0 );                             //!<  Set SDATA output/low
    SetSDATA ( 1 );                             //!<  Set SDATA input/high

    return ERR_NONE;
}

static int8_t I2C_Read ( uint16_t address, uint8_t *data_in, uint16_t nums )
{
    uint8_t addr0, addr1;
    uint16_t i;

    addr0 = MB85RC16_WRITE_ADDR | ( ( uint8_t ) ( ( address >> 7 ) & 0x0E ) );
    addr1 = MB85RC16_READ_ADDR | ( ( uint8_t ) ( ( address >> 7 ) & 0x0E ) );

    Start();                                        //!<  Send start signal
    if ( Write ( addr0 ) )                          //!<  Send device address
    {
        Stop();                                     //!<  Send I2C Stop Transfer
        return ERR_PROCESS_FAILED;                  //!<  return false
    }
    if ( Write ( ( uint8_t ) address ) )            //!<  Send address low 8bit to device
    {
        Stop();
        return ERR_PROCESS_FAILED;
    }

    Start();                                        //!<  Send I2C Start Transer
    if ( Write ( addr1 ) )                          //!<  Send  I2C read address
    {
        Stop();                                     //!<  Send I2C Stop Transfer
        return ERR_PROCESS_FAILED;
    }

    for ( i = 0; i < ( nums - 1 ); i++ )
    {
        if ( Read ( &data_in[i], 1 ) )              //!<  Read byte
        {
            Stop();                                 //!<  Send I2C Stop Transfer
            return ERR_PROCESS_FAILED;
        }
    }

    if ( Read ( &data_in[ ( nums - 1 )], 0 ) )      //!<  Read last byte,does't send
    {
        Stop();
        return ERR_PROCESS_FAILED;
    }

    Stop();                                         //!<  Send I2C Stop Transfer

    return ERR_NONE;
}


static int8_t I2C_Write ( uint16_t address, uint8_t *data_out, uint16_t nums )
{
    uint8_t addr0;
    uint16_t i, j;

    addr0 = MB85RC16_WRITE_ADDR | ( ( uint8_t ) ( ( address >> 7 ) & 0x0E ) );

    Start();                                        //!<  Send start signal
    if ( Write ( addr0 ) )
    {
        Stop();
        return ERR_PROCESS_FAILED;
    }
    if ( Write ( ( uint8_t ) address ) )            //!<  Send address to device
    {
        Stop();
        return ERR_PROCESS_FAILED;
    }

    for ( i = 0; i < nums; i++ )
    {
        if ( Write ( data_out[i] ) )                //!<  Send byte to device
        {
            Stop();
            return ERR_PROCESS_FAILED;
        }
    }

    Stop();

    //need to delay some time
    for ( j = 0; j < 5; j++ )
    {
        for ( i = 0; i < 0x4000; i++ )
        {
            __asm ( "nop;" );
        }    //!<  Delay
    }

    return ERR_NONE;
}

//拷贝参数到备份区
void DB_Backup ( void )
{
    uint8_t E2P_Buf[E2PROM_PAGE_LEN];

    EEPROM_Read ( E2PROM_PAGE_DB_Start, E2P_Buf, E2PROM_PAGE_LEN );

    EEPROM_Write ( E2PROM_PAGE_DB_Copy_Start, E2P_Buf, E2PROM_PAGE_LEN );

    E2P_Buf[0] = ROM_Back_Success;
    EEPROM_Write ( ROM_ADDR_BackReadyFlg, E2P_Buf, 1 ); //备份完成标记
}

//恢复参数从备份区
uint8_t DB_Recovery ( void )
{
    uint8_t E2P_Buf[E2PROM_PAGE_LEN];


    EEPROM_Read ( ROM_ADDR_BackReadyFlg, E2P_Buf, 1 ); //备份完成标记
    if ( E2P_Buf[0] != ROM_Back_Success )
    {
        return 1;
    }


    EEPROM_Read ( E2PROM_PAGE_DB_Copy_Start, E2P_Buf, E2PROM_PAGE_LEN );

    EEPROM_Write ( E2PROM_PAGE_DB_Start, E2P_Buf, E2PROM_PAGE_LEN );
    return 0;

}
//-----------------------End of file------------------------------------------
/** @} */ /* End of group */



