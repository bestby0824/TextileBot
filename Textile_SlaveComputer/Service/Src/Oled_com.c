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
* 2022/11/16 | 1.0.0 | HWW | 创建文件
*
*/
//------------------------------------------------------------------------------

//-------------------- pragmas ----------------------------------------------

//-------------------- include files ----------------------------------------
#include "Oled_com.h"
#include "string.h"
#include "Oledfont.h"
//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
uint8_t OLED_GRAM[128][8];  //128*64 = 128*(8*8)
uint8_t OledBuf_TX[255];
//-------------------- private functions declare ----------------------------
static void Oled_Send_CMD ( uint8_t cmd );
static void Oled_Send_Data ( void );
//-------------------- public data ------------------------------------------
uint8_t IIC1_result = 0;
OledSta OledSta1 = 0;
//-------------------- public functions -------------------------------------
/**
  * @brief  void Oled_Init(void)
  * @param  none
  * @retval None
  */


//-------------------- private functions ----------------------------
void OledcomTXCallback ( void )
{
    IIC1_result = 1;
}
void OledcomRXCallback ( void )
{
    IIC1_result = 2;
}
void OledcomERRCallback ( void )
{
    IIC1_result = 3;
}
void Oled_Init ( void )
{
    IIC_Init ( IIC1 );
    IIC_SetCallback ( IIC1, OledcomTXCallback, OledcomRXCallback, OledcomERRCallback );

//    Oled_Send_CMD ( 0xAE );
//    HAL_Delay ( 3 );       /*display off*/
//    Oled_Send_CMD ( 0x00 );
//    HAL_Delay ( 3 );       /*set lower column address*/
//    Oled_Send_CMD ( 0x10 );
//    HAL_Delay ( 3 );       /*set higher column address*/
//    Oled_Send_CMD ( 0x00 );
//    HAL_Delay ( 3 );       /*set display start line*/
//    Oled_Send_CMD ( 0xB0 );
//    HAL_Delay ( 3 );       /*set page address*/
//    Oled_Send_CMD ( 0x81 );
//    HAL_Delay ( 3 );       /*contract control*/
//    Oled_Send_CMD ( 0xff );
//    HAL_Delay ( 3 );       /*128*/
//    Oled_Send_CMD ( 0xA1 );
//    HAL_Delay ( 3 );       /*set segment remap*/
//    Oled_Send_CMD ( 0xA6 );
//    HAL_Delay ( 3 );       /*normal / reverse*/
//    Oled_Send_CMD ( 0xA8 );
//    HAL_Delay ( 3 );       /*multiplex ratio*/
//    Oled_Send_CMD ( 0x1F );
//    HAL_Delay ( 3 );       /*duty = 1/32*/
//    Oled_Send_CMD ( 0xC8 );
//    HAL_Delay ( 3 );       /*Com scan direction*/
//    Oled_Send_CMD ( 0xD3 );
//    HAL_Delay ( 3 );       /*set display offset*/
//    Oled_Send_CMD ( 0x00 );
//    HAL_Delay ( 3 );
//    Oled_Send_CMD ( 0xD5 );
//    HAL_Delay ( 3 );       /*set osc division*/
//    Oled_Send_CMD ( 0x80 );
//    HAL_Delay ( 3 );
//    Oled_Send_CMD ( 0xD9 );
//    HAL_Delay ( 3 );       /*set pre-charge period*/
//    Oled_Send_CMD ( 0x1f );
//    HAL_Delay ( 3 );
//    Oled_Send_CMD ( 0xDA );
//    HAL_Delay ( 3 );       /*set COM pins*/
//    Oled_Send_CMD ( 0x00 );
//    HAL_Delay ( 3 );
//    Oled_Send_CMD ( 0xdb );
//    HAL_Delay ( 3 );       /*set vcomh*/
//    Oled_Send_CMD ( 0x40 );
//    HAL_Delay ( 3 );
//    Oled_Send_CMD ( 0x8d );
//    HAL_Delay ( 3 );       /*set charge pump enable*/
//    Oled_Send_CMD ( 0x14 );
//    HAL_Delay ( 3 );
//    OLED_Clear();
//    Oled_Send_CMD ( 0xAF ); /*display ON*/

    Oled_Send_CMD ( 0xAE );
    HAL_Delay ( 3 ); //--turn off oled panel
    Oled_Send_CMD ( 0x00 );
    HAL_Delay ( 3 ); //---set low column address
    Oled_Send_CMD ( 0x10 );
    HAL_Delay ( 3 ); //---set high column address
    Oled_Send_CMD ( 0x40 );
    HAL_Delay ( 3 ); //--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    Oled_Send_CMD ( 0x81 );
    HAL_Delay ( 3 ); //--set contrast control register
    Oled_Send_CMD ( 0xCF );
    HAL_Delay ( 3 ); // Set SEG Output Current Brightness
    Oled_Send_CMD ( 0xA1 );
    HAL_Delay ( 3 ); //--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
    Oled_Send_CMD ( 0xC8 );
    HAL_Delay ( 3 ); //Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
    Oled_Send_CMD ( 0xA6 );
    HAL_Delay ( 3 ); //--set normal display
    Oled_Send_CMD ( 0xA8 );
    HAL_Delay ( 3 ); //--set multiplex ratio(1 to 64)
    Oled_Send_CMD ( 0x3f );
    HAL_Delay ( 3 ); //--1/64 duty
    Oled_Send_CMD ( 0xD3 );
    HAL_Delay ( 3 ); //-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
    Oled_Send_CMD ( 0x00 );
    HAL_Delay ( 3 ); //-not offset
    Oled_Send_CMD ( 0xd5 );
    HAL_Delay ( 3 ); //--set display clock divide ratio/oscillator frequency
    Oled_Send_CMD ( 0x80 );
    HAL_Delay ( 3 ); //--set divide ratio, Set Clock as 100 Frames/Sec
    Oled_Send_CMD ( 0xD9 );
    HAL_Delay ( 3 ); //--set pre-charge period
    Oled_Send_CMD ( 0xF1 );
    HAL_Delay ( 3 ); //Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    Oled_Send_CMD ( 0xDA );
    HAL_Delay ( 3 ); //--set com pins hardware configuration
    Oled_Send_CMD ( 0x12 );
    HAL_Delay ( 3 );
    Oled_Send_CMD ( 0xDB );
    HAL_Delay ( 3 ); //--set vcomh
    Oled_Send_CMD ( 0x40 );
    HAL_Delay ( 3 ); //Set VCOM Deselect Level
    Oled_Send_CMD ( 0x20 );
    HAL_Delay ( 3 ); //-Set Page Addressing Mode (0x00/0x01/0x02)
    Oled_Send_CMD ( 0x02 );
    HAL_Delay ( 3 ); //
    Oled_Send_CMD ( 0x8D );
    HAL_Delay ( 3 ); //--set Charge Pump enable/disable
    Oled_Send_CMD ( 0x14 );
    HAL_Delay ( 3 ); //--set(0x10) disable
    Oled_Send_CMD ( 0xA4 );
    HAL_Delay ( 3 ); // Disable Entire Display On (0xa4/0xa5)
    Oled_Send_CMD ( 0xA6 );
    HAL_Delay ( 3 ); // Disable Inverse Display On (0xa6/a7)
    OLED_Clear();
    HAL_Delay ( 3 );
    Oled_Send_CMD ( 0xAF );
    HAL_Delay ( 3 );

}

static void Oled_Send_CMD ( uint8_t cmd )
{
    OledBuf_TX[0] = cmd;
    IIC_WriteByte7 ( IIC1, 0x78, 0x00, OledBuf_TX, 1 );
    IIC_EN_Flg[IIC1] = 1;
}
static void Oled_Send_Data ( void )
{
    IIC_WriteByte7 ( IIC1, 0x78, 0x40, OledBuf_TX, 128 );
    IIC_EN_Flg[IIC1] = 1;
}
uint8_t OledRow;
/**
  * @brief  void OledStateMachine(void)
  * @param  none
  * @retval None
  */

void OledStateMachine ( void )
{

    static uint32_t u32Tick;
    if ( u32Tick > 0 ) u32Tick--;
    switch ( OledSta1 )
    {
    case Oled_Sta_PreCharg:
    {
        Oled_Send_CMD ( 0x8D ); //电荷泵使能
        OledSta1++;
    }
    break;

    case Oled_Sta_PWRON:
    {
        Oled_Send_CMD ( 0x14 ); //开启电荷泵
        OledSta1++;
    }
    break;

    case Oled_Sta_LightON:
    {
        Oled_Send_CMD ( 0xAF ); //点亮屏幕
        OledSta1++;
        OledRow = 0;
    }
    break;

    case Oled_Sta_LineStart:
    {
        if ( u32Tick == 0 )
        {
            Oled_Send_CMD ( 0xb0 + OledRow ); //设置行起始地址
            OledSta1++;
        }
    }
    break;
    case Oled_Sta_RowStart_L:
    {
        Oled_Send_CMD ( 0x00 );     //设置低列起始地址
        OledSta1++;
    }
    break;
    case Oled_Sta_RowStart_H:
    {
        Oled_Send_CMD ( 0x10 );     //设置高列起始地址
        OledSta1++;
    }
    break;
    case Oled_Sta_Data:
    {
        //发送第i组128*8
//        memcpy ( OledBuf_TX, &OLED_GRAM[0][i], 128 );
        for ( uint8_t j = 0; j < 128; j++ )
        {
            OledBuf_TX[j] = OLED_GRAM[j][OledRow];
        }

        Oled_Send_Data();
        OledSta1 = Oled_Sta_LineStart;
        u32Tick = 36;
        OledRow++;
        if ( OledRow >= 8 )
        {
            OledRow = 0;
        }
    }
    break;
    case Oled_Sta_Halt:
    {

    }
    break;
    }
}

//清屏函数
void OLED_Clear ( void )
{
    uint8_t i, n;
    for ( i = 0; i < 8; i++ )
    {
        for ( n = 0; n < 128; n++ )
        {
            OLED_GRAM[n][i] = 0; //清除所有数据
        }
    }
}
//刷新屏幕
void OLED_Refresh ( void )
{
    OledSta1 = Oled_Sta_LightON;
}

//画点
//x:0~127
//y:0~63
//t:1 填充 0,清空
void OLED_DrawPoint ( uint8_t x, uint8_t y, uint8_t t )
{
    uint8_t i, m, n;
    i = y / 8;
    m = y % 8;
    n = 1 << m;
    if ( t ) {
        OLED_GRAM[x][i] |= n;
    }
    else
    {
        OLED_GRAM[x][i] = ~OLED_GRAM[x][i];
        OLED_GRAM[x][i] |= n;
        OLED_GRAM[x][i] = ~OLED_GRAM[x][i];
    }
}

//画线
//x1,y1:起点坐标
//x2,y2:结束坐标
void OLED_DrawLine ( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t mode )
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
    delta_x = x2 - x1; //计算坐标增量
    delta_y = y2 - y1;
    uRow = x1; //画线起点坐标
    uCol = y1;
    if ( delta_x > 0 ) incx = 1; //设置单步方向
    else if ( delta_x == 0 ) incx = 0; //垂直线
    else {
        incx = -1;
        delta_x = -delta_x;
    }
    if ( delta_y > 0 ) incy = 1;
    else if ( delta_y == 0 ) incy = 0; //水平线
    else {
        incy = -1;
        delta_y = -delta_x;
    }
    if ( delta_x > delta_y ) distance = delta_x; //选取基本增量坐标轴
    else distance = delta_y;
    for ( t = 0; t < distance + 1; t++ )
    {
        OLED_DrawPoint ( uRow, uCol, mode ); //画点
        xerr += delta_x;
        yerr += delta_y;
        if ( xerr > distance )
        {
            xerr -= distance;
            uRow += incx;
        }
        if ( yerr > distance )
        {
            yerr -= distance;
            uCol += incy;
        }
    }
}
//x,y:圆心坐标
//r:圆的半径
void OLED_DrawCircle ( uint8_t x, uint8_t y, uint8_t r )
{
    int a, b, num;
    a = 0;
    b = r;
    while ( 2 * b * b >= r * r )
    {
        OLED_DrawPoint ( x + a, y - b, 1 );
        OLED_DrawPoint ( x - a, y - b, 1 );
        OLED_DrawPoint ( x - a, y + b, 1 );
        OLED_DrawPoint ( x + a, y + b, 1 );

        OLED_DrawPoint ( x + b, y + a, 1 );
        OLED_DrawPoint ( x + b, y - a, 1 );
        OLED_DrawPoint ( x - b, y - a, 1 );
        OLED_DrawPoint ( x - b, y + a, 1 );

        a++;
        num = ( a * a + b * b ) - r * r; //计算画的点离圆心的距离
        if ( num > 0 )
        {
            b--;
            a--;
        }
    }
}



//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//size1:选择字体 6x8/6x12/8x16/12x24
//mode:0,反色显示;1,正常显示
void OLED_ShowChar ( uint8_t x, uint8_t y, uint8_t chr, uint8_t size1, uint8_t mode )
{
    uint8_t i, m, temp, size2, chr1;
    uint8_t x0 = x, y0 = y;
    if ( size1 == 8 ) size2 = 6;
    else size2 = ( size1 / 8 + ( ( size1 % 8 ) ? 1 : 0 ) ) * ( size1 / 2 ); //得到字体一个字符对应点阵集所占的字节数
    chr1 = chr - ' '; //计算偏移后的值
    for ( i = 0; i < size2; i++ )
    {
        if ( size1 == 8 )
        {
            temp = asc2_0806[chr1][i];   //调用0806字体
        }
        else if ( size1 == 12 )
        {
            temp = asc2_1206[chr1][i];   //调用1206字体
        }
        else if ( size1 == 16 )
        {
            temp = asc2_1608[chr1][i];   //调用1608字体
        }
        else if ( size1 == 24 )
        {
            temp = asc2_2412[chr1][i];   //调用2412字体
        }
        else return;
        for ( m = 0; m < 8; m++ )
        {
            if ( temp & 0x01 ) OLED_DrawPoint ( x, y, mode );
            else OLED_DrawPoint ( x, y, !mode );
            temp >>= 1;
            y++;
        }
        x++;
        if ( ( size1 != 8 ) && ( ( x - x0 ) == size1 / 2 ) )
        {
            x = x0;
            y0 = y0 + 8;
        }
        y = y0;
    }
}


//显示字符串
//x,y:起点坐标
//size1:字体大小
//*chr:字符串起始地址
//mode:0,反色显示;1,正常显示
void OLED_ShowString ( uint8_t x, uint8_t y, char *chr, uint8_t size1, uint8_t mode )
{
    while ( ( *chr >= ' ' ) && ( *chr <= '~' ) ) //判断是不是非法字符!
    {
        OLED_ShowChar ( x, y, *chr, size1, mode );
        if ( size1 == 8 ) x += 6;
        else x += size1 / 2;
        chr++;
    }
}

//m^n
uint32_t OLED_Pow ( uint8_t m, uint8_t n )
{
    uint32_t result = 1;
    while ( n-- )
    {
        result *= m;
    }
    return result;
}

//显示数字
//x,y :起点坐标
//len :数字的位数
//size:字体大小
//mode:0,反色显示;1,正常显示
void OLED_ShowNum ( uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1, uint8_t mode )
{
    uint8_t t, temp, m = 0;
    if ( size1 == 8 ) m = 2;
    for ( t = 0; t < len; t++ )
    {
        temp = ( num / OLED_Pow ( 10, len - t - 1 ) ) % 10;
        if ( temp == 0 )
        {
            OLED_ShowChar ( x + ( size1 / 2 + m ) *t, y, '0', size1, mode );
        }
        else
        {
            OLED_ShowChar ( x + ( size1 / 2 + m ) *t, y, temp + '0', size1, mode );
        }
    }
}

//显示汉字
//x,y:起点坐标
//num:汉字对应的序号
//mode:0,反色显示;1,正常显示
void OLED_ShowChinese ( uint8_t x, uint8_t y, uint8_t num, uint8_t size1, uint8_t mode )
{
    uint8_t m, temp;
    uint8_t x0 = x, y0 = y;
    uint16_t i, size3 = ( size1 / 8 + ( ( size1 % 8 ) ? 1 : 0 ) ) * size1; //得到字体一个字符对应点阵集所占的字节数
    for ( i = 0; i < size3; i++ )
    {
        if ( size1 == 16 )
        {
            temp = Hzk1[num][i];   //调用16*16字体
        } else if ( size1 == 32 )
        {
            temp = Hzk32[num][i];   //调用32*32字体
        }
        else return;
        for ( m = 0; m < 8; m++ )
        {
            if ( temp & 0x01 ) OLED_DrawPoint ( x, y, mode );
            else OLED_DrawPoint ( x, y, !mode );
            temp >>= 1;
            y++;
        }
        x++;
        if ( ( x - x0 ) == size1 )
        {
            x = x0;
            y0 = y0 + 8;
        }
        y = y0;
    }
}

//num 显示汉字的个数
//space 每一遍显示的间隔
//mode:0,反色显示;1,正常显示
void OLED_ScrollDisplay ( uint8_t num, uint8_t space, uint8_t mode )
{
    uint8_t i, n, t = 0, m = 0, r;
    while ( 1 )
    {
        if ( m == 0 )
        {
            OLED_ShowChinese ( 128, 8, t, 16, mode ); //写入一个汉字保存在OLED_GRAM[][]数组中
            t++;
        }
        if ( t == num )
        {
            for ( r = 0; r < 16 * space; r++ ) //显示间隔
            {
                for ( i = 1; i < 144; i++ )
                {
                    for ( n = 0; n < 4; n++ )
                    {
                        OLED_GRAM[i - 1][n] = OLED_GRAM[i][n];
                    }
                }

            }
            t = 0;
        }
        m++;
        if ( m == 16 ) {
            m = 0;
        }
        for ( i = 1; i < 144; i++ ) //实现左移
        {
            for ( n = 0; n < 4; n++ )
            {
                OLED_GRAM[i - 1][n] = OLED_GRAM[i][n];
            }
        }

    }
}

//x,y：起点坐标
//sizex,sizey,图片长宽
//BMP[]：要写入的图片数组
//mode:0,反色显示;1,正常显示
void OLED_ShowPicture ( uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[], uint8_t mode )
{
    uint16_t j = 0;
    uint8_t i, n, temp, m;
    uint8_t x0 = x, y0 = y;
    sizey = sizey / 8 + ( ( sizey % 8 ) ? 1 : 0 );
    for ( n = 0; n < sizey; n++ )
    {
        for ( i = 0; i < sizex; i++ )
        {
            temp = BMP[j];
            j++;
            for ( m = 0; m < 8; m++ )
            {
                if ( temp & 0x01 ) OLED_DrawPoint ( x, y, mode );
                else OLED_DrawPoint ( x, y, !mode );
                temp >>= 1;
                y++;
            }
            x++;
            if ( ( x - x0 ) == sizex )
            {
                x = x0;
                y0 = y0 + 8;
            }
            y = y0;
        }
    }
}

//反显函数
void OLED_ColorTurn ( uint8_t i )
{
    if ( i == 0 )
    {
        Oled_Send_CMD ( 0xA6 ); //正常显示
    }
    if ( i == 1 )
    {
        Oled_Send_CMD ( 0xA7 ); //反色显示
    }
}

//屏幕旋转180度
void OLED_DisplayTurn ( uint8_t i )
{
    if ( i == 0 )
    {
        Oled_Send_CMD ( 0xC8 ); //正常显示
        Oled_Send_CMD ( 0xA1 );
    }
    if ( i == 1 )
    {
        Oled_Send_CMD ( 0xC0 ); //反转显示
        Oled_Send_CMD ( 0xA0 );
    }
}

/*****************************END OF FILE****/
