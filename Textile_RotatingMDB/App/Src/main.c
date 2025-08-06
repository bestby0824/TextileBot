/**
* 版权所有(C)
*
* ********
*
* @file main.c
* @brief
* @details
* @author MuNiu
* @version 1.0.0
* @date xxxx-xx-xx
* @par Description:
* @par Change History:
*
* <日期> | <版本> | <作者> | <描述>
*
* xxxx/xx/xx | 1.0.0 | MuNiu | 创建文件
*
*/
//------------------------------------------------------------------------------

//-------------------- pragmas ----------------------------------------------

//-------------------- include files ----------------------------------------
#include "main.h"
#include "stm32f4xx_hal.h"
#include "TimerBase.h"
#include "Int_Ctrl.h"
#include "TimerPWM.h"
#include "ADC.h"
#include "Uart.h"
#include "Can_Host.h"
#include "HostProcess.h"
#include "DataBaseProcess.h"
#include "HostState_Ctrl.h"
#include "ReangleRS485.h"
#include "Foc.h"
#include "CoderTama.h"
#include "Dido.h"
#include "Monitor.h"

//-------------------- local definitions ------------------------------------
#define ADDRESS_NEWAPP          0x08041000
#define ADDRESS_FACTORYAPP      0x080A1000

//-------------------- private data -----------------------------------------

//-------------------- private functions declare ----------------------------
static void SystemClock_Config ( void );
static void Error_Handler ( void );
static void GpioClock_Enable ( void );
static void LED_Blink ( void );
//-------------------- public data ------------------------------------------
uint8_t u8Standby_Flag = 0;

//-------------------- public functions -------------------------------------
/**
  * @brief  Main program
  * @param  None
  * @retval None
  * @notes  None
  */
int main ( void )
{
    static uint32_t u32Tick_Main = 0;
    static uint32_t u32CurrentPC, u32VTOR_LongAddr;//中断向量表地址重定向,NewAPP:0x08041000,FactoryAPP:0x080A1000

    u32CurrentPC = __current_pc();
    u32VTOR_LongAddr = ( u32CurrentPC & 0xFFFF0000 ) + 0x1000;
    SCB->VTOR = u32VTOR_LongAddr;
    __enable_irq();

    HAL_Init();
    SystemClock_Config ( );
    DataBaseInit ( );  //参数初始化
    
    if ( g_u32FaultNum.bit.uDBVersion == 0 ) {
        GpioClock_Enable ( );
        TimerBase1_Init ( TIM_BASE1_FREQ );
        TimerBase2_Init ( TIM_BASE2_FREQ );
        TIMER_PWM_Source_Init ( TIM_BASE_FREQ );
        TimerPWM_Init ( TIM_PWM_FREQ );       //PWM定时器初始化
        ADC_Init();
        Dido_Init ( );
        Can_Host_Config ( 1000 );  //CAN初始化
        ReangleRS485_Init ( 2500000 );
        CoderTama_Init ( 115200 );
        Uart_Init ( 256000 );    //调试串口初始化

        MotorInfo_Init ( );
        FOC_PID_Init ( );
        MonitorInit ( );  //上电自检
    }

    if ( u32VTOR_LongAddr == ADDRESS_NEWAPP )
    {
        Set_BootSuccessFlg ( );
        StoreVersion_NewApp ( );
    } else {
        StoreVersion_Factory ( );
    }

    while ( 1 )
    {
        if ( u8Tick1kHz_Flag )
        {
            u8Tick1kHz_Flag = 0;
            u32Tick_Main = TimerBase2_GetTicks();
            if ( u32Tick_Main > Standby_Delay )
            {
                u8Standby_Flag = 1;
            }
            switch ( u32Tick_Main % 10 ) //100Hz
            {
            case 0:
            {
                LED_Blink();                    //LED闪烁
            }
            break;

            case 1:
            {
                if ( RegReal_ResetCmd.u16Delay > 1 )
                {
                    RegReal_ResetCmd.u16Delay--;

                } else if ( RegReal_ResetCmd.u16Delay == 1 )
                {
                    SoftReset();
                }
            }
            break;

            case 2:
            {

            }
            break;

            case 3:
            {

            }
            break;

            case 4:
            {

            }
            break;

            case 5:
            {

            }
            break;

            case 6:
            {

            }
            break;

            case 7:
            {

            }
            break;

            case 8:
            {

            }
            break;

            case 9:
            {

            }
            break;

            default:
            {

            }
            break;
            }
        }
    }
}
/**
 * @brief  LED闪烁
 * @param  None
 * @retval None
 * @notes  None
 */
static void LED_Blink ( void )
{
    static LEDFaultHandle LEDFaultHandle1 = LED_Ready;
    static uint32_t u32LedTickLast = 0, u32FaultNum = 0;
    uint32_t u32LedTickNow;
    static uint16_t HoldTime_500ms = 500;

    u32LedTickNow = TimerBase2_GetTicks ( );
    if ( g_u32FaultNum.all )
    {
        switch ( LEDFaultHandle1 )
        {
        case LED_Ready:
            u32FaultNum = FaultNum_Get ( g_u32FaultNum );
            u32LedTickLast = u32LedTickNow;
            DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_LED );
            LEDFaultHandle1 = LED_LonOffSta;
            break;
        case LED_LonOffSta:
            if ( u32LedTickNow - u32LedTickLast > _IQ2 ( HoldTime_500ms ) ) //常灭2S
            {
                DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED );  //第一次点亮
                u32LedTickLast = u32LedTickNow;
                LEDFaultHandle1 = LED_On_Sta;
            }
            break;
        case LED_On_Sta:
            if ( u32LedTickNow - u32LedTickLast > _IQdiv2 ( HoldTime_500ms ) ) //点亮时间为0.25S
            {
                DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_LED );    //熄灭
                u32LedTickLast = u32LedTickNow;
                LEDFaultHandle1 = LED_Cnt1Sta;
                if ( u32FaultNum > 0 ) u32FaultNum--;
            }
            break;
        case LED_Cnt1Sta:
            if ( u32LedTickNow - u32LedTickLast > _IQdiv4 ( HoldTime_500ms ) ) //熄灭时间为0.25S
            {

                if ( u32FaultNum > 0 )
                {
                    DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED ); //再一次点亮
                    u32LedTickLast = u32LedTickNow;
                    LEDFaultHandle1 = LED_On_Sta;
                } else {
                    u32FaultNum = FaultNum_Get ( g_u32FaultNum );
                    u32LedTickLast = u32LedTickNow;
                    DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_LED );
                    LEDFaultHandle1 = LED_LonOffSta;
                }

            }
            break;
        default:
            u32LedTickLast = u32LedTickNow;
            LEDFaultHandle1 = LED_Ready;
            break;
        }
    }
    else
    {
        if ( Foc_EN_Flag == 0 )  //初始化未完成/失败
        {
            if ( ( u32LedTickNow - u32LedTickLast ) > _IQdiv4 ( HoldTime_500ms ) )
            {
                HAL_GPIO_TogglePin ( GPIOC, GPIO_PIN_7 );
                u32LedTickLast = u32LedTickNow;
            }
        }
        else
        {
            if ( ( u32LedTickNow - u32LedTickLast ) > HoldTime_500ms )
            {
                HAL_GPIO_TogglePin ( GPIOC, GPIO_PIN_7 );
                u32LedTickLast = u32LedTickNow;
            }
        }
    }
}
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  * @notes  None
  */
static void SystemClock_Config ( void )
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* The voltage scaling allows optimizing the power consumption when the device is
       clocked below the maximum system frequency, to update the voltage scaling value
       regarding system frequency refer to product datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG ( PWR_REGULATOR_VOLTAGE_SCALE1 );

    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;   //主时钟：8M/PLLM*PLLN/PLLP=168M
    RCC_OscInitStruct.PLL.PLLQ = 7;               //48M时钟：8M/PLLM*PLLN/PLLQ=168M
    if ( HAL_RCC_OscConfig ( &RCC_OscInitStruct ) != HAL_OK )
    {
        /* Initialization Error */
        Error_Handler();
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
       clocks dividers */
    RCC_ClkInitStruct.ClockType = ( RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 );
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if ( HAL_RCC_ClockConfig ( &RCC_ClkInitStruct, FLASH_LATENCY_5 ) != HAL_OK )
    {
        /* Initialization Error */
        Error_Handler();
    }

    /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
    if ( HAL_GetREVID() == 0x1001 )
    {
        /* Enable the Flash prefetch */
        __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    }
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  * @notes  None
  */
static void Error_Handler ( void )
{
    /* User may add here some code to deal with this error */
    while ( 1 )
    {
    }
}
static void GpioClock_Enable ( void )
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
