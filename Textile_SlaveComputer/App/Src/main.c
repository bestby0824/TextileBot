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
//-------------------- pragmas ----------------------------------------------
//-------------------- include files ----------------------------------------
#include "main.h"
#include "DidoProcess.h"
#include "Uart_Dbug.h"
#include "IMU_Com.h"
#include "Uart_Host.h"
#include "TimerEncoder.h"
#include "StateCtrl.h"
#include "Accelerator.h"
#include "Monitor.h"
#include "Uart_Dbug.h"
#include "AdcProcess.h"
#include "Oled_com.h"
#include "RGBLED_Process.h"
#include "CAN_VCU.h"
#include "CAN_Node.h"
#include "VCUProcess.h"
#include "VCU_Ctrl.h"
#include "SteeringWheel_Ctrl.h"
#include "Host_Com.h"
#include "NodeProcess.h"
#include "Xint.h"
#include "Uart_Sprintf.h"

//-------------------- local definitions ------------------------------------
//-------------------- private data -----------------------------------------
static uint32_t m_unCnt1mS = 0;
static uint32_t m_unCnt10mS = 0;
static uint32_t m_unCnt1mS_Reset = 0;//计算启动延时
//-------------------- private functions declare ----------------------------
static void GpioClock_Enable ( void );
static void SystemClock_Config ( void );
static void LED_Blink ( BlinkFreq Freq );
static void SoftReset ( void );
static void Error_Handler ( void );
//-------------------- public data ------------------------------------------
PosLim TestLiftPosLim1;
#define ADDRESS_NEWAPP          0x08041000
#define ADDRESS_FACTORYAPP      0x080A1000
uint32_t u32CurrentPC, u32VTOR_LongAddr;//中断向量表地址重定向,NewAPP:0x08040000,FactoryAPP:0x080A0000
uint16_t CANSendFlag  = 0;
GPIO_PinState   HostShutDownFlag, FaultRetFlag;
uint8_t   HostShutDowncmd;
uint8_t   u8FaultRetFlag = 0, u8StandbyFlag = 0;
uint16_t u16StandbyCnt = 0;
uint8_t SolenoidLast_u16StaFlg = 1;
//uint32_t mcuID[3], Lock_Code, IdAddr = 0x1FFF7A10;

//void cpuidGetId ( void )
//{
//    mcuID[0] = * ( __IO uint32_t* ) ( IdAddr );
//    mcuID[1] = * ( __IO uint32_t* ) ( IdAddr + 4 );
//    mcuID[2] = * ( __IO uint32_t* ) ( IdAddr + 8 );
//    Lock_Code = ( mcuID[0] >> 1 ) + ( mcuID[1] >> 2 ) + ( mcuID[2] >> 3 );
//}

//-------------------- public functions -------------------------------------
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main ( void )
{
    uint8_t initbyte[2] = {0x01, 0x00};
    static uint32_t Tick_Main_1K = 0;
    // cpuidGetId();
    u32CurrentPC = __current_pc();
    u32VTOR_LongAddr = ( u32CurrentPC & 0xFFFF0000 ) + 0x1000;
    SCB->VTOR = u32VTOR_LongAddr;
    __enable_irq();
    HAL_Init();
    /* Configure the system clock to 48 MHz */
    SystemClock_Config();
    GpioClock_Enable();
    DataBaseInit();
    if ( FaultCtrl.bit.EepromReadErr == 0 )
    {
        DidoProcessInit();

        Adc_Init();
        RS485_SteeringWheel_Init ( 230400 );
        Uart_Host_Init ( 115200 );
        Uart_PCDbug_Init ( 115200 );
        TimerENC1_Init ( ENCLines );
        TimerENC2_Init ( ENCLines );
        SpdCtrl_Init();      //车速控制初始化
        // Spi_RgbLed_Init ();
        Can_VCU_Config ( 500 ); /* CAN1初始化 */
        Can_Node_Config ( 1000 );  /* CAN2初始化 */
        Ctrl_OffLine_Flag = 0x11;   //复位标志初始化
        /* Add your application code here */
        TimerBase_Init ( TIM_BASE_FREQ );//10k
        TIMER_Fast_Init ( TIM_FAST_FREQ );//50k
        Solenoid_Init();
        //    Uart_Dbug_Init ( 9600 );
        Uart_Sprintf_Init ( 256000 );
        //    Hart_SolenoidCmdInit();
        if ( u32VTOR_LongAddr == ADDRESS_NEWAPP )
        {
            Set_BootSuccessFlg ( );
            StoreVersion_NewApp ( );
        } else {
            StoreVersion_Factory ( );
        }
        Can_VCU_send_msg ( 0, ( uint8_t * ) initbyte, 2 );
        Reg_AGVTurnCtrlCmd1.u16CtrlWord.all = 0X0F;
    }
    /* Infinite loop */
    while ( 1 ) {
//
        if ( g_u8Tick1kHz ) //timer设置，1KHz
        {
            g_u8Tick1kHz = 0;
            m_unCnt1mS++;
            if ( m_unCnt1mS_Reset < 0xFFFFFFFF )
            {
                m_unCnt1mS_Reset++;
                if ( Ctrl_OffLine_Flag == 0 )
                {
                    ReSet_Flag = ReSet_Flag & 0xF0;
                }
                else if ( Ctrl_OffLine_Flag == 1 )
                {
                    ReSet_Flag = ReSet_Flag | 0x01;
                }
            }
            if ( m_unCnt1mS_Reset == Standby_Delay ) {  //延时2s启动
                RunStateInit();
                Oled_Init( );
                PowerCtrl ( AllPower, ON );
                Ctrl_OffLine_Flag = 0;
            }
            if ( m_unCnt1mS_Reset == 5000 )            //开机上电5秒后给VCU发送CAN指令
            {
                //SolenoidInit();
                CANSendFlag = 1;
            }
            else if ( m_unCnt1mS_Reset > Standby_Delay )
            {
                if ( ( SolenoidReport.u16StaFlg == 1 ) && ( SolenoidLast_u16StaFlg == 0 ) )
                {
                    ReSet_Flag += 0x10;
                }
                SolenoidLast_u16StaFlg = SolenoidReport.u16StaFlg;
            }
            else
            {
                SolenoidReport.u16StaFlg = 1;
                SolenoidLast_u16StaFlg = SolenoidReport.u16StaFlg;
            }
            if ( m_unCnt1mS_Reset >= Standby_Delay ) //2s后开始运行
            {
                // if ( 0 == m_unCnt1mS % 2 )OledStateMachine(); //最小单位，3个字节2mS
                switch ( m_unCnt1mS % 10 ) //100Hz
                {
                case 0:
                {
                    m_unCnt10mS++;
                    //RunStateCtrl();//注意 GotoIdleState() 会产生最大延时占用125mS
                }
                break;
                case 1:
                {
                    LED_Blink ( Normal );
                }
                break;

                case 2:
                {
                    ParamRead_Write ();
                }
                break;

                case 3://100hz----车辆控制模式切换
                {
                    LastMode =  m_eCtrlMode;
//                    CtrlModeFeedback ( );   //检测车辆控制模式修改m_eCtrlMode
                    m_eCtrlMode = InputIOInfor.CtrlModeOpt;
                    if ( LastMode != m_eCtrlMode )
                    {
                        HostCom_ClearCommand();
//                        m_sCurLiftCmd.LiftMode = 0;  //模式切换时清除一降到底模式
                        s16TargetSpeed = 0;
                        BreakPos = 0;
                        u8StandbyFlag = 1;
                    }
                    if ( u8StandbyFlag )
                    {
                        if ( u16StandbyCnt < 200 )  //2s
                        {
                            u16StandbyCnt++;
                        }
                        else
                        {
                            u16StandbyCnt = 0;
                            u8StandbyFlag = 0;
                        }
                    }
                }
                break;

                case 4:
                {
//                    Monitordisplay ();//小屏显示
                }
                break;

                case 5:
                {
                    //ParamReceiveProcess();
                }
                break;

                case 6:                      //
                {

                }
                break;

                case 7:
                {
                    FualtClear ();
                    ResetCheck();
                }
                break;

                case 8:
                {
                    HostCom_AutoCharCommand();
                    HostCom_RadLiftCommand();
                    HostCom_Fault_DisableCommand();
                    EncoderCalaCommand();
                    SteerCalaCommand();

                }
                break;

                case 9:
                {

                    if ( m_PowerSwCmd.u16Delay > 1 )
                    {
                        m_PowerSwCmd.u16Delay--;
                    }
                    else
                    {
                        m_PowerSwCmd.u16Delay = 1;
                    }
                    if ( u8PowerSwFlag )
                    {
                        if ( m_PowerSwCmd.u16Delay == 1 )
                        {

                            if ( m_PowerSwCmd.PowerSwOpt.bit.PcPowerSw == 1 )
                            {
                                PowerCtrl ( PCPower, ON );
                            }
                            else
                            {
                                PowerCtrl ( PCPower, OFF );
                            }
                            if ( m_PowerSwCmd.PowerSwOpt.bit.Motor1PowerSw == 1 )
                            {
                                PowerCtrl ( Motor1Power, ON );
                                PowerCtrl ( Motor2Power, ON );
                            }
                            else
                            {
                                PowerCtrl ( Motor1Power, OFF );
                                PowerCtrl ( Motor2Power, OFF );
                            }
                            if ( m_PowerSwCmd.PowerSwOpt.bit.Motor2PowerSw == 1 )
                            {
                                PowerCtrl ( Motor1Power, ON );
                                PowerCtrl ( Motor2Power, ON );
                            }
                            else
                            {
                                PowerCtrl ( Motor1Power, OFF );
                                PowerCtrl ( Motor2Power, OFF );
                            }
                            if ( m_PowerSwCmd.PowerSwOpt.bit.SensorPowerSw == 1 )
                            {
                                PowerCtrl ( SensorPower, ON );
                            }
                            else
                            {
                                PowerCtrl ( SensorPower, OFF );
                            }
                            u8PowerSwFlag = 0;
                        }
                    }
                    if ( AutoCharFlag )             //自动充电
                    {
                        Reg_AGVChargeSWCmd.u8SwCmd = 0x01;
                        Reg_AGVBMSCtrlCmd.u8ChargerCtrl = 0xA2;
                    }
                    else
                    {
                        Reg_AGVChargeSWCmd.u8SwCmd = 0x00;
                        Reg_AGVBMSCtrlCmd.u8ChargerCtrl = 0xA1;
                    }
                    Reg_AGVChargeSWCmd.u8SwCmd = 0x01;
                    if ( Reg_AGVBMSStaOrigDataFb.Staflag & 2 )
                    {
                        Reg_AGVBMSCtrlCmd.u8ChargerCtrl = 0xA2;
                    }
                    else if ( Reg_AGVBMSStaOrigDataFb.Staflag & 4 )
                    {
                        Reg_AGVBMSCtrlCmd.u8ChargerCtrl = 0xA1;
                    }
//                    {
//                        RegReal_ChassisParam.u8ChassisEnabled.bit.ChargingEnabled = 1;//自动充电使能
//                    }
//                    else
//                    {
//                        RegReal_ChassisParam.u8ChassisEnabled.bit.ChargingEnabled = 0;//自动充电失能
//                    }
                }
                break;

                default:
                {

                }
                break;
                }
            } else {
            }
        }
    }
}


/**
  * @brief
  * @param  None
  * @retval None
  */
static void LED_Blink ( BlinkFreq Freq )
{
    static LEDFaultHandle LEDFaultHandle1 = LED_Ready;
    static uint32_t u32LedTickLast;
    static uint16_t LEDFaultNum_Shadow;
    uint32_t u32LedTickNow;

    u32LedTickNow = TimerBase_GetTicks_10KHz();
    if ( FaultCtrl.all )
    {
        switch ( LEDFaultHandle1 )
        {
        case LED_Ready:
            LEDFaultNum_Shadow = FaultNum_Get();
            u32LedTickLast = u32LedTickNow;
            DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_LED );
            LEDFaultHandle1 = LED_LonOffSta;
            break;
        case LED_LonOffSta:
            if ( u32LedTickNow - u32LedTickLast > TIM_BASE_FREQ * 2 ) //常灭2S
            {
                DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED );  //第一次点亮
                u32LedTickLast = u32LedTickNow;
                LEDFaultHandle1 = LED_On_Sta;
            }
            break;
        case LED_On_Sta:
            if ( u32LedTickNow - u32LedTickLast > _IQdiv4 ( TIM_BASE_FREQ ) ) //点亮时间为0.25S
            {
                DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_LED );    //熄灭
                u32LedTickLast = u32LedTickNow;
                LEDFaultHandle1 = LED_Cnt1Sta;
                if ( LEDFaultNum_Shadow > 0 ) LEDFaultNum_Shadow--;
            }
            break;
        case LED_Cnt1Sta:
            if ( u32LedTickNow - u32LedTickLast > _IQdiv4 ( TIM_BASE_FREQ ) ) //熄灭时间为0.25S
            {
                if ( LEDFaultNum_Shadow > 0 )
                {
                    DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_LED ); //再一次点亮
                    u32LedTickLast = u32LedTickNow;
                    LEDFaultHandle1 = LED_On_Sta;
                } else {
                    LEDFaultNum_Shadow = FaultNum_Get();
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
    else {
        if ( Freq == FAST )
        {
            if ( u32LedTickNow - u32LedTickLast > _IQdiv8 ( TIM_BASE_FREQ ) )
            {
                DO_ToggleByIndex ( DO_CH_LED );
                u32LedTickLast = u32LedTickNow;
            }
        } else {
            if ( u32LedTickNow - u32LedTickLast > _IQdiv2 ( TIM_BASE_FREQ ) )
            {
                DO_ToggleByIndex ( DO_CH_LED );
                u32LedTickLast = u32LedTickNow;
            }
        }
        LEDFaultHandle1 = LED_Ready;
    }
}
/**
  * @brief  充电模式选择函数
  * @param  None
  * @retval None
  */
void ModeChange ( uint8_t Mode )
{
    if ( Mode )
    {
        //DO_WriteByIndex ( GPIO_PIN_RESET, DO_CH_IDRIVE_SW );     //自动
        RegReal_ChassisParam.u8ChassisEnabled.bit.AutoModeEnabled = ENABLE;//自动模式使能
    }
    else
    {
        //DO_WriteByIndex ( GPIO_PIN_SET, DO_CH_IDRIVE_SW );     //手动
        RegReal_ChassisParam.u8ChassisEnabled.bit.AutoModeEnabled = DISABLE;//自动模式失能
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
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed ( uint8_t* file, uint32_t line )
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while ( 1 )
    {
    }
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
