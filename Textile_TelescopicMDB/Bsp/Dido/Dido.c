
//-------------------- include files ----------------------------------------
#include "Dido.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
const GPIO_TypeDef* g_psInputGPIOPort[DI_CH_NUM] = DI_GPIO_PORT_LIST;

const uint16_t g_u16InputGPIOPin[DI_CH_NUM] = DI_GPIO_PIN_LIST;

const GPIO_TypeDef* g_psOutputGPIOPort[DO_CH_NUM] = DO_GPIO_PORT_LIST;

const uint16_t g_u16OutputGPIOPin[DO_CH_NUM] = DO_GPIO_PIN_LIST;

//-------------------- private functions declare ----------------------------

//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------

/*! \fn		 void Dido_Init(void)
 * @brief  Dido init.
 * @param  None
 * @retval None
 */
void Dido_Init ( void )
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    uint8_t i = 0;

    GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;//GPIO_NOPULL;//TODO
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

    for ( i = 0; i < DI_CH_NUM; i++ )
    {
        GPIO_InitStruct.Pin = g_u16InputGPIOPin[i];
        HAL_GPIO_Init ( ( GPIO_TypeDef* ) g_psInputGPIOPort[i], &GPIO_InitStruct );
    }
    
    GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull      = GPIO_PULLDOWN;//GPIO_NOPULL;//TODO
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

    for ( i = 0; i < DI_CH_HALL1; i++ )
    {
        GPIO_InitStruct.Pin = g_u16InputGPIOPin[i];
        HAL_GPIO_Init ( ( GPIO_TypeDef* ) g_psInputGPIOPort[i], &GPIO_InitStruct );
    }

    GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;//TODO
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

    for ( i = 0; i < DO_CH_NUM; i++ )
    {
        GPIO_InitStruct.Pin       = g_u16OutputGPIOPin[i];
        HAL_GPIO_Init ( ( GPIO_TypeDef* ) g_psOutputGPIOPort[i], &GPIO_InitStruct );
        HAL_GPIO_WritePin ( ( GPIO_TypeDef* ) g_psOutputGPIOPort[i], g_u16OutputGPIOPin[i], GPIO_PIN_RESET );
    }
}

/*! \fn			uint8_t DI_ReadByIndex(GPIO_PinState* eStatus, E_DI_CH eIndex)
 *  \brief 		read DI data by index
 *  \param 		eStatus: pointer to read di value
 *  \param 		eIndex: di channel index
 *  \return 	0:success other:false
 */
uint8_t DI_ReadByIndex ( GPIO_PinState* eStatus, E_DI_CH eIndex )
{
    *eStatus = HAL_GPIO_ReadPin ( ( GPIO_TypeDef* ) g_psInputGPIOPort[eIndex], g_u16InputGPIOPin[eIndex] );

    return 0;
}

/*! \fn			  uint8_t DI_ReadAll(uint16_t* pu16Value)
 *  \brief 		read all DI data
 *  \param 		pointer to read ALL DI value
 *  \return 	0:success other:false
 */
uint8_t DI_ReadAll ( uint16_t* pu16Value )
{
    uint8_t i = 0;
    uint16_t u16Temp = 0;
    GPIO_PinState eStatus = GPIO_PIN_RESET;

    for ( i = 0; i < DI_CH_NUM; i++ )
    {
        eStatus = HAL_GPIO_ReadPin ( ( GPIO_TypeDef* ) g_psInputGPIOPort[i], g_u16InputGPIOPin[i] );

        if ( eStatus == GPIO_PIN_SET )
        {
            u16Temp |= ( ( ( uint16_t ) 1 ) << i );
        }
        else
        {
            //NULL
        }
    }

    *pu16Value = u16Temp;

    return 0;
}

/*! \fn			  uint8_t DO_WriteByIndex(GPIO_PinState eStatus, E_DO_CH eIndex)
 *  \brief 		write do data by index
 *  \param 		eStatus: write DO value
 *  \param 		eIndex: DO channel index
 *  \return 	0:success other:false
 */
uint8_t DO_WriteByIndex ( GPIO_PinState eStatus, E_DO_CH eIndex )
{
    HAL_GPIO_WritePin ( ( GPIO_TypeDef* ) g_psOutputGPIOPort[eIndex], g_u16OutputGPIOPin[eIndex], eStatus );

    return 0;
}

/*! \fn			  uint8_t DO_WriteAll(uint16_t u16Value)
 *  \brief 		read all DO data
 *  \param 		write ALL DO value
 *  \return 	0:success other:false
 */
uint8_t DO_WriteAll ( uint16_t u16Value )
{
    uint8_t i = 0;

    for ( i = 0; i < DO_CH_NUM; i++ )
    {
        if ( u16Value & ( ( ( uint16_t ) 1 ) << i ) )
        {
            HAL_GPIO_WritePin ( ( GPIO_TypeDef* ) g_psOutputGPIOPort[i], g_u16OutputGPIOPin[i], GPIO_PIN_SET );
        }
        else
        {
            HAL_GPIO_WritePin ( ( GPIO_TypeDef* ) g_psOutputGPIOPort[i], g_u16OutputGPIOPin[i], GPIO_PIN_RESET );
        }
    }

    return 0;
}

/*! \fn			  uint8_t DO_ToggleByIndex(E_DO_CH eIndex)
 *  \brief 		Toggle DO by index
 *  \param 		eIndex: DO channel index
 *  \return 	0:success other:false
 */
uint8_t DO_ToggleByIndex ( E_DO_CH eIndex )
{
    HAL_GPIO_TogglePin ( ( GPIO_TypeDef* ) g_psOutputGPIOPort[eIndex], g_u16OutputGPIOPin[eIndex] );

    return 0;
}


//-------------------- private functions ------------------------------------

//-----------------------End of file------------------------------------------
/** @} */ /* End of group */
