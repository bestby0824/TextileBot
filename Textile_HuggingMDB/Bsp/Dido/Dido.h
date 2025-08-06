
//------------------------------------------------------------------------------

#ifndef DIDO_H_
#define DIDO_H_

//-------------------- include files ----------------------------------------
#include "stm32f4xx_hal.h"

//-------------------- public definitions -----------------------------------
#define DI_VER1_GPIO_PORT             GPIOA
#define DI_VER2_GPIO_PORT             GPIOA
#define DI_VER3_GPIO_PORT             GPIOA
#define DI_HALL1_GPIO_PORT            GPIOA
#define DI_HALL2_GPIO_PORT            GPIOB
#define DI_HALL3_GPIO_PORT            GPIOC

#define DI_VER1_GPIO_PIN              GPIO_PIN_7
#define DI_VER2_GPIO_PIN              GPIO_PIN_6
#define DI_VER3_GPIO_PIN              GPIO_PIN_5
#define DI_HALL1_GPIO_PIN             GPIO_PIN_15
#define DI_HALL2_GPIO_PIN             GPIO_PIN_3
#define DI_HALL3_GPIO_PIN             GPIO_PIN_6

#define DO_LED_GPIO_PORT              GPIOC

#define DO_LED_GPIO_PIN               GPIO_PIN_7

typedef enum
{
    DI_CH_VER1,
    DI_CH_VER2,
    DI_CH_VER3,
    DI_CH_HALL1,
    DI_CH_HALL2,
    DI_CH_HALL3,
    DI_CH_NUM

} E_DI_CH, *PE_DI_CH;

typedef enum
{
    DO_CH_LED,

    DO_CH_NUM
} E_DO_CH, *PE_DO_CH;


#define DI_GPIO_PORT_LIST \
{                         \
    DI_VER1_GPIO_PORT,\
    DI_VER2_GPIO_PORT,\
    DI_VER3_GPIO_PORT,\
    DI_HALL1_GPIO_PORT,\
    DI_HALL2_GPIO_PORT,\
    DI_HALL3_GPIO_PORT,\
}

#define DI_GPIO_PIN_LIST \
{                        \
    DI_VER1_GPIO_PIN,\
    DI_VER2_GPIO_PIN,\
    DI_VER3_GPIO_PIN,\
    DI_HALL1_GPIO_PIN,\
    DI_HALL2_GPIO_PIN,\
    DI_HALL3_GPIO_PIN,\
}

#define DO_GPIO_PORT_LIST \
{                         \
  DO_LED_GPIO_PORT,       \
}

#define DO_GPIO_PIN_LIST \
{                        \
    DO_LED_GPIO_PIN,       \
}

//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
void Dido_Init ( void );
uint8_t DI_ReadAll ( uint16_t* pu16Value );
uint8_t DI_ReadByIndex ( GPIO_PinState* eStatus, E_DI_CH eIndex );
uint8_t DO_WriteAll ( uint16_t u16Value );
uint8_t DO_WriteByIndex ( GPIO_PinState eStatus, E_DO_CH eIndex );
uint8_t DO_ToggleByIndex ( E_DO_CH eIndex );

//-------------------- inline functions -------------------------------------

#endif /* DIDO_H_ */
//-----------------------End of file------------------------------------------
/** @}*/

