
//-------------------- include files ----------------------------------------
#include "Iwdg.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------
IWDG_HandleTypeDef IwdgHandle1;
uint8_t IWDG_EN = 0;
//-------------------- private functions declare ----------------------------

//-------------------- public data ------------------------------------------

//-------------------- public functions -------------------------------------
void IwdgInit ( uint16_t IwdgMs )
{
    if ( IwdgMs >= 0x0FFF ) IwdgMs = 0x0FFF;
    IwdgHandle1.Init.Prescaler = IWDG_PRESCALER_32;//单位约1mS
    IwdgHandle1.Init.Reload = IwdgMs;//超时
    IwdgHandle1.Instance = IWDG;
    HAL_IWDG_Init ( &IwdgHandle1 );
    HAL_IWDG_Refresh ( &IwdgHandle1 );
    IWDG_EN = 1;
}

void IwdgReload ( void )
{
    if ( IWDG_EN )
    {
        HAL_IWDG_Refresh ( &IwdgHandle1 );
    }
}

//-------------------- private functions ------------------------------------

//-----------------------End of file------------------------------------------
/** @} */ /* End of group */
