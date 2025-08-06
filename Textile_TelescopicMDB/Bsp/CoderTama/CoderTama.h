
//------------------------------------------------------------------------------

#ifndef TAMA_CODER_H_
#define TAMA_CODER_H_
//-------------------- include files ----------------------------------------
#include "stm32f4xx_hal.h"
#include "IQmath.h"
//-------------------- public definitions -----------------------------------
#define TAMA_MAX_COMM_LENTH            255
#define TAMA_BAUD                      115200

#define TAMA_Set                       UART4
#define TAMA_CLK_ENABLE()              __HAL_RCC_UART4_CLK_ENABLE()

#define TAMA_TX_PIN                    GPIO_PIN_10
#define TAMA_TX_GPIO_PORT              GPIOC
#define TAMA_RX_PIN                    GPIO_PIN_11
#define TAMA_RX_GPIO_PORT              GPIOC
#define TAMA_EN_PIN                    GPIO_PIN_2
#define TAMA_EN_GPIO_PORT              GPIOD
#define TAMA_GPIO_AF                   GPIO_AF8_UART4

#define TAMA_IRQn                      UART4_IRQn
#define TAMA_IRQHandler                UART4_IRQHandler

#define TAMA_RX_DMA                    DMA1
#define TAMA_RX_DMA_ClK_ENABLE()       __HAL_RCC_DMA1_CLK_ENABLE()
#define TAMA_RX_DMA_STREAM             DMA1_Stream2
#define TAMA_RX_DMA_CHANNEL            DMA_CHANNEL_4
#define TAMA_RX_DMA_IRQ                DMA1_Stream2_IRQn
#define TAMA_RX_DMA_IRQ_Handler        DMA1_Stream2_IRQHandler
#define TAMA_RX_DMA_IRQ_CLEAR()        DMA1->LIFCR |= 0x3F<<16

#define TAMA_TX_DMA                    DMA1
#define TAMA_TX_DMA_ClK_ENABLE()       __HAL_RCC_DMA1_CLK_ENABLE()
#define TAMA_TX_DMA_STREAM             DMA1_Stream4
#define TAMA_TX_DMA_CHANNEL            DMA_CHANNEL_4
#define TAMA_TX_DMA_IRQ                DMA1_Stream4_IRQn
#define TAMA_TX_DMA_IRQ_Handler        DMA1_Stream4_IRQHandler
#define TAMA_TX_DMA_IRQ_CLEAR()        DMA1->HIFCR |= 0x3F<<0


#define ENCODER_PPR_TAMAGAWA           65535

typedef struct {

    _iq _iqLeft;
    _iq _iqRight;
    _iq _iqLSin;
    _iq _iqRSin;
    _iq _iqR_LSin;

    _iq _iqM_Tan;
    _iq _iqMean_RL;

} WheelAngle;

typedef union {
    uint8_t bytes[2];
    uint16_t all;
} U16Type;

typedef union {
    uint8_t bytes[4];
    uint32_t all;
} U32Type;

typedef struct {
    uint8_t Id;//地址
    uint8_t Featurecodes;//功能码
    uint16_t RegAddr;//寄存器起始地址
    uint8_t FrameSize;//寄存器数量 RegData=1:7,RegData=2,9
    uint16_t RegData;//寄存器值
    U16Type RxBuf16;//
    U32Type RxBuf32;//
    uint8_t RxFlag;
} ModbusTypeDef;


#define WheelAngleDefault   {\
  0, /*_iqLeft;    */    \
  0, /*_iqRight;    */   \
  0, /*_iqLSin;  */   \
  0, /*_iqRSin;   */  \
  0, /*_iqR_LSin;   */  \
  0, /*_iqM_Tan;   */    \
  0 /*_iqMean_RL;   */    \
}

//-------------------- public data ------------------------------------------
extern WheelAngle WheelAngleHandle1;

//-------------------- public functions -------------------------------------
void CoderTama_Init ( uint32_t baud );
void RelayCtrl_Send ( ModbusTypeDef *pModbusVars );
void SteerEcoderCalProcess ( void );
_iq CoderTama_GetCode ( void );
uint16_t CoderTama_GetCheckFlags ( void );
//-------------------- inline functions -------------------------------------

#endif /* TAMA_CODER_H_ */
//-----------------------End of file------------------------------------------
/** @}*/
