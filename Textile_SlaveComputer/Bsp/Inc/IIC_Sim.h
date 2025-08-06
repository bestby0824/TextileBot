#ifndef __IIC_SIM_H__
#define __IIC_SIM_H__
 
#include "Driver.h"


#define IIC_COUNT       1
#define MAXFREQ         500000

typedef enum {
    IIC1 = 0,
    IIC2,
    IIC3,
}IICIndex;

extern uint8_t IIC_EN_Flg[IIC_COUNT];
 
extern uint8_t IIC_Read7(IICIndex IIC, uint8_t device, uint8_t Addr, uint8_t *Buf, uint8_t Count);
 
extern uint8_t IIC_Read16(IICIndex IIC, uint8_t device, uint16_t Addr, uint8_t *Buf, uint8_t Count);
 
extern uint8_t IIC_WriteByte7 ( IICIndex IIC, uint8_t device, uint8_t Addr, uint8_t *Buf, uint8_t Count );
 
extern uint8_t IIC_Write16(IICIndex IIC, uint8_t device, uint16_t Addr, uint8_t *Buf, uint8_t Count);
 
extern void IIC_Init ( IICIndex IIC );
 
extern void IIC_SetCallback(IICIndex IIC, void(*OnTx)(void), void(*OnRx)(void) ,void(*OnErr)(void));
void __INLINE SetIicSdaDir ( IICIndex IIC, uint8_t x );
#if (IIC_COUNT>=1)
void IIC1_IRQHandler ( void );
#endif

#if (IIC_COUNT>=2)
void IIC2_IRQHandler ( void );
#endif

#if (IIC_COUNT>=3)
void IIC3_IRQHandler ( void );
#endif 
#endif