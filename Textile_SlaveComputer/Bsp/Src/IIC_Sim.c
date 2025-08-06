#include <string.h>
#include "IIC_Sim.h"
#include "AngleMath.h"



#if (IIC_COUNT>3)
Error! To many IIC
#endif

/*----------- IIC1 Device -----------*/


#define IIC1_SCL_GPIO_PORT          GPIOF
//#define IIC1_SCL_GPIO_CLK           RCC_AHB1Periph_GPIOF
#define IIC1_SCL_GPIO_PIN           GPIO_PIN_2
#define IIC1_SCL_PIN_POS            (2*2)

#define IIC1_SDA_GPIO_PORT          GPIOF
//#define IIC1_SDA_GPIO_CLK           RCC_AHB1Periph_GPIOF
#define IIC1_SDA_GPIO_PIN           GPIO_PIN_1
#define IIC1_SDA_PIN_POS            (1*2)





/*-----------IIC2 Device -----------*/

//#define IIC2_SCL_GPIO_PORT          GPIOA
//#define IIC2_SCL_GPIO_CLK           RCC_AHB1Periph_GPIOA
//#define IIC2_SCL_GPIO_PIN           GPIO_PIN_8
//#define IIC2_SCL_PIN_POS            (8*2)

//#define IIC2_SDA_GPIO_PORT          GPIOC
//#define IIC2_SDA_GPIO_CLK           RCC_AHB1Periph_GPIOC
//#define IIC2_SDA_GPIO_PIN           GPIO_PIN_9
//#define IIC2_SDA_PIN_POS            (9*2)

///*-----------IIC3 Device -----------*/

//#define IIC3_SCL_GPIO_PORT          GPIOH
//#define IIC3_SCL_GPIO_CLK           RCC_AHB1Periph_GPIOH
//#define IIC3_SCL_GPIO_PIN           GPIO_PIN_7
//#define IIC3_SCL_PIN_POS            (7*2)

//#define IIC3_SDA_GPIO_PORT          GPIOH
//#define IIC3_SDA_GPIO_CLK           RCC_AHB1Periph_GPIOH
//#define IIC3_SDA_GPIO_PIN           GPIO_PIN_8
//#define IIC3_SDA_PIN_POS            (8*2)


GPIO_TypeDef* IIC_SCL_GPIO_PORT[1] = {IIC1_SCL_GPIO_PORT};
const uint16_t IIC_SCL_GPIO_PIN[1] = {IIC1_SCL_GPIO_PIN};
const uint8_t IIC_SCL_PIN_POS[1] = {IIC1_SCL_PIN_POS};

GPIO_TypeDef* IIC_SDA_GPIO_PORT[1] = {IIC1_SDA_GPIO_PORT};
const uint16_t IIC_SDA_GPIO_PIN[1] = {IIC1_SDA_GPIO_PIN};
const uint8_t IIC_SDA_PIN_POS[1] = {IIC1_SDA_PIN_POS};


#define SDA_Clear(IIC) HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT[IIC],IIC_SDA_GPIO_PIN[IIC],GPIO_PIN_RESET)
#define SDA_Set(IIC) HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT[IIC],IIC_SDA_GPIO_PIN[IIC],GPIO_PIN_SET)

#define SCL_Clear(IIC) HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT[IIC],IIC_SCL_GPIO_PIN[IIC],GPIO_PIN_RESET)
#define SCL_Set(IIC) HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT[IIC],IIC_SCL_GPIO_PIN[IIC],GPIO_PIN_SET)

#define En_SDA_Input(IIC) IIC_SDA_GPIO_PORT[IIC]->MODER&=~(0x0003 << IIC_SDA_PIN_POS[IIC])
#define En_SDA_Output(IIC) IIC_SDA_GPIO_PORT[IIC]->MODER|=(0x0001 << IIC_SDA_PIN_POS[IIC])

#define SDA_Read(IIC) ((IIC_SDA_GPIO_PORT[IIC]->IDR&IIC_SDA_GPIO_PIN[IIC])!=0)?1:0


typedef struct {
    __IO uint8_t StartState;
    __IO uint8_t StopState;
    __IO int8_t ReadByteState;
    __IO uint8_t TransferByte;
    __IO uint8_t ReadStop;
    __IO uint8_t WriteByteState;
    __IO uint8_t WriteACK;
    __IO uint8_t Command;   //1-Read, 0=Write;
    __IO uint8_t Device;
    __IO uint32_t SubAddr;
    __IO uint8_t SubAddrLen;
    __IO uint8_t *TransferBuf;
    __IO uint16_t TransferCount;
    __IO uint8_t ReadState;
    __IO uint8_t WriteState;

    __IO uint8_t dat;
    __IO uint8_t bit;
    __IO    uint8_t IIC_BUSY;
    __IO uint8_t ERROR;
}   IIC_State;

static IIC_State iic_state[IIC_COUNT];

typedef struct {
    void ( *OnTx ) ( void );
    void ( *OnRx ) ( void );
    void ( *OnErr ) ( void );
} IIC_Callback;

__IO IIC_Callback iic_callback[IIC_COUNT];


#define IN            1
#define OUT           0
/*-------------------------------------------------------------------------*/
uint8_t IIC_EN_Flg[IIC_COUNT];







void __INLINE SetIicSdaDir ( IICIndex IIC, uint8_t x ) {
    if ( x ) En_SDA_Input ( IIC );
    else En_SDA_Output ( IIC );
}

static void IIC_GPIOInit ( IICIndex IIC )
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    GPIO_InitStruct.Mode    = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull    = GPIO_PULLUP;
    GPIO_InitStruct.Speed   = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pin     = IIC_SCL_GPIO_PIN[IIC];
    HAL_GPIO_Init ( ( GPIO_TypeDef* ) IIC_SCL_GPIO_PORT[IIC], &GPIO_InitStruct );

    GPIO_InitStruct.Mode    = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull    = GPIO_PULLUP;
    GPIO_InitStruct.Speed   = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pin     = IIC_SDA_GPIO_PIN[IIC];
    HAL_GPIO_Init ( ( GPIO_TypeDef* ) IIC_SDA_GPIO_PORT[IIC], &GPIO_InitStruct );

}




void IIC_Init ( IICIndex IIC )
{
    IIC_GPIOInit ( IIC );
    SDA_Set ( IIC );
    SCL_Set ( IIC );

}
#define p iic_state[IIC]
#define q iic_callback[IIC]

void IIC_SetCallback ( IICIndex IIC, void ( *OnTx ) ( void ), void ( *OnRx ) ( void ), void ( *OnErr ) ( void ) )
{
    q.OnErr = OnErr;
    q.OnTx = OnTx;
    q.OnRx = OnRx;
}


static uint8_t IIC_StartStateMachine ( IICIndex IIC )
{
    switch ( p.StartState ) {
    case 0:
        SDA_Set ( IIC );
        SCL_Set ( IIC );
        p.StartState++;
        break;
    case 1:
        SDA_Clear ( IIC );
        //SoftDelay(0);
        p.StartState++;
        break;
    case 2:
        SCL_Clear ( IIC );
        p.StartState = 0;
        break;
    }
    return p.StartState;
}

static uint8_t IIC_StopStateMachine ( IICIndex IIC )
{
    switch ( p.StopState ) {
    case 0:
        SCL_Set ( IIC );
        SDA_Clear ( IIC );
        //SoftDelay(1);
        p.StopState++;
        break;
    case 1:
        SDA_Set ( IIC );
        p.StopState = 0;
        break;
    }
    return p.StopState;
}

static uint8_t IIC_ReadByteStateMachine ( IICIndex IIC )
{
    switch ( p.ReadByteState ) {
    case 0:
        SetIicSdaDir ( IIC, IN );
        p.bit = 0;
        p.ReadByteState++;
        break;
    case 1:
        p.dat <<= 1;
        SCL_Set ( IIC );
        p.ReadByteState++;
        break;
    case 2:
        if ( SDA_Read ( IIC ) )
        {
            p.dat |= 1;
        }
        SCL_Clear ( IIC );
        p.bit++;
        if ( p.bit == 8 ) p.ReadByteState++;
        else {
            p.ReadByteState--;
            break;
        }
    case 3:
        p.TransferByte = p.dat;
        SetIicSdaDir ( IIC, OUT );
        if ( p.ReadStop ) SDA_Set ( IIC );
        else SDA_Clear ( IIC );  // ReadStop = 0; ask, ReadStop = 1,stop
        p.ReadByteState++;
        break;
    case 4:
        SCL_Set ( IIC );
        p.ReadByteState++;
        break;
    case 5:
        SCL_Clear ( IIC );
        p.ReadByteState++;
    case 6:
        p.ReadByteState = 0;
        break;
    }
    return p.ReadByteState;
}

static uint8_t IIC_WriteByteStateMachine ( IICIndex IIC )
{
    switch ( p.WriteByteState ) {
    case 0:
        p.dat = p.TransferByte;
        p.bit = 8;
        p.WriteByteState++;
    case 1:
        if ( p.dat & 0x80 )
        {
            SDA_Set ( IIC );
        }
        else
        {
            SDA_Clear ( IIC );
        }
        p.WriteByteState++;
        break;
    case 2:
        SCL_Set ( IIC );
        p.WriteByteState++;
        break;
    case 3:
        p.dat <<= 1;
        SCL_Clear ( IIC );
        p.bit--;
        if ( p.bit ) {
            p.WriteByteState = 1;
            break;
        }
        else p.WriteByteState++;
    case 4:
        SetIicSdaDir ( IIC, IN );
        p.WriteByteState++;
        break;
    case 5:
        SCL_Set ( IIC );
        p.WriteByteState++;
        break;
    case 6:
        p.WriteACK = SDA_Read ( IIC );
        SCL_Clear ( IIC );
        SetIicSdaDir ( IIC, OUT );
        p.WriteByteState++;
        break;
    case 7:
        p.WriteByteState = 0;
        break;
    }
    return p.WriteByteState;
}

static uint8_t IIC_ReadStateMachine ( IICIndex IIC )
{
    switch ( p.ReadState ) {
    case 0:
        p.ReadState++;
    case 1:
        if ( IIC_StartStateMachine ( IIC ) == 0 ) p.ReadState++;
        break;
    case 2:
        p.TransferByte = p.Device;
        p.ReadState++;
    case 3:
        if ( IIC_WriteByteStateMachine ( IIC ) == 0 ) {
            if ( p.WriteACK == 1 ) {
                p.ReadState = 14;   //Stop
            }
            else {
                if ( p.SubAddrLen )   p.ReadState++; //Send Access Address
                else p.ReadState += 3;  //No Address
            }
        }
        break;
    case 4: //Send Address
        switch ( p.SubAddrLen ) {
        case 4:
            p.TransferByte = ( p.SubAddr >> 24 ) & 0x000000FF;
            break;
        case 3:
            p.TransferByte = ( p.SubAddr >> 16 ) & 0x000000FF;
            break;
        case 2:
            p.TransferByte = ( p.SubAddr >> 8 ) & 0x000000FF;
            break;
        case 1:
            p.TransferByte = p.SubAddr & 0x000000FF;
            break;
        }
        p.SubAddrLen--;
        p.ReadState++;
    case 5:
        if ( IIC_WriteByteStateMachine ( IIC ) == 0 ) {
            if ( p.WriteACK == 1 ) {
                p.ReadState = 14;   //Stop
            }
            else {
                if ( p.SubAddrLen == 0 ) p.ReadState++;
                else p.ReadState--;
            }
        }
        break;
    case 6:
        if ( IIC_StartStateMachine ( IIC ) == 0 ) p.ReadState++;
        break;
    case 7: //Send Device Read
        p.TransferByte = p.Device | 0x01;
        p.ReadState++;
    case 8:
        if ( IIC_WriteByteStateMachine ( IIC ) == 0 ) {
            if ( p.WriteACK == 1 ) {
                p.ReadState = 14;
            }
            else {
                if ( p.TransferCount == 1 ) p.ReadState += 3;
                else p.ReadState++;
            }
        }
        break;
    case 9: //Read Bytes
        p.ReadStop = 0;
        p.ReadState++;
    case 10:
        if ( IIC_ReadByteStateMachine ( IIC ) == 0 ) {
            *p.TransferBuf = p.TransferByte;
            p.TransferBuf++;
            p.TransferCount--;
            if ( p.TransferCount == 1 ) p.ReadState++;
        }
        break;
    case 11:    //Read Last Byte
        p.ReadStop = 1;
        p.ReadState++;
    case 12:    //Read Last Byte
        if ( IIC_ReadByteStateMachine ( IIC ) == 0 ) {
            *p.TransferBuf = p.TransferByte;
            p.TransferCount = 0;
            p.ReadState++;
        }
        break;
    case 13:
        if ( IIC_StopStateMachine ( IIC ) == 0 ) {
            p.ReadState = 0;
            p.IIC_BUSY = 0;
            p.ERROR = 0;
            if ( q.OnRx ) q.OnRx();
        }
        break;
    case 14:
        if ( IIC_StopStateMachine ( IIC ) == 0 ) {
            p.ReadState = 0;
            p.IIC_BUSY = 0;
            p.ERROR = 1;
            if ( q.OnErr ) q.OnErr();
        }
        break;
    }
    return p.ReadState;
}

static uint8_t IIC_WriteStateMachine ( IICIndex IIC )
{
    switch ( p.WriteState ) {
    case 0:
        p.WriteState++;
    case 1:
        if ( IIC_StartStateMachine ( IIC ) == 0 ) p.WriteState++;
        break;
    case 2:
        p.TransferByte = p.Device;
        p.WriteState++;
    case 3:
        if ( IIC_WriteByteStateMachine ( IIC ) == 0 ) {
            if ( p.WriteACK == 1 ) {
                p.WriteState = 11;      //Stop
            }
            else {
                if ( p.SubAddrLen )   p.WriteState++; //Send Access Address
                else {
                    if ( p.TransferCount ) p.WriteState += 5; //Multi-Bytes;
                    else p.WriteState += 3; //Single Byte
                }
            }
        }
        break;
    case 4: //Send Address
        switch ( p.SubAddrLen ) {
        case 4:
            p.TransferByte = ( p.SubAddr >> 24 ) & 0x000000FF;
            break;
        case 3:
            p.TransferByte = ( p.SubAddr >> 16 ) & 0x000000FF;
            break;
        case 2:
            p.TransferByte = ( p.SubAddr >> 8 ) & 0x000000FF;
            break;
        case 1:
            p.TransferByte = p.SubAddr & 0x000000FF;
            break;
        }
        p.SubAddrLen--;
        p.WriteState++;
    case 5:
        if ( IIC_WriteByteStateMachine ( IIC ) == 0 ) {
            if ( p.WriteACK == 1 ) {
                p.WriteState = 11;      //Stop
            }
            else {
                if ( p.SubAddrLen == 0 ) {
                    if ( p.TransferCount ) p.WriteState += 3; //Multi-Bytes;
                    else p.WriteState++; //Single Byte
                }
                else p.WriteState--;
            }
        }
        break;
    case 6: //Send Only One Byte
        p.TransferByte = ( uint32_t ) p.TransferBuf;
        p.WriteState++;
    case 7:
        if ( IIC_WriteByteStateMachine ( IIC ) == 0 ) {
            if ( p.WriteACK == 1 ) {
                p.WriteState = 11;      //Stop
            }
            else {
                p.WriteState += 3;
            }
        }
        break;
    case 8: //Send Multi-Bytes Data
        p.TransferByte = *p.TransferBuf;
        p.TransferBuf++;
        p.TransferCount--;
        p.WriteState++;
    case 9:
        if ( IIC_WriteByteStateMachine ( IIC ) == 0 ) {
            if ( p.WriteACK == 1 ) {
                p.WriteState = 11;      //Stop
            }
            else {
                if ( p.TransferCount == 0 ) p.WriteState++;
                else p.WriteState--;
            }
        }
        break;
    case 10:
        if ( IIC_StopStateMachine ( IIC ) == 0 ) {
            p.WriteState = 0;
            p.IIC_BUSY = 0;
            p.ERROR = 0;
            if ( q.OnTx ) q.OnTx();
        }
        break;
    case 11:
        if ( IIC_StopStateMachine ( IIC ) == 0 ) {
            p.WriteState = 0;
            p.IIC_BUSY = 0;
            p.ERROR = 1;
            if ( q.OnErr ) q.OnErr();
        }
        break;
    }
    return p.WriteState;
}

static uint8_t IIC_StateMachine ( IICIndex IIC )
{
    if ( p.Command ) return IIC_ReadStateMachine ( IIC );
    return IIC_WriteStateMachine ( IIC );
}


uint8_t IIC_Read7 ( IICIndex IIC, uint8_t device, uint8_t Addr, uint8_t *Buf, uint8_t Count )
{
    if ( p.IIC_BUSY == 0 ) {
        memset ( &p, 0, sizeof ( IIC_State ) );
        p.Command = 1;  //1-Read, 0=Write;
        p.Device = device;
        p.SubAddr = Addr;
        p.SubAddrLen = 1;
        p.TransferBuf = Buf;
        p.TransferCount = Count;
        p.IIC_BUSY = 1;
        return 1;
    }
    else return 0;
}

uint8_t IIC_Read16 ( IICIndex IIC, uint8_t device, uint16_t Addr, uint8_t *Buf, uint8_t Count )
{
    if ( p.IIC_BUSY == 0 ) {
        memset ( &p, 0, sizeof ( IIC_State ) );
        p.Command = 1;  //1-Read, 0=Write;
        p.Device = device;
        p.SubAddr = Addr;
        p.SubAddrLen = 2;
        p.TransferBuf = Buf;
        p.TransferCount = Count;
        p.IIC_BUSY = 1;
        return 1;
    }
    else return 0;
}

uint8_t IIC_WriteByte7 ( IICIndex IIC, uint8_t device, uint8_t Addr, uint8_t *Buf, uint8_t Count )
{
    if ( p.IIC_BUSY == 0 ) {
        memset ( &p, 0, sizeof ( IIC_State ) );
        p.Command = 0;  //1-Read, 0=Write;
        p.Device = device;
        p.SubAddr = Addr;
        p.SubAddrLen = 1;
        p.TransferBuf = Buf;
        p.TransferCount = Count;
        p.IIC_BUSY = 1;
        return 1;
    }
    else return 0;
}

uint8_t IIC_Write16 ( IICIndex IIC, uint8_t device, uint16_t Addr, uint8_t *Buf, uint8_t Count )
{
    if ( p.IIC_BUSY == 0 ) {
        memset ( &p, 0, sizeof ( IIC_State ) );
        p.Command = 0;  //1-Read, 0=Write;
        p.Device = device;
        p.SubAddr = Addr;
        p.SubAddrLen = 2;
        p.TransferBuf = Buf;
        p.TransferCount = Count;
        p.IIC_BUSY = 1;
        return 1;
    }
    else return 0;
}

#if (IIC_COUNT>=1)
void IIC1_IRQHandler ( void )
{

    if ( IIC_StateMachine ( IIC1 ) == 0 ) {
        if ( iic_state[IIC1].IIC_BUSY == 0 ) {
            IIC_EN_Flg[IIC1] = 0;
        }
    }

}
#endif

#if (IIC_COUNT>=2)
void IIC2_IRQHandler ( void )
{

    if ( IIC_StateMachine ( 1 ) == 0 ) {
        if ( iic_state[1].IIC_BUSY == 0 ) {
            Stop
        }
    }

}
#endif

#if (IIC_COUNT>=3)
void IIC3_IRQHandler ( void )
{

    if ( IIC_StateMachine ( 2 ) == 0 ) {
        if ( iic_state[2].IIC_BUSY == 0 ) {
            Stop
        }
    }

}
#endif
