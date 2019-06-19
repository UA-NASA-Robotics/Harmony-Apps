  #include <xc.h>
#include <stdbool.h>
#include <stdio.h>
#include <sys/attribs.h>
#include "uart_handler.h"
#include "app.h"


void _mon_putc(char ch);

void wipeBuf(unsigned char * buf, unsigned int numelems)
{
    int i=0;
    for(i=0;i<numelems;i++)
        buf[i]=0;
    
}

void myprintf(char * stringed)
{
    int i=0;
    while(stringed[i]!='\n')
    {
        //Send_put_2(stringed[i]);  //DEBUG TX
        Send_put(stringed[i++]);      //LIDAR TX
        LED5^=1;
    }
    
    Send_put(0x0A);     //LIDAR TX
    Send_put(0x0D);     //LIDAR TX
    //Send_put_2(0x0A);   //DEBUG TX
    //Send_put_2(0x0D);   //DEBUG TX
}

//Called when Printf() is used
void _mon_putc(char ch){
//    while (U5STAbits.UTXBF == 1) { // uart transmit FIFO full
//    }
    
    
    Send_put_2(ch);     //DEBUG TX
}



void UART_buff_init(struct UART_ring_buff* _this)
{
    /*****
      The following clears:
        -> buf
        -> head
        -> tail
        -> count
      and sets head = tail
     ***/
    Transmit_stall = true;
    Transmit_stall_1 = true;
    Transmit_stall_2 = true;
    Transmit_stall_3 = true;
    
    //WARNING
    //memset(_this, 0, sizeof (*_this));
}

void UART_buff_put(struct UART_ring_buff* _this, const unsigned char c)
{
    if (_this->count < UART_BUFFER_SIZE)
    {
        _this->buf[_this->head] = c;
        _this->head = UART_buff_modulo_inc(_this->head, UART_BUFFER_SIZE);
        ++_this->count;
    } else
    {
        _this->buf[_this->head] = c;
        _this->head = UART_buff_modulo_inc(_this->head, UART_BUFFER_SIZE);
        _this->tail = UART_buff_modulo_inc(_this->tail, UART_BUFFER_SIZE);

    }
}

unsigned char UART_buff_get(struct UART_ring_buff* _this)
{
    unsigned char c;
    if (_this->count > 0)
    {
        c = _this->buf[_this->tail];
        _this->tail = UART_buff_modulo_inc(_this->tail, UART_BUFFER_SIZE);
        --_this->count;
    } else
    {
        c = 0;
    }
    return (c);
}

void UART_buff_flush(struct UART_ring_buff* _this, const int clearBuffer)
{
    _this->count = 0;
    _this->head = 0;
    _this->tail = 0;
    if (clearBuffer)
    {
        wipeBuf(_this->buf,sizeof (_this->buf));
       // memset(_this->buf, 0, sizeof (_this->buf));
    }
}

int UART_buff_size(struct UART_ring_buff* _this)
{
    return (_this->count);
}

unsigned int UART_buff_modulo_inc(const unsigned int value, const unsigned int modulus)
{
    unsigned int my_value = value + 1;
    if (my_value >= modulus)
    {
        my_value = 0;
    }
    return (my_value);
}

unsigned char UART_buff_peek(struct UART_ring_buff* _this)
{
    return _this->buf[_this->tail];
}

unsigned char Receive_peek(void)
{
    return UART_buff_peek(&input_buffer);
}
unsigned char Receive_peek_1(void)
{
    return UART_buff_peek(&input_buffer_1);
}
unsigned char Receive_peek_2(void)
{
    return UART_buff_peek(&input_buffer_2);
}


int Receive_available(void)
{
    return UART_buff_size(&input_buffer);
}
int Receive_available_1(void)
{
    return UART_buff_size(&input_buffer_1);
}
int Receive_available_2(void)
{
    return UART_buff_size(&input_buffer_2);
}
int Receive_available_3(void)
{
    return UART_buff_size(&input_buffer_3);
}


unsigned char Receive_get(void)
{
    return UART_buff_get(&input_buffer);
}
unsigned char Receive_get_1(void)
{
    return UART_buff_get(&input_buffer_1);
}
unsigned char Receive_get_2(void)
{
    return UART_buff_get(&input_buffer_2);
}
unsigned char Receive_get_3(void)
{
    return UART_buff_get(&input_buffer_3);
}

void Send_put(unsigned char _data)
{
    
    //U6TXREG=_data;
    UART_buff_put(&output_buffer, _data);
    if((Transmit_stall == true) && (UART_buff_size(&output_buffer) > 5))
    {
        Transmit_stall = false;
        DRV_USART0_WriteByte(UART_buff_get(&output_buffer));
        /* Enable the Transmit interrupt source */
        //SYS_INT_SourceEnable(INT_SOURCE_USART_1_TRANSMIT);
        DRV_USART0_TransmitInterruptEnable();
    }
}

void Send_put_1(unsigned char _data)
{
    
    //U6TXREG=_data;
    UART_buff_put(&output_buffer_1, _data);
    if((Transmit_stall_1 == true) && (UART_buff_size(&output_buffer_1) > 5))
    {
        Transmit_stall_1 = false;
        DRV_USART2_WriteByte(UART_buff_get(&output_buffer_1));
        /* Enable the Transmit interrupt source */
        //SYS_INT_SourceEnable(INT_SOURCE_USART_1_TRANSMIT);
        DRV_USART2_TransmitInterruptEnable();
    }
    
}void Send_put_2(unsigned char _data)   //UART5 - CAN - DRV3
{
    
    //U6TXREG=_data;
    UART_buff_put(&output_buffer_2, _data);
    if((Transmit_stall_2 == true) && (UART_buff_size(&output_buffer_2) > 5))
    {
        Transmit_stall_2 = false;
        DRV_USART3_WriteByte(UART_buff_get(&output_buffer_2));
        /* Enable the Transmit interrupt source */
        //SYS_INT_SourceEnable(INT_SOURCE_USART_1_TRANSMIT);
        DRV_USART3_TransmitInterruptEnable();
    }
}
void Send_put_3(unsigned char _data)
{
    
    //U6TXREG=_data;
    UART_buff_put(&output_buffer_3, _data);
    if((Transmit_stall_3 == true) && (UART_buff_size(&output_buffer_3) > 5))
    {
        Transmit_stall_3 = false;
        DRV_USART1_WriteByte(UART_buff_get(&output_buffer_3));
        /* Enable the Transmit interrupt source */
        SYS_INT_SourceEnable(INT_SOURCE_USART_6_TRANSMIT);
        //DRV_USART1_TransmitInterruptEnable();
    }
}

void Send_put_debug(unsigned char _data)
{
    //LED7 ^= ON;
    //U6TXREG=_data;
    UART_buff_put(&output_buffer_debug, _data);
    if((Transmit_stall == true) && (UART_buff_size(&output_buffer_debug) > 5))
    {
        Transmit_stall = false;
        DRV_USART3_WriteByte(UART_buff_get(&output_buffer_debug));
        /* Enable the Transmit interrupt source */
        //SYS_INT_SourceEnable(INT_SOURCE_USART_1_TRANSMIT);
        DRV_USART3_TransmitInterruptEnable();
    }
}

void SetTransmitStall(bool _value)
{
    Transmit_stall = _value;
}

void SetTransmitStall_1(bool _value)
{
    Transmit_stall_1 = _value;
}

void SetTransmitStall_2(bool _value)
{
    Transmit_stall_2 = _value;
}

void SetTransmitStall_3(bool _value)
{
    Transmit_stall_3 = _value;
}
//void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void) //<----- DO NOT USE THIS CALL ... IT IS FOR dsPIC33  16bit
//{
//
//    //INDICATOR1=ON;
//    unsigned char data = U1RXREG;
//    UART_buff_put(&input_buffer, data);
//    IFS0bits.U1RXIF = 0; // Clear RX interrupt flag
//    //INDICATOR1=OFF;
//}

//void __ISR(_UART4_TX_VECTOR, IPL1AUTO) _U4TXInterrupt(void)
//{
//
//         //LATBbits.LATB13^=1;
//        if (UART_buff_size(&output_buffer) > 0)
//        {
//            U5TXREG = UART_buff_get(&output_buffer);
//        }
//        else
//        {
//            Transmit_stall = true;
//            IEC5bits.U4TXIE = 0;
//        }
//         //INDICATOR2=OFF;
//
//        IFS5bits.U4TXIF = 0; // Clear TX interrupt flag
//        //IFS2CLR = _IFS2_U5TXIF_MASK;
//}
//void __ISR(_UART3_TX_VECTOR, IPL1AUTO) _U3TXInterrupt(void)
//{
//        //FLASH LED
//        // LATEbits.LATE4^=1;
//        if (UART_buff_size(&output_buffer) > 0)
//        {
//            U4TXREG = UART_buff_get(&output_buffer);
//        }
//        else
//        {
//            Transmit_stall = true;
//            IEC4bits.U3TXIE = 0;
//        }
//         //INDICATOR2=OFF;
//
//        IFS4bits.U3TXIF = 0; // Clear TX interrupt flag
//   // IFS4CLR = _IFS2_U4TXIF_MASK;
//}
