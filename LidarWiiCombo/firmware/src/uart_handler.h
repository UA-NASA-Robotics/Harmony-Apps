/* File:   uart_handler.h
 * Author: Igor
 * Created on July 5, 2015, 8:17 AM
 */

#ifndef UART_HANDLER_H
#define	UART_HANDLER_H



//Ring Buffer parameters for input UART (UART 4: U4RX)
#define UART_BUFFER_SIZE 1000


void myprintf(char * stringed);
void wipeBuf(unsigned char * buf, unsigned int numelems);

//void *memset(void *s, int c, size_t n);

struct UART_ring_buff {
    unsigned char buf[UART_BUFFER_SIZE];
    int head;
    int tail;
    int count;
};

struct UART_ring_buff input_buffer;     //this holds the incoming data from fastTrans 
struct UART_ring_buff output_buffer;    //this holds the complete message to be sent out through fastTrans (NOW LIDAR)

struct UART_ring_buff input_buffer_1;     //this holds the incoming data from fastTrans_1
struct UART_ring_buff output_buffer_1;    //this holds the complete message to be sent out through fastTrans_1

struct UART_ring_buff input_buffer_2;     //this holds the incoming data from fastTrans_2
struct UART_ring_buff output_buffer_2;    //this holds the complete message to be sent out through fastTrans_2

struct UART_ring_buff input_buffer_3;     //this holds the incoming data from lidar
struct UART_ring_buff output_buffer_3;    //this holds the complete message to be sent out through a pin that doesnt work

struct UART_ring_buff output_buffer_debug;  //this holds the output data for the debugging
struct UART_ring_buff lidar_buffer;     //this holds the data coming in from the lidar


void UART_init(void);
void UART_buff_init(struct UART_ring_buff* _this);
void UART_buff_put(struct UART_ring_buff* _this, const unsigned char c);
unsigned char UART_buff_get(struct UART_ring_buff* _this);
void UART_buff_flush(struct UART_ring_buff* _this, const int clearBuffer);
int UART_buff_size(struct UART_ring_buff* _this);
unsigned int UART_buff_modulo_inc(const unsigned int value, const unsigned int modulus);
unsigned char UART_buff_peek(struct UART_ring_buff* _this);

void Send_put_debug(unsigned char _data);
unsigned char Receive_peek(void);
int Receive_available(void);
unsigned char Receive_get(void);
void Send_put(unsigned char _data);

//FASTTRANS_1
unsigned char Receive_peek_1(void);
int Receive_available_1(void);
unsigned char Receive_get_1(void);
void Send_put_1(unsigned char _data);

//FASTTRANS_1
unsigned char Receive_peek_2(void);
int Receive_available_2(void);
unsigned char Receive_get_2(void);
void Send_put_2(unsigned char _data);

//LIDAR STUFF
int Receive_available_3(void);
unsigned char Receive_get_3(void);
void Send_put_3(unsigned char _data);

void SetTransmitStall(bool _value);
void SetTransmitStall_1(bool _value);
void SetTransmitStall_2(bool _value);
void SetTransmitStall_3(bool _value);

bool Transmit_stall;
bool Transmit_stall_1;
bool Transmit_stall_2;
bool Transmit_stall_3;

#endif	/* UART_HANDLER_H */