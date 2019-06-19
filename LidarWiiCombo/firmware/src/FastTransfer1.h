/* 
 * File:   FastTransfer.h
 * Author: Igor
 *
 * Created on March 23, 2015, 1:21 PM
 */

#ifndef FASTTRANSFER1_H
#define	FASTTRANSFER1_H



void (*serial_write1)(unsigned char);
unsigned char (*serial_read1)(void);
int (*serial_available1)(void);
unsigned char (*serial_peek1)(void);
unsigned char rx_buffer1[255]; //address for temporary storage and parsing buffer
unsigned char rx_array_inx1; //index for RX parsing buffer
unsigned char rx_len1; //RX packet length according to the packet
unsigned char calc_CS1; //calculated Checksum
unsigned char moduleAddress1; // the address of this module
unsigned char returnAddress1; //the address to send the crc back to
unsigned char maxDataAddress1; //max address allowable
int * receiveArrayAddress1; // this is where the data will go when it is received
unsigned char * sendStructAddress1; // this is where the data will be sent from
bool AKNAKsend1; // turns the acknowledged or not acknowledged on/off
unsigned int alignErrorCounter1; //counts the align errors
unsigned int crcErrorCounter1; // counts any failed crcs
unsigned int addressErrorCounter1; // counts every time a wrong address is received
unsigned int dataAdressErrorCounter1; // counts if the received data fall outside of the receive array
unsigned char rx_address1; //RX address received
#define polynomial 0x8C  //polynomial used to calculate crc
#define CRC_COUNT 5 // how many AKNAKs are stored
#define CRC_DEPTH 3  // how many pieces of data are stored with each CRC send event
#define CRC_BUFFER_SIZE (CRC_COUNT * CRC_DEPTH) //crc buffer size 5 deep and 3 bytes an entry


struct crcBufS crc_buffer1;

union stuff group1;

void begin1(int * ptr, unsigned char maxSize, unsigned char givenAddress, bool error, void (*stufftosend)(unsigned char), unsigned char (*stufftoreceive)(void),int (*stuffavailable)(void), unsigned char (*stuffpeek)(void));
bool sendData1(unsigned char whereToSend);
bool receiveData1();
void ToSend1(const unsigned char where, const unsigned int what);


void CRCcheck1(void);




#endif	/* FASTTRANSFER_H */

