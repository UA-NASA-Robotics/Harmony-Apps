#include <xc.h>
#include <stdbool.h>
#include <stdlib.h>
#include "system_config/default/system_definitions.h"
//#include "uart_handler.h"
#include "FastTransfer.h"
#include "FastTransfer1.h"
//#include "app.h"
//typedef struct
//{
//    unsigned char buf[BUFFER_SIZE];
//    int head;
//    int tail;
//    int count;
//}ringBuf_t;

struct ringBufS ringBuffSend;



//Captures address of receive array, the max data address, the address of the module, true/false if AKNAKs are wanted and the Serial address

void begin1(int * ptr, unsigned char maxSize, unsigned char givenAddress, bool error, void (*stufftosend)(unsigned char), unsigned char (*stufftoreceive)(void),int (*stuffavailable)(void), unsigned char (*stuffpeek)(void))
{
    receiveArrayAddress1 = ptr;
    moduleAddress1 = givenAddress;
    serial_write1 = stufftosend;
    serial_available1 = stuffavailable;
    serial_peek1 = stuffpeek;
    serial_read1 = stufftoreceive;
    maxDataAddress1 = maxSize / 2;
    //sendStructAddress = (unsigned char*) & ring_buffer;
    AKNAKsend1 = error;
    alignErrorCounter1 = 0;

}


//Sends out send buffer with a 2 start bytes, where the packet is going, where it came from, the size of the data packet, the data and the crc.

bool sendData1(unsigned char whereToSend )
{
   
    //calculate the crc
    unsigned char CS = CRC8(ringBuffSend.buf, ringBuffSend.count);
    
    serial_write1(0x06); //start address
    serial_write1(0x85); //start address
    serial_write1(whereToSend);
    serial_write1(moduleAddress1);
    serial_write1(ringBuffSend.count); //length of packet not including the crc


    //send the rest of the packet
    int i;
    for (i = 0; i < ringBuffSend.count; i++)
    {
        serial_write1(*(ringBuffSend.buf + i));
    }
    
    //send the crc
    serial_write1(CS);

    // clears the buffer after a sending
    FastTransfer_buffer_flush(&ringBuffSend, 1);
    return true;
}

bool receiveData1()
{
    //start off by looking for the header bytes. If they were already found in a previous call, skip it.
    if (rx_len1 == 0)
    {
        //this size check may be redundant due to the size check below, but for now I'll leave it the way it is.
        if (serial_available1() > 4)
        {
            
            //this will block until a 0x06 is found or buffer size becomes less then 3.
            while (serial_read1() != 0x06)
            {
                //This will trash any preamble junk in the serial buffer
                //but we need to make sure there is enough in the buffer to process while we trash the rest
                //if the buffer becomes too empty, we will escape and try again on the next call
                alignErrorCounter1++; //increments the counter whenever a byte is trashed
                if (serial_available1() < 5)
                    return false;
            }
            if (serial_read1() == 0x85)
            {
                rx_address1 = serial_read1(); // pulls the address
                returnAddress1 = serial_read1(); // pulls where the message came from
                rx_len1 = serial_read1(); // pulls the length
               
                               
                //make sure the address received is a match for this module if not throw the packet away
              
                if (rx_address1 != moduleAddress1)
                {
                    addressErrorCounter1++; // increments a counter whenever the wrong address is received
                    //if the address does not match the buffer is flushed for the size of
                    //the data packet plus one for the CRC
                    int u;
                    for (u = 0; u <= (rx_len1 + 1); u++)
                    {
                        serial_read1();
                    }
                    rx_len1 = 0; // reset length
                    return false;
                }
                // if the address matches the a dynamic buffer is created to store the received data
                //rx_buffer1 = (unsigned char*) malloc(rx_len + 1);
            }
        }
    }

    //we get here if we already found the header bytes, the address matched what we know, and now we are byte aligned.
    if (rx_len1 != 0)
    {
        //this check is preformed to see if the first data address is a 255, if it is then this packet is an AKNAK
        if (rx_array_inx1 == 0)
        {
            while (!(serial_available1() >= 1));
            if (255 == serial_peek1())
            {
                CRCcheck1();
                rx_len1 = 0;
                rx_array_inx1 = 0;
                //free(rx_buffer1);
                
                wipeBuf(rx_buffer1,255);
                return receiveData();
            }
        }

        
        while ((serial_available1()) > 0 && ( rx_array_inx1 <= rx_len1))//(serial_available1() > 0) &&( rx_array_inx <= rx_len))
        {  
             rx_buffer1[rx_array_inx1++] = serial_read1();
              
        }

        if (rx_len1 == (rx_array_inx1 - 1))
        {
            //seem to have got whole message
            //last uint8_t is CS
            calc_CS1 = CRC8(rx_buffer1, rx_len1);


            
            if (calc_CS1 == rx_buffer1[rx_array_inx1 - 1])
            {//CS good

                // reassembles the data and places it into the receive array according to data address.
                int r;
                for (r = 0; r < rx_len1; r = r + 3)
                {
                    if (rx_buffer1[r] < maxDataAddress1)
                    {
                        group1.parts[0] = rx_buffer1[r + 1];
                        group1.parts[1] = rx_buffer1[r + 2];
                        
                        receiveArrayAddress1[(rx_buffer1[r])] = group1.integer;
                        
                        
                    } else
                    {
                        dataAdressErrorCounter1++;
                    }
                }

                useFastTransData(receiveArrayAddress1);

                if (AKNAKsend1)
                { // if enabled sends an AK
                    unsigned char holder[3];
                    holder[0] = 255;
                    holder[1] = 1;
                    holder[2] = rx_buffer1[rx_array_inx1 - 1];
                    unsigned char crcHolder = CRC8(holder, 3);
                    serial_write1(0x06);
                    serial_write1(0x85);
                    serial_write1(returnAddress1);
                    serial_write1(moduleAddress1);
                    serial_write1(3);
                    serial_write1(255);
                    serial_write1(1);
                    serial_write1(rx_buffer1[rx_array_inx1 - 1]);
                    serial_write1(crcHolder);
                }


                
                rx_len1 = 0;
                rx_array_inx1 = 0;
                //free(rx_buffer1);
               
                wipeBuf(rx_buffer1,255);
                return true;
            }
            else
            {
                crcErrorCounter1++; //increments the counter every time a crc fails

                if (AKNAKsend1)
                { // if enabled sends NAK
                    unsigned char holder[3];
                    holder[0] = 255;
                    holder[1] = 2;
                    holder[2] = rx_buffer1[rx_array_inx1 - 1];
                    unsigned char crcHolder = CRC8(holder, 3);
                    serial_write1(0x06);
                    serial_write1(0x85);
                    serial_write1(returnAddress1);
                    serial_write1(moduleAddress1);
                    serial_write1(3);
                    serial_write1(255);
                    serial_write1(2);
                    serial_write1(rx_buffer1[rx_array_inx1 - 1]);
                    serial_write1(crcHolder);
                }
                
                //failed checksum, need to clear this out
                rx_len1 = 0;
                rx_array_inx1 = 0;
                //free(rx_buffer1);
                
                wipeBuf(rx_buffer1,255);
                return false;
            }
        }
    }


    return false;
}


// populates to what data address and what info needs to be sent

void ToSend1(unsigned char where, unsigned int what)
{
    FastTransfer_buffer_put(&ringBuffSend, where, what);
}

//when an AK or NAK is received this compares it to the buffer and records the status
void CRCcheck1(void)
{

    while (!(serial_available1() > 3)); // trap makes sure that there are enough bytes in the buffer for the AKNAK check

    unsigned char arrayHolder[3];
    arrayHolder[0] = serial_read1();
    arrayHolder[1] = serial_read1();
    arrayHolder[2] = serial_read1();
    unsigned char SentCRC = serial_read1();
    unsigned char calculatedCRC = CRC8(arrayHolder, 3);


    if (SentCRC == calculatedCRC)
    {

        int rt;
        for (rt = 0; rt < CRC_COUNT; rt++)
        {
            if (returnAddress1 == crcBufS_get(&crc_buffer1, rt, 0))
            {
                if (arrayHolder[2] == crcBufS_get(&crc_buffer1, rt, 1))
                {
                    if (arrayHolder[1] == 1)
                    {
                        crcBufS_status_put(&crc_buffer1, rt, 1);
                        break;
                    } else if (arrayHolder[1] == 2)
                    {
                        crcBufS_status_put(&crc_buffer1, rt, 2);
                        break;
                    }
                }
            }
        }
    } else
    {
        crcErrorCounter1++;
    } //increments the counter every time a crc fails
}

