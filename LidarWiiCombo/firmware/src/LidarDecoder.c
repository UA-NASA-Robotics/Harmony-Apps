#include <xc.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "system_definitions.h"
#include "app.h"
#include "uart_handler.h"
#include "FastTransfer.h"
#include "LidarDecoder.h"
#include "LidarDataInterpreter.h"
#include "system/int/sys_int.h"
#include "timers.h"

#define EXPIRED_DATA_INTERVAL 5000

bool isPausedDebug = false;
int startOfErrorSector = 0;
static int counter = 0;
uint16_t numberOfGoodRays=0,lastNumberOfGoodRays=0;
timer_t sendTimer;
uint16_t packetToSend=1;
unsigned long lastUpdateTime[180];


void clearLidarData(void)
{
    int i=0;
    for(i=0;i<180;i++)
    {
        distanceReading[i] = 0;
    }
}

void setupLidarTimers(void)
{
    setTimerInterval(&sendTimer,0);
}

void decode_LidarData()
{
    unsigned char data_Packet[22];
    unsigned short calculated_CRC;
    unsigned short given_CRC;
    
    counter++;
    if(counter >= 1000)
    {
        LED8 ^= 1;
        counter = 0;
    }
    
    //while we potentially have a full packet from the lidar sitting in the buffer and the we don't find any incoherent data that doesn't follow the correct format we will continue looping
    //If a bad/incoherent data packet is found we exit the loop to make sure all the other operations in the code have a chance to be run
    //(if the start byte is not found in the first element of the buffer the the function will allow a 5 byte grace period to find the byte until it returns false). 
    while((UART_buff_size(&lidar_buffer) > 22) )   
    { 
        if((find_Packet(data_Packet)))  //(find_packet)-->will store the full length packet in the given array
        {
            //if the data in this packet is a portion of the data representing the first 180 degrees the lidar outputs
            if((data_Packet[1] <= (unsigned char)LIDAR_FIRST_180_INDEX_BYTE) && (data_Packet[1] >= (unsigned char)LIDAR_MIN_INDEX_BYTE))    
            {
                //Calculating the CRC with Provided formula on  https://xv11hacking.wikispaces.com/LIDAR+Sensor?responseToken=a1fca28ffc7b87087290541350c7b5a3
                calculated_CRC = CRC_calculator(data_Packet);  
                given_CRC = concatenate_Packets(data_Packet[20], data_Packet[21]);  //concatenating the sent CRC for evaluation
                if(calculated_CRC == given_CRC) // If condition is true we have a valid packet of data (if the sent CRC and the calculated CRC match exactly))
                {
                   //Separating the data out into the magnitude and Quality
                   parse_dataBytes(distanceReading, signal_strength, data_Packet, data_Packet[1]);  
                }
            }
        } 
    }
}
int outputCompareValue=1800;
void parse_dataBytes(unsigned short * _magReading, unsigned short * _qualityReading, unsigned char *_data_packet, unsigned char startIndex)
{  
    int FinalIndex = (startIndex - PACKET_INDEX_OFFSET)*4; 
    short inc = 0;
    short packetByteIndex = 0;
    
    if(startIndex == (unsigned char)LIDAR_FIRST_180_INDEX_BYTE)
    {
        lastNumberOfGoodRays=numberOfGoodRays;
        numberOfGoodRays=0;
    }
    
    if(RUN_REALTIME_LIDAR_SPEED_VERIFICATION == true)
    {
        //setting LidarSpeed to the rotation speed of the lidar coming from the bits transmitted by the lidar  <start> <index> <speed_L> <speed_H>
        LidarSpeed = concatenate_Packets(_data_packet[2],_data_packet[3]);
        LidarSpeed = LidarSpeed >> 6;
        
        
        if(!((LidarSpeed < LIDAR_SPEED_UPPER_BOUND) && (LidarSpeed > LIDAR_SPEED_LOWER_BOUND)))
        {            
            if(LidarSpeed > LIDAR_SPEED_UPPER_BOUND) 
            {
                outputCompareValue -= 1;
                DRV_OCO_Change_PulseWidth(outputCompareValue);
            }
            if(LidarSpeed < LIDAR_SPEED_LOWER_BOUND) 
            {
                 outputCompareValue += 1;
                 DRV_OCO_Change_PulseWidth(outputCompareValue);
            }                   
        }         
    }
    
//    DRV_USART3_WriteByte(0xFF);
//    DRV_USART3_WriteByte(LidarSpeed);
//    DRV_USART3_WriteByte(LidarSpeed >> 8);
    for(inc=0;inc<4;inc++)
    {
        bool goodRay=true;
        packetByteIndex = (inc *4) + 4;     //Orients the index on the correct byte in the Packet
        if(error_Verification(_data_packet[packetByteIndex + 1]))//checking to see if the error bit was set (true = good packet/false = bad packet)
        {
            //combining the high and low bits together into the _magReading array
            _magReading[FinalIndex + inc] = concatenate_Packets(_data_packet[packetByteIndex], (_data_packet[packetByteIndex + 1] & CLEAN_UPPER_DISTANCE_BYTE_MASK));
            
            _qualityReading[FinalIndex + inc] = concatenate_Packets(_data_packet[packetByteIndex + 2],(_data_packet[packetByteIndex + 3]));
           
            //Store the last time the data got acquired correctly
            lastUpdateTime[FinalIndex + inc]=millis();
            
            
            //Flashing the beautiful light
            //LED7 ^= 1;
           
            if(startOfErrorSector > 0 && AVERAGE_BAD_RAYS == true)
            { 
                int _angleInc = 0;
                //finding the number of sequential  rays that have errors
              
                //seeing if the the range/number of rays that are bad is within a certain amount AND the left and right good rays are within a reasonable distance 
                //(FinalIndex + inc): is the angle of data that that is being analyzed NOW 
                if(((FinalIndex + inc) - (startOfErrorSector)) <= ERROR_SECTOR_MAX_RANGE && (abs(_magReading[FinalIndex + inc] - _magReading[startOfErrorSector-1]) <= ERROR_SECTOR_RAY_MAX_DELTA))
                {                    
                    //finding the average of the rays on either side of the sector of bad data
                    int avg = ((int)_magReading[FinalIndex + inc] + (int)_magReading[startOfErrorSector-1]) / 2;
                    
                    for(_angleInc = startOfErrorSector; _angleInc < (FinalIndex + inc);_angleInc++)
                    {
                        //setting the error rays with the average of the two adjacent valid rays
                        _magReading[_angleInc] = avg;
                    }
                }
                else
                {
                    for(_angleInc = startOfErrorSector; _angleInc < (FinalIndex + inc);_angleInc++)
                    {
                        //setting the error rays with the average of the two adjacent valid rays
                        _magReading[_angleInc] = LIDAR_MAX_VALUE;
                    } 
                }                
            }
            //reseting the variable that hold the value of the angles that was the start of the sector of bad rays
            startOfErrorSector = 0;
        }
        else
        {
            //if(( a sector of bad has not been detected) AND (the boolean that when set to true allow the average Error operation to be implemented))
            if(startOfErrorSector == 0 && AVERAGE_BAD_RAYS == true)
            {
                //identifying the beginning of a sector of bad rays
                startOfErrorSector = FinalIndex + inc;
            }
            if(SET_BAD_RAYS_TO_MAX == true && AVERAGE_BAD_RAYS == false)
            {
                //_magReading[FinalIndex + inc] = 0;//LIDAR_MAX_VALUE;
                goodRay=false;
            }           
        }        
        if(goodRay)
        {
            numberOfGoodRays++;
        }
        if((lastUpdateTime[FinalIndex + inc]+EXPIRED_DATA_INTERVAL)<millis())
        {
            _magReading[FinalIndex + inc] = 0;
        }
    }
    //***************************************************************************************************************************
    //make sure that the SEND_DATA_TO_DEBUGGER is set to true before you try to send data
    if(SEND_DATA_TO_DEBUGGER )
    {
        if((packetToSend==(startIndex - PACKET_INDEX_OFFSET)))
        {
            if(timerDone(&sendTimer))
            {
                SendFastTransferData(FinalIndex);
                packetToSend++;
                if(packetToSend > 45)
                {
                    packetToSend=0;
                }
            }
        }
        else
        {
            //Send_put_2(FinalIndex);
        }
    }

}

//Warning: if RUN_REALTIME_LIDAR_SPEED_VERIFICATION is set to false this function will not return the speed of the lidar
unsigned short Get_LidarSpeed()
{
    long sum = 0;
    int i = 0;
    //Sum up 8 values of the speed
    for (i = 0; i < 8;i++)
    {
        decode_LidarData();
        sum += LidarSpeed;
    }
   
    return (sum >> 3);  //divide sum by 8 for average
}

void SendFastTransferData(int FinalIndex )
{
    short inc = 0;
   
    ToSend(0,FinalIndex, &debugRingBuff);
    for(inc=0;inc<4;inc++)
    {
        if(DEBUGGER_SEND_DISTANCES == true)
        {
            //sending the distance data
            ToSend(inc+1,distanceReading[FinalIndex+inc], &debugRingBuff); 
        }
        if(DEBUGGER_SEND_QUALITY_DATA == true)
        {
            //sending the quality/reflectance data
            ToSend(inc+5,signal_strength[FinalIndex+inc], &debugRingBuff);
        }
    }

    ToSend(18,LidarSpeed,&debugRingBuff);
    ToSend(19,outputCompareValue,&debugRingBuff);
    sendData(4, &debugRingBuff);
}

uint16_t getNumberGoodRays(void)
{
    return lastNumberOfGoodRays;
}

//This function should return a true or false based on if the <invalid data flag> status bit in Byte 1 of [Data 0] [Data 1] [Data 2] [Data 3]
//WARNING: THE VALUE OF "byteIndex" should be a value between 0-3
//the byteIndex is the Data element in the packet received from the lidar 
bool error_Verification(unsigned char _data_packet) 
{
    if((_data_packet & INVALID_DATA_FLAG_STATUS_MASK) == 0)
    {
        return true;    //The error bit is not set
    }
    else
    {
        return false;
    }
}

bool find_Packet(unsigned char * _this)
{
//    int c;
    //giving a grace period incase there is some inconsistency with the a data
//    for(c = 0;c < 5; c++)
//    {
    
        if(UART_buff_get(&lidar_buffer) == 0xFA)
        {
            unsigned short inc;
            _this[0] = 0xFA;    //insert the <start> byte since it will no longer be in the lidar_buffer
            for(inc = 1; inc < 22; inc++)   //increments through the next 21 bytes and verifies there isn't a misplaced <start>, then continues to place the bytes in the array
            {
                if(UART_buff_peek(&lidar_buffer) != 0xFA)       //Looking for <start>
                {
                    _this[inc] = UART_buff_get(&lidar_buffer);
                }
                else
                {
                    return false;       //AHHH-HHHA just found the <start> byte where it is not supposed to be
                }
            }
            return true;            
        }
//    }   
}
 
unsigned short CRC_calculator(unsigned char * _this) {
    unsigned short dataList[10];
    unsigned short IndexTwoBytesGrouped = 0;
    unsigned char IndexEachByte = 0;
    unsigned int chk32 = 0;

    for (IndexTwoBytesGrouped = 0; IndexTwoBytesGrouped < 10; IndexTwoBytesGrouped++) {
        // group the data by word(16 bits), little-endian (LSB stays at LSB and MSB shifts left by 8 when bytes are "concatinated")
        dataList[IndexTwoBytesGrouped] = _this[2 * IndexEachByte] + (((unsigned short)_this[2 * IndexEachByte + 1]) << 8);
        IndexEachByte++;
    }

    for (IndexTwoBytesGrouped = 0; IndexTwoBytesGrouped < 10; IndexTwoBytesGrouped++) {
        chk32 = (chk32 << 1) + dataList[IndexTwoBytesGrouped];
    }

    // return a value wrapped around on 15bits, and truncated to still fit into 15 bits
    unsigned short checksum = (chk32 & 0x7FFF) + (chk32 >> 15);
    return (checksum & 0x7FFF);
}

unsigned short concatenate_Packets(unsigned char lower, unsigned char upper)
{
    //add the upper after bit shifting the upper by 8 bits then anding it with 0xFF00 to remove any extraneous bits in [7:0] 
    return (lower + ((upper << 8)& 0xFF00));    

}

unsigned short getDistanceReading(int index)
{
    return distanceReading[index];
}
unsigned short getSignalStrength(int index)
{
    return signal_strength[index];
}
