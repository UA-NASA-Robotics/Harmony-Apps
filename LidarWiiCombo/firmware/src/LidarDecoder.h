/* 
 * File:   LidarDecoder.h
 * Author: Seth Carpenter
 *
 * Created on December 22, 2016, 1:02 PM
 * this code is for the University of Akron's NASA robotic team.
 * The purpose of this code is to take data from a NETO lidar system and parse the packet into individual array to be analyzed 
 */

#ifndef LIDARDECODER_H
#define	LIDARDECODER_H



void decode_LidarData();
unsigned short CRC_calculator(unsigned char * _this);
bool find_Packet(unsigned char * _this);
unsigned short concatenate_Packets(unsigned char lower, unsigned char upper);
void SendBytes_debug(int startByte, int endByte, unsigned char * _this);
void parse_dataBytes(unsigned short * _magReading, unsigned short * _qualityReading, unsigned char *_data_packet, unsigned char startIndex);
bool error_Verification(unsigned char _data_packet); 
void SendFastTransferData();
unsigned short Get_LidarSpeed();
unsigned short getDistanceReading(int index);
unsigned short getSignalStrength(int index);
uint16_t getNumberGoodRays(void);
void setupLidarTimers(void);
void clearLidarData(void);
void getprintcal(void);

#endif	/* LIDARDECODER_H */

