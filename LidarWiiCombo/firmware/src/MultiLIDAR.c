#include "MultiLIDAR.h"
#include "FastTransfer1.h"
#include "uart_handler.h"
#include <math.h>
#include "system_config/default/system_definitions.h"

#define LIDAR_DISTANCE_BETWEEN 1.5

int receiveArray1[LIDAR_RECEIVE_ARRAY_LENGTH];
int x,y,angle,dist;

void initFTLIDARPair(bool masta)
{
    if(masta== LIDAR_IS_MASTER)
		begin1(receiveArray1, sizeof(receiveArray1), LIDAR_MASTER, false, Send_put, Receive_get, Receive_available, Receive_peek);  
	else
		begin1(receiveArray1, sizeof(receiveArray1), LIDAR_SLAVE, false, Send_put, Receive_get, Receive_available, Receive_peek);  
	
}

void sendLIDARObject(int x,int y, int angle, int dist)
{
//	ToSend1(FT_ADDRESS_X, x);
//	ToSend1(FT_ADDRESS_Y, y);
	ToSend1(FT_ADDRESS_ANGLE,angle);
	ToSend1(FT_ADDRESS_DIST,dist);
	sendData1(LIDAR_MASTER);
}

void receiveLIDARData(void)
{
	if(receiveData1())
	{
        LED7 ^=1;
		x		=receiveArray1[FT_ADDRESS_X];
		y		=receiveArray1[FT_ADDRESS_Y];
		angle	=receiveArray1[FT_ADDRESS_ANGLE];        
		dist	=receiveArray1[FT_ADDRESS_DIST];
	}
}

int getSecondaryAngle()
{
    return angle;
}

int getSecondaryDist()
{
    return dist;
}

//Called from the application on master, passes the local LIDAR data about an object
//and uses the data gathered above to compare the data sets to form the environment
//Assume this LIDAR is on the left
//https://www.mathsisfun.com/algebra/trig-solving-asa-triangles.html

//
//                    C
//                    /\
//                   /  \
//                  /    \
//                b/      \a
//                /        \
//               /          \
//              /____________\
//				A	   c      B
void findRobotCenter(int x1,int y1, int angle1, int dist1)
{
	//compare
	//ASA
	//angle1 - CONST DIST - angle
	int angleC = 180-angle1-angle;
    //leg 'a'
	float distLIDARRight = (LIDAR_DISTANCE_BETWEEN/sin(angleC)) * sin(angle1);
    //leg 'b'
	float distLIDARLeft  = (LIDAR_DISTANCE_BETWEEN/sin(angleC)) * sin(angle);
	
	
	
}