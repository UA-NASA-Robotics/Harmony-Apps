#ifndef MULTI_LIDAR_H
#define MULTI_LIDAR_H

#include "FastTransfer.h"
#include <stdbool.h>

#define LIDAR_IS_MASTER             true

#define LIDAR_MASTER				1
#define LIDAR_SLAVE					2

#define LIDAR_RECEIVE_ARRAY_LENGTH 	5

#define FT_ADDRESS_X 				1
#define FT_ADDRESS_Y				2
#define FT_ADDRESS_ANGLE			3
#define FT_ADDRESS_DIST				4

void initFTLIDARPair(bool masta);
void sendLIDARObject(int x,int y, int angle, int dist);
int getSecondaryAngle();
int getSecondaryDist();



#endif