#include <stdbool.h>
#include <math.h>
#include "system_config/default/system_definitions.h"
#include "LidarDataInterpreter.h"
#include "STD_HelperMethods.h"
#include "LidarDecoder.h"



//Returns true if the slope is within tolerence of being parallel with the arena wall(collection bin)
bool IsParallel(double _Slope)
{
    //geting the slope from the polar values of the right and left bounds
    //double slope = getSlope(RightInc,LeftInc,RightMag, LeftMag) * 1000.00;
    //verifying if the slope calculated is close enough to the zero slope. If true then the reflective slope is parallel to the wall
    if(isWithinTolerance(_Slope, 0, PARALLEL_SLOPE_TOLERENCE))
    {
        return true;
    }
    else
    {
        return false;
    }
}
double getSlope(unsigned short RightInc, unsigned short LeftInc, unsigned short RightMag,unsigned short LeftMag)
{
    //geting the points of the two sides of the object
    struct pointD rightPoint = PolarToCartesianTwo(RightInc, RightMag);
    struct pointD leftPoint = PolarToCartesianTwo(LeftInc, LeftMag);
    unsigned short den = rightPoint.x - leftPoint.x;
    if(den == 0)
    {
        return NINETY_DEG_RETURN_VAL;
    }
    double slope = ((rightPoint.y - leftPoint.y)/(rightPoint.x - leftPoint.x));
    if(slope > SLOPE_OF_NINETY_DEG)
    {
        return NINETY_DEG_RETURN_VAL;
    }
    //calculating the slope between the two points
    return slope;
}

bool isWithinTolerance(double _testVal, double target, double _delta)
{
    if(Dabs(target - _testVal) > _delta)
    {
        return false;
    }
    else
    {
        return true;
    }
}
double Dabs(double _Val)
{
    if(_Val < 0)
    {
        return -_Val;
    }
    else
    {
        return _Val;
    }
}
struct pointD tmpPointTwo;
struct pointD PolarToCartesianTwo(unsigned short angle, unsigned short mag)
{
//    DRV_USART3_WriteByte(0x55);
//    DRV_USART3_WriteByte(angle);
    //this will store the two captured locations of the robot in cartesian form
    tmpPointTwo.x = (double)mag * cos(((double)angle * DEGREE_TO_RAD));
    tmpPointTwo.y = (double)mag * sin(((double)angle * DEGREE_TO_RAD));  
   
//    DRV_USART3_WriteByte((((int)mag) >> 8));
//    DRV_USART3_WriteByte(((int)mag));
    return tmpPointTwo;    
} 
struct pointD tmpPoint;
struct pointD PolarToCartesian(unsigned short angle, unsigned short mag)
{
   
    //this will store the two captured locations of the robot in cartesian form
    tmpPoint.x = (double)mag * cos(((double)angle * DEGREE_TO_RAD));
    tmpPoint.y = (double)mag * sin(((double)angle * DEGREE_TO_RAD));  
   

    return tmpPoint;    
} 
unsigned short Calculate_PointDistance(struct pointD pointOne, struct pointD pointTwo)
{
    return sqrt(pow(pointTwo.y - pointOne.y,2) + pow(pointTwo.x - pointOne.x,2));
}
//This function takes in an angle and magnitude and verifies if it is inside the bounds of the virtually setup box
//to change the size of the box Edit the #defines in the LidarDataInterpreter.h file on lines 16-17-18 (LEFTSIDE_DISTANCE, RIGHTSIDE_DISTANCE, AREA_LENGTH)
bool isWithinArea(int _angle, unsigned short _mag,unsigned short *MAGarray)
{
    if(_mag > MAGarray[_angle])
    {
        return false;
    }
    else
    {
        return true;
    }
}

//void Buffer_Add(struct SlopeRingBuffer *_buff, double _data)
//{
//    _buff->slope[_buff->head] = _data;
//    _buff->head++;
//    if(_buff->head >= SLOPE_DEVIATION_GRACE_PERIOD )
//    {
//        _buff->head = 0;
//        if(_buff->head = _buff->tail){
//            _buff->tail++;
//        }
//    }
//}
//double Buffer_Get(struct SlopeRingBuffer *_buff)
//{
//    _buff->tail++;
//    if(_buff->tail >= SLOPE_DEVIATION_GRACE_PERIOD )
//    {
//        _buff->tail = 0;
//        if(_buff->head = _buff->tail){
//            _buff->tail--;
//        }
//    }
//    
//    return _buff->slope[_buff->tail];
//}
unsigned short SlopeReflectance(unsigned short _startAngle, unsigned short _endAngle)
{
    int i;
    long summation = 0;

    for (i = _startAngle; i < _endAngle;i++)
    {
        summation += getSignalStrength(i);
    }
    //finding the average....
    return ((double)summation / (double)(_endAngle - _startAngle));
}