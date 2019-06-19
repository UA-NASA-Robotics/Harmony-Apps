/* 
 * File:   STD_HelperMethods.h
 * Author: John
 *
 * Created on April 27, 2017, 7:49 AM
 */

#ifndef STD_HELPERMETHODS_H
#define	STD_HELPERMETHODS_H

bool IsParallel(double _Slope);
double getSlope(unsigned short RightInc, unsigned short LeftInc, unsigned short RightMag,unsigned short LeftMag);
bool isWithinTolerance(double _testVal, double target, double _delta);
unsigned short trig_ABS(double _val);
struct pointD PolarToCartesian(unsigned short angle, unsigned short mag);
struct pointD PolarToCartesianTwo(unsigned short angle, unsigned short mag);
unsigned short Calculate_PointDistance(struct pointD pointOne, struct pointD pointTwo);
bool isWithinArea(int _angle, unsigned short _mag,unsigned short *MAGarray);
double Dabs(double _Val);
//double Buffer_Get(struct SlopeRingBuffer *_buff);
//void Buffer_Add(struct SlopeRingBuffer *_buff, double _data);
unsigned short SlopeReflectance(unsigned short _startAngle, unsigned short _endAngle);

#endif	/* STD_HELPERMETHODS_H */

