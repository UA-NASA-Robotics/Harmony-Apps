/* 
 * File:   LidarDataInterpreter.h
 * Author: Seth Carpenter
 *
 * Created on January 2, 2017, 1:28 PM
 */

#ifndef LIDARDATAINTERPRETER_H
#define	LIDARDATAINTERPRETER_H




#define ROBOT_WIDTH 750//cm
#define GROUND_DEADZONE 10

unsigned short LastMag;
unsigned short LidarDistanceReading[180];
unsigned short object_Angles[MAX_OBJECTS * 2];
unsigned short object_Mag[MAX_OBJECTS * 2];
unsigned short MaxMAG_LookUpTable[180];


//***Ground Objects***


unsigned short groundRange;
//******************************
bool initialPlacmentCaptured;
bool firstPlacmentCaptured;
bool ContinuousTranmission;

bool isSingleSideSeen;

unsigned char TransmissionMode;
void LidarDataInterpreter();

void useFastTransData(int * receiveArrayAddress);
void transmitOBJ_Data(bool _transOBJ_angle, bool _transOBJ_heading, bool _transOBJ_AtoB, bool _transOBJ_mag);

struct objHeading
{
    unsigned short StartAngle;
    unsigned short EndAngle;
    unsigned short StartMag;
    unsigned short EndMag;
    unsigned short Heading;
    unsigned short AtoB_dist;
    unsigned short Old_Heading;
    unsigned short Old_AtoB_dist;
};

struct object
{
    unsigned short rightSideAngle;
    unsigned short leftSideAngle;
    unsigned short rightSideMAG;
    unsigned short leftSideMAG;
    unsigned short objectAngle;
    unsigned short AVG_OBJangle;
    unsigned short Magnitude;
    unsigned short AVG_OBJmag;
    unsigned short ReflectiveSlope;
    bool isSlopeALinear;
    double SlopeA,SlopeB;
    unsigned short slopeAReflect, slopeBReflect;
    unsigned short Corner;
    unsigned short Stored_mags[AVERAGE_ITEMS_COUNT];
    unsigned short Stored_angles[AVERAGE_ITEMS_COUNT];
    struct objHeading Heading;
};
struct object LidarOBJ[MAX_OBJECTS];


//struct SlopeRingBuffer
//{
//    double slope[SLOPE_DEVIATION_GRACE_PERIOD];
//    short head ;
//    short tail;
//};
//struct SlopeRingBuffer SlopeBuff;

//struct pointD
//{
//    double x;
//    double y;
//};


void startObjectDetection();
//bool isWithinArea(int _angle, unsigned short _mag);
void Generate_LookUpTable();

unsigned short Get_objMag();




void Calculate_Heading();
//unsigned short Calculate_PointDistance(struct pointD pointOne, struct pointD pointTwo);
//struct pointD PolarToCartesian(unsigned short angle, unsigned short mag);
void captureOBJ_placment(bool _transmit);
unsigned short Get_objAngle();
//unsigned short trig_ABS(double _val);
void LEAPFROG_OBJ_CAPTURE();
void set_objAVG(int _OBJIndex);
void set_OBJangle();
void EndOBJ_found(int inc);
void CartesianCenter(unsigned short leftAngle, unsigned short leftMag, unsigned short rightAngle, unsigned short rightMag);
void SlopeOBJ_IdentifierMethod(unsigned short leftAngle, unsigned short rightAngle);
int getObjctsFound();
void FindSlopes(unsigned short rightAngle, unsigned short leftAngle);
void SlopeFunction(unsigned short rightAngle, unsigned short leftAngle);
void SendOBJData_debug();
//bool isWithinTolerance(unsigned short _testVal, unsigned short target, int _delta);
//bool isWithinTolerance(double _testVal, double target, double _delta);
//void IsParallel(unsigned short LeftInc, unsigned short RightInc, unsigned short RightMag,unsigned short LeftMag);
//double getSlope(unsigned short LeftInc, unsigned short RightInc, unsigned short RightMag,unsigned short LeftMag);
#endif	/* LIDARDATAINTERPRETER_H */
/*---------------------------------------------DEFINITIONS & TERMANOLOGIES--------------------------------------------------------------/
 * 
 * -->THE LEAPFROG METHOD:
 *      This method is a dynamic way of capturing the Object's heading and displacement. Every time this functionality is called, the object heading and displacement is calculated by using a previously 
 *      stored placement of the object. The current placement of the object is then stored for use of the next call to this functionality. 
 * 
 * 
 */
