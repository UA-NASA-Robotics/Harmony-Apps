#include <stdbool.h>
#include <math.h>
#include "system_config/default/system_definitions.h"
#include "GroundOBJprocessing.h"
#include "LidarDataInterpreter.h"



//unsigned short object_Angles[MAX_OBJECTS * 2];
//unsigned short object_Mag[MAX_OBJECTS * 2];
const int MAX_ANGLE = 250;  //this is the max angle the code will use to find an object on the ground
const int MIN_ANGLE = 30;   //this is the min angle the code will use to find an object on the ground
unsigned short MaxMAGground_LookUpTable[180];
unsigned short groundRange;
struct object GroundOBJ;
//void startGroundObjectDetection()
//{
//   //use the lidar width #define to minimize the amount of trig that need to be done. so the code is only analyzing what is directly in front of the robot.
//    bool objRightSideFound = false;
//    int _ang;
//    for(_ang = MIN_ANGLE; _ang < MAX_ANGLE;_ang++)
//    {
//       if(abs(MaxMAGground_LookUpTable[_ang] - getDistanceReading(_ang)) > GROUND_DEADZONE)
//       {
//           if(objRightSideFound == false)
//           {
//              GroundOBJ.rightSideAngle = _ang; 
//           }
//           else
//           {
//               GroundOBJ.leftSideAngle = _ang;
//           }
//           
//       }
//    }
//    
//}
void Set_GroundMaxRange()
{
    groundRange = getDistanceReading(90);
    Generate_GroundLookUpTable(getDistanceReading(90));
}

void Generate_GroundLookUpTable()
{
   int _angle = 0;
        int quad = 0;
        MaxMAGground_LookUpTable[90] = AREA_LENGTH;
        for(quad = 0; quad < 2; quad++)
        {

            //calculating the max magnitude the is allowed inside the virtual box from angles 91 -180)
            MaxMAGground_LookUpTable[_angle] = MaxMAGground_LookUpTable[_angle - ((_angle - 90)*2)];
           
        }
        //Setting a max range1573 3000

            for(_angle = 0; _angle < 90; _angle++)
            {
                if(quad == 0)
                {
                    //will only calculate the magnitudes for the end of the virtual box( max distances would only be calculated for the angle that will fall in between the intersection of the maxwidth and max length)
                    if(_angle > (int)(atan(AREA_LENGTH/RIGHTSIDE_DISTANCE) * RAD_TO_DEGREE))
                    {
                       MaxMAGground_LookUpTable[_angle] = (AREA_LENGTH / cos(((90-_angle)*DEGREE_TO_RAD)));
                    }
                }
                else
                {
                    //just copying the value that have already been calculated over to the second quadrant 
                    MaxMAGground_LookUpTable[180 - _angle] = MaxMAGground_LookUpTable[_angle];
                }
            }
          

        
    }