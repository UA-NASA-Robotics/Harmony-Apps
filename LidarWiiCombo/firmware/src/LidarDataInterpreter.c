#include <stdbool.h>
#include <math.h>
#include "system_definitions.h"
#include "FastTransfer.h"
#include "app.h"
#include "LidarDataInterpreter.h"
#include "LidarDecoder.h"
#include "STD_HelperMethods.h"

bool ObjStartEdgeFound = false;
unsigned short LastMag;
int ObjectsFound = 0;
bool isObj = false;
void clearOBJvariables();
int Trans1 = 0,Trans2 = 0;
int getObjctsFound()
{
    return ObjectsFound;
}


void startObjectDetection()
{
    ObjStartEdgeFound = false;
    ObjectsFound = 0;
    //setting the first value to a variable (LastMag) so I can start analyzing from entry 1 and compare it with entry 0
    LastMag = distanceReading[0]; 
    


    //Initializing a loop counter
    int inc;
    //looping through the entire Array of distanceReading to find a "jump" in the magnitudes. this will be the start of finding an object.
    for(inc = 1;inc < 180;inc++)    
    {
       
        //if this becomes true, the difference in the current and last angle magnitudes should be greater than the Constant OBJECT_IDENTIFIER_MAG_RANGE
        if((abs(distanceReading[inc] - LastMag) >= OBJECT_IDENTIFIER_RANGE) && distanceReading[inc] > 0)
        {
            //Have we found the beginning of and object ans is the next point farther from the last 
            if(ObjStartEdgeFound == true && distanceReading[inc] > LastMag && isWithinArea(inc, distanceReading[inc - 1],MaxMAG_LookUpTable) == true )
            {
                if(ENABLE_OBJ_GRACE_PERIOD)
                {
                    //this set of functionality allows me to look 4 Steps in the future to see if this is really the end of the obj
                    bool isThisTheEND = true;
                    int goodVal = 0;
                    int q;
                    for(q = inc + 1;q < ((inc)+OBJ_END_GRACE_PERIOD_VAL);q++)
                    {
                        if(isThisTheEND)
                        {
                            distanceReading[q] = distanceReading[inc -1];
                            if(q == goodVal)
                            {
                                q = (inc + OBJ_END_GRACE_PERIOD_VAL);
                            }
                        }
                        else
                        {
                            if(abs(distanceReading[q] - LastMag) <= GRACE_P_SIGMA_MAG)
                            {
                               
                                isThisTheEND = false;
                                //jump out of for loop so that we don't wast time running through it even though we set isThisTheEND to false
                                goodVal = q;
                                q = ((inc)-1);

                            }
                        }
                    }
                    if(isThisTheEND)
                    {
                        if(distanceReading[inc] > LastMag)
                            EndOBJ_found(inc);

                    }
                }
                else
                {
                   EndOBJ_found(inc);
                }

            }
            //This should be the beginning of the object
            else if(distanceReading[inc] < LastMag && (USE_BOUNDARY_LIMITS?isWithinArea(inc, distanceReading[inc],MaxMAG_LookUpTable) == true: true))
            {
                //printf("rightSide: %d\n", inc);
                //grabbing the angle of the starting edge of the object
                LidarOBJ[ObjectsFound].rightSideAngle = inc;
                //grabbing the magnitude of the starting edge of the object
                LidarOBJ[ObjectsFound].rightSideMAG = distanceReading[inc];
                //A rising edge of an object( a Fall in magnitude greater than OBJECT_IDENTIfIER_RANGE) was found
                ObjStartEdgeFound = true;
             
            }
            else
            {
                //this may be the actual start of the object
                if(ObjStartEdgeFound == true)
                {
                    if(distanceReading[inc] < LastMag && isWithinArea(inc, distanceReading[inc],MaxMAG_LookUpTable) == true)
                    {
                        //grabbing the angle of the starting edge of the object
                        LidarOBJ[ObjectsFound].rightSideAngle = inc;
                        //grabbing the magnitude of the starting edge of the object
                        LidarOBJ[ObjectsFound].rightSideMAG = distanceReading[inc];
                        //A rising edge of an object( a Fall in magnitude greater than OBJECT_IDENTIfIER_RANGE) was found
                        ObjStartEdgeFound = true;
                    }
                }
            }
        } 
        //setting the now old distance reading to LastMag for comparison in the next iteration of this loop
        LastMag = distanceReading[inc];
        //exit the for loop since we found the object
        if(isObj)
        {
            inc = 180;
            isObj = false;
        }
     
    }
    //this is if the OBJ is going out of range(past 180). So I won't see the end of the OBJ so I will need to force the end of the obj to be 180 degrees
    if(ObjStartEdgeFound == true && isWithinArea(179, distanceReading[179],MaxMAG_LookUpTable) == true && LidarOBJ[ObjectsFound].rightSideAngle > 90)
    {
        EndOBJ_found(180);
        LidarOBJ[ObjectsFound].objectAngle = (180 + LidarOBJ[ObjectsFound].rightSideAngle)/2;
    }
//    if(isObj)
//    {
//        clearOBJvariables();
//    }
        
    
}
void clearOBJvariables()
{
    struct object tmpLidarOBJ;

    LidarOBJ[ObjectsFound] = tmpLidarOBJ;
}
void EndOBJ_found(int inc)
{
    LidarOBJ[ObjectsFound].leftSideAngle = inc - 1;
//    Trans1 = LidarOBJ[ObjectsFound].rightSideAngle;
//    Trans2 = LidarOBJ[ObjectsFound].leftSideAngle;
    LidarOBJ[ObjectsFound].leftSideMAG = LastMag;
    ObjStartEdgeFound = false;
    //LED1 ^=1;
    //finding the average magnitude and allowing that to be the distance of the robot relative to the beacon
    LidarOBJ[ObjectsFound].Magnitude = ( LidarOBJ[ObjectsFound].leftSideMAG +  LidarOBJ[ObjectsFound].rightSideMAG) / 2;
    //Calculating the center point of the OBJ (Finding the average of the two points)
    //This statement will find the width of an abject and compare it to the constant MIN_OBJECT_WIDTH to see if the object we found is what we are looking for
   
    if(OBJ_DETECTION_USE_WIDTH)
    {
        if((Calculate_PointDistance(PolarToCartesian(LidarOBJ[ObjectsFound].leftSideAngle, LidarOBJ[ObjectsFound].leftSideMAG), PolarToCartesian(LidarOBJ[ObjectsFound].rightSideAngle, LidarOBJ[ObjectsFound].rightSideMAG))) > MIN_OBJECT_WIDTH)
        {
           
            set_OBJangle();
            isObj = true;
            //ObjectsFound++;
        }
    }
    else
    {
        LidarOBJ[ObjectsFound].objectAngle = (LidarOBJ[ObjectsFound].leftSideAngle  + LidarOBJ[ObjectsFound].rightSideAngle) / 2;
        //ObjectsFound++;
    }
    if(RUNNING_AVG_OBJ_DATA == true)
    {
         set_objAVG(ObjectsFound);
         LidarOBJ[ObjectsFound].objectAngle =  LidarOBJ[ObjectsFound].AVG_OBJangle;
         LidarOBJ[ObjectsFound].Magnitude =  LidarOBJ[ObjectsFound].AVG_OBJmag;
    }
}



void SlopeFunction(unsigned short rightAngle, unsigned short leftAngle)
{
	int validSlopeCount = 0;
	bool isFirstRefSlope = true;
    LidarOBJ[ObjectsFound].isSlopeALinear = true;
	bool SlopeJustPhasedOut = false;
	unsigned short inc = 0;
	unsigned short tmpTransAngle = 0;
	const unsigned short GRACE_PERIOD_VAL = 4;
    double tmpSlope;
	//Setting our initial slope to start the comparison processes
	double RunningSlope = getSlope(rightAngle, rightAngle +1,distanceReading[rightAngle],distanceReading[rightAngle+1]);
	
     //DRV_USART3_WriteByte(0x11);
    //DRV_USART3_WriteByte(tmpTransAngle);
    //We are going to loop through the entire width of the OBJ
	for(inc = rightAngle + 1; inc <= leftAngle; inc++)
	{
		//Getting the next slope 
		tmpSlope = getSlope(inc, inc+1,distanceReading[inc],distanceReading[inc+1]);
		//looking to see of the two slope are similar enough to be called the same slope
        if(isWithinTolerance(tmpSlope, RunningSlope, POINT_TO_POINT_SLOPE_TOLERANCE))
        {
			//looking to see of the RunningSlope is still the same slope that was originally captured
            //this indicated whether of not the we are looking at a NON-linear section of the OBJ
            //if(true)-> The slopes of to now have been in the clear denoting Linear section
            //if(false)-> We are now searching for the start of a section that is linear from some angle X to the end of the OBJ
			if(isFirstRefSlope== false)
			{
                LidarOBJ[ObjectsFound].isSlopeALinear = false;
                //IF we just had a violation in the slope set
				if(SlopeJustPhasedOut)
				{
                    //We just a slope correlation so we are going to reset the boolean that tells us if we just had bad phase in the slope trend
					SlopeJustPhasedOut = false;
                    //This may be the beginning of the linear side of the OBJ so we are going to save this location
					tmpTransAngle = inc;
				}		
			}
            //counting every time we have a slope correlation
			validSlopeCount++;
			//setting RunningSlope to the average between the two slopes
			RunningSlope += tmpSlope;
			RunningSlope /= 2;
		}
		else
		{
            

            bool GraceExceeded = true;
            int a;
            //looking at the next # angles to see if we violate the Slope trend 
            for(a = inc+1;a < (inc + (GRACE_PERIOD_VAL + 1)); a++)
            {
                double slope = getSlope(inc, inc+1,distanceReading[inc],distanceReading[inc+1]);
                //looking to see of the two slope are similar enough to be called the same slope
                if(isWithinTolerance(slope, RunningSlope, POINT_TO_POINT_SLOPE_TOLERANCE))
                {
                    GraceExceeded = false;
                }
            }
            //Giving a grace period in the detection of the end of a slope trend. So that in the case of magnitude that is not a valid representation
            //of the actual OBJ presents its self
			if(GraceExceeded)
			{
				if(isFirstRefSlope)
				{
					if(validSlopeCount >= GRACE_PERIOD_VAL)
					{
						tmpTransAngle = inc;
						//exiting  the loop
						inc = leftAngle+1;
					}
					else
					{
						isFirstRefSlope = false;
						//getting a new reference slope 
						RunningSlope = getSlope(inc, inc+1,distanceReading[inc],distanceReading[inc+1]);
					} 
                    validSlopeCount = 0;
                    SlopeJustPhasedOut = true;
				}
				else
				{
					isFirstRefSlope = false;
                    //getting a new reference slope 
                    RunningSlope = getSlope(inc, inc+1,distanceReading[inc],distanceReading[inc+1]);
				}
			}
		}
	}
    LED2 ^= 1;
    //if no OBJ corner was found most likely the the lidar can only see one side of the OBJ
    if(tmpTransAngle == leftAngle || tmpTransAngle == 0)
    {
        isSingleSideSeen = true;
        LidarOBJ[ObjectsFound].SlopeA = getSlope(rightAngle, leftAngle,distanceReading[rightAngle],distanceReading[leftAngle]);
        LidarOBJ[ObjectsFound].SlopeB = LidarOBJ[ObjectsFound].SlopeA;
        isSingleSideSeen = true;
    }
    else
    {
        isSingleSideSeen = false;
        LidarOBJ[ObjectsFound].SlopeA = getSlope(rightAngle, tmpTransAngle,distanceReading[rightAngle],distanceReading[tmpTransAngle]);
        LidarOBJ[ObjectsFound].SlopeB = getSlope(tmpTransAngle, leftAngle,distanceReading[tmpTransAngle],distanceReading[leftAngle]);   
    } //Setting the slopes of the side of the OBJ
    //Setting the slopes of the side of the OBJ
    LidarOBJ[ObjectsFound].Corner = tmpTransAngle;
    
    LidarOBJ[ObjectsFound].slopeAReflect = SlopeReflectance(LidarOBJ[ObjectsFound].rightSideAngle, tmpTransAngle);
    LidarOBJ[ObjectsFound].slopeBReflect = SlopeReflectance(tmpTransAngle, LidarOBJ[ObjectsFound].leftSideAngle );

    DRV_USART3_WriteByte(0x11);
    DRV_USART3_WriteByte(tmpTransAngle);
    //Setting the slopes of the side of the OBJ
     
}


void CartesianCenter(unsigned short leftAngle, unsigned short leftMag, unsigned short rightAngle, unsigned short rightMag)
{
    struct pointD pointA = PolarToCartesian(leftAngle, leftMag);
    struct pointD pointB = PolarToCartesian(rightAngle, rightMag);
    struct pointD pointC;
    pointC.y = (pointA.y + pointB.y) / 2;
    pointC.x = (pointA.x + pointB.x) / 2; 
    LidarOBJ[ObjectsFound].Magnitude = sqrt(pow(pointC.y,2) + pow(pointC.x,2));
    LidarOBJ[ObjectsFound].objectAngle = trig_ABS(atan2(pointC.y, pointC.x) * RAD_TO_DEGREE);
    if(isSingleSideSeen == true)
    {
       LidarOBJ[ObjectsFound].Magnitude += 100;
       LidarOBJ[ObjectsFound].objectAngle -= 1;
    }
}

//this function will take the new data and  average 
void set_objAVG(int _OBJIndex)
{
    unsigned long Mag_sum = 0;
    unsigned long Angle_sum = 0;
    int inc;
    //looping through all the stored values and adding them to the longs to be averaged
    for(inc = 0; inc < AVERAGE_ITEMS_COUNT; inc++)
    {
        Mag_sum += LidarOBJ[_OBJIndex].Stored_mags[inc];
        Angle_sum += LidarOBJ[_OBJIndex].Stored_angles[inc];
    }
    //adding the newest object angles and magnitude the summed values to be averaged
    Mag_sum += LidarOBJ[_OBJIndex].Magnitude;
    Angle_sum += LidarOBJ[_OBJIndex].objectAngle;
    //averaging the values then storing them in a variable that is in the object struct
    LidarOBJ[_OBJIndex].AVG_OBJmag = (Mag_sum / (AVERAGE_ITEMS_COUNT + 1));
    LidarOBJ[_OBJIndex].AVG_OBJangle = (Angle_sum / (AVERAGE_ITEMS_COUNT + 1));
    //shifting all the stored values down in order to make room for the new value that is to come in
    for(inc = AVERAGE_ITEMS_COUNT - 1; inc > 0;inc--)
    {
        LidarOBJ[_OBJIndex].Stored_mags[inc] = LidarOBJ[_OBJIndex].Stored_mags[inc -1];
        LidarOBJ[_OBJIndex].Stored_angles[inc] = LidarOBJ[_OBJIndex].Stored_angles[inc -1];
    }

    //adding the new values to the array that holds the old values of mag and angle for the object
    LidarOBJ[_OBJIndex].Stored_mags[0] = LidarOBJ[_OBJIndex].Magnitude;
    LidarOBJ[_OBJIndex].Stored_angles[0] = LidarOBJ[_OBJIndex].objectAngle;
    
}


unsigned short Get_objMag()
{
    //returns the current distance of the object
    return  LidarOBJ[ObjectsFound].Magnitude;
}
void set_OBJangle()
{
    LidarOBJ[ObjectsFound].objectAngle = (LidarOBJ[ObjectsFound].leftSideAngle  + LidarOBJ[ObjectsFound].rightSideAngle) / 2;
    SlopeFunction(LidarOBJ[ObjectsFound].rightSideAngle, LidarOBJ[ObjectsFound].leftSideAngle);
    if(USE_CARTESIAN_CENTER == true)
    {
        CartesianCenter(LidarOBJ[ObjectsFound].leftSideAngle, LidarOBJ[ObjectsFound].leftSideMAG,LidarOBJ[ObjectsFound].rightSideAngle,LidarOBJ[ObjectsFound].rightSideMAG);
    }
}
unsigned short Get_objAngle()
{
    //returns the current angle of the object
    return LidarOBJ[ObjectsFound].objectAngle;
}
//Captures and stores the objects position for a point A or B depending on the Value of "initialPlacmentCaptured"
void captureOBJ_placment(bool _transmit)
{
    if(initialPlacmentCaptured == false)      //if the initial position of the robot wasn't collected yet
    {
        //collect the angle of the object and store in the heading struct
        LidarOBJ[ObjectsFound].Heading.StartAngle = Get_objAngle();
        //storing the current magnitude in the same mannar as the previously
        LidarOBJ[ObjectsFound].Heading.StartMag = Get_objMag();
        initialPlacmentCaptured = true;   //Initial position data was collected
        
    }
    else
    {
        
        LidarOBJ[ObjectsFound].Heading.EndAngle = Get_objAngle();
        LidarOBJ[ObjectsFound].Heading.EndMag = Get_objMag();
        //the second placement of the object has been logged 
        initialPlacmentCaptured = false;
        Calculate_Heading();
        LED6 ^= 1;
        if(_transmit == true)
        {
            transmitOBJ_Data(true,true,true,true);     //Let the flood gates open 
        }
    }
}
void Calculate_Heading()
{
    //this will store the two captured locations of the robot in cartesian form
    struct pointD PointA = PolarToCartesian(LidarOBJ[ObjectsFound].Heading.StartAngle,LidarOBJ[ObjectsFound].Heading.StartMag);
    struct pointD PointB = PolarToCartesian(LidarOBJ[ObjectsFound].Heading.EndAngle,LidarOBJ[ObjectsFound].Heading.EndMag);
    
    LidarOBJ[ObjectsFound].Heading.AtoB_dist = Calculate_PointDistance(PointA, PointB);
    //Min_Heading_dist: this value is the minimum distance that is allowed when the
    //heading is calculated. This eliminates headings that are erradic seen when an object doesn't move
    //but a heading is calculated 
    if(LidarOBJ[ObjectsFound].Heading.AtoB_dist > Min_Heading_dist)
    {
        LidarOBJ[ObjectsFound].Heading.Old_Heading = LidarOBJ[ObjectsFound].Heading.Heading;

        LidarOBJ[ObjectsFound].Heading.Old_AtoB_dist =  LidarOBJ[ObjectsFound].Heading.AtoB_dist;

        LidarOBJ[ObjectsFound].Heading.Heading = trig_ABS(atan2(PointB.y - PointA.y, PointB.x - PointA.x) * RAD_TO_DEGREE);
    }else{
        LidarOBJ[ObjectsFound].Heading.Heading = 0xFFFF;
    }
    
   
}

unsigned short trig_ABS(double _val)
{
    //returning a value that is between 0-360
    //the trig math will calculate a negative value so this function converts it to a positive value
    if(_val >= 0)
    {
        return (unsigned short)_val;
    }else{
        return (unsigned short)(_val + 360);
    }
}
void useFastTransData(int *receiveArrayAddress)
{
    char Val = receiveArrayAddress[FASTTRANS_MODE_SELECT_INDEX];
    
    //In God we trust. All others must bring data.--> W. Edwards Deming
    ContinuousTranmission = false;
    TransmissionMode = Val;
    //unless I am capturing the initial placement of the object in the first communication between the object and the beacon
    //I will just send the objects magnitude and angle.
    if(firstPlacmentCaptured == true || Val == FASTTRANS_RX_CAPTURE_OBJ_HEADING)
    {
        
        switch(TransmissionMode)
        {
            case FASTTRANS_RX_NULL:
                //do nothing......
                
                break;
            case FASTTRANS_RX_CAPTURE_OBJ_HEADING:
                captureOBJ_placment(true);
               
                break;
            case FASTTRANS_RX_LEAPFROG_OBJ_CAPTURE:
                LEAPFROG_OBJ_CAPTURE();  
                break;
            case FASTTRANS_RX_CONTINUOUS_LEAPFROG_OBJ_CAPTURE: 
                //This state will transmit the the object heading, displacement, angle and magnitude, using the LeapFrog Method(For def of LeapFrog method see bottom of .h file)
                //(look in the app.c state machine for the implementation )
                ContinuousTranmission = true;                     
                break;                                           
            case FASTTRANS_RX_CONTINUOUS_OBJ_CAPTURE:  
                //This state will transmit the object's Angle and magnitude at intervals of 100ms (look in the app.c state machine for the implementation) 
                ContinuousTranmission = true;
                break;
            case FASTTRANS_RX_RETURN_OLD:
                ToSend(FASTTRANS_MODE_SENT_MYADDRESS,moduleAddress ,&OutGoing_DataTransBuff);
                ToSend(FASTTRANS_TX_OBJ_ANGLE, Get_objAngle(), &OutGoing_DataTransBuff);
                ToSend(FASTTRANS_TX_OBJ_HEADING, LidarOBJ[ObjectsFound].Heading.Old_Heading, &OutGoing_DataTransBuff);
                ToSend(FASTTRANS_TX_OBJ_DISPLACEMENT, LidarOBJ[ObjectsFound].Heading.Old_AtoB_dist, &OutGoing_DataTransBuff);
                ToSend(FASTTRANS_TX_OBJ_MAG,  Get_objMag(), &OutGoing_DataTransBuff);
                sendData(FastTransferAddressDestination, &OutGoing_DataTransBuff);
                break;
            default:
                //LED5 ^= 1;
                break;
        }
    }
    else
    {
        initialPlacmentCaptured = false;
        //The value passed is set to false to ensure that the heading data is not sent
        captureOBJ_placment(false);
        transmitOBJ_Data(true, true, true, true);
        firstPlacmentCaptured = true;
    } 
    //reseting the command value sent from the robot
    receiveArrayAddress[FASTTRANS_MODE_SELECT_INDEX] = 0;
  
    
}
void LEAPFROG_OBJ_CAPTURE()
{
    //setting it true so that the first operation is to calculate the heading of the object
    initialPlacmentCaptured = true;
    //calculating the heading from the previous point and transmitting it
    captureOBJ_placment(true);
    //setting the initial point of object
    captureOBJ_placment(false);  
}
void transmitOBJ_Data(bool _transOBJ_angle, bool _transOBJ_heading, bool _transOBJ_AtoB, bool _transOBJ_mag)
{
    //WATCH THE FUCK OUT _SETH CARPENTER
    SendOBJData_debug();
    return;
    
    
    ToSend(FASTTRANS_MODE_SENT_MYADDRESS,moduleAddress ,&OutGoing_DataTransBuff);
    //sending the Angle of the OBJ relative to the Beacon 
    if(_transOBJ_angle == true )
    {
        Trans1 = Get_objAngle();
        //ToSend(FASTTRANS_TX_OBJ_ANGLE, Get_objAngle(), &OutGoing_DataTransBuff);
        ToSend(FASTTRANS_TX_OBJ_ANGLE, Trans1, &OutGoing_DataTransBuff);
    }
    //sending the Angle the OBJ traveled between two captured points 
    if(_transOBJ_heading == true)
    {
        ToSend(FASTTRANS_TX_OBJ_HEADING, LidarOBJ[ObjectsFound].Heading.Heading, &OutGoing_DataTransBuff);
    }
    //sending the Distance traveled by the OBJ in between the two point captures
    if(_transOBJ_AtoB == true )
    {
        ToSend(FASTTRANS_TX_OBJ_DISPLACEMENT, LidarOBJ[ObjectsFound].Heading.AtoB_dist, &OutGoing_DataTransBuff);
    }
    if(_transOBJ_mag == true)
    {
        Trans2 = Get_objMag();
        //ToSend(FASTTRANS_TX_OBJ_MAG,  Get_objMag(), &OutGoing_DataTransBuff);
        ToSend(FASTTRANS_TX_OBJ_MAG,  Trans2, &OutGoing_DataTransBuff);
    }
    //Transmit data
    
    sendData(FastTransferAddressDestination, &OutGoing_DataTransBuff);
//    GLOBAL_50ms_TIMER_FLAG = false;
//    while(GLOBAL_50ms_TIMER_FLAG == false);
//    GLOBAL_50ms_TIMER_FLAG = false;
    //SendOBJData_debug();
}
void SendOBJData_debug()
{
      //DRV_USART3_WriteByte(0xFF);
     ToSend(FASTTRANS_TX_OBJ_HEADING, getSecondaryAngle(),&OutGoing_DataTransBuff);
     ToSend(FASTTRANS_TX_OBJ_DISPLACEMENT, getSecondaryDist(),&OutGoing_DataTransBuff);
     ToSend(FASTTRANS_TX_OBJ_ANGLE,Get_objAngle() , &OutGoing_DataTransBuff);
     ToSend(FASTTRANS_TX_OBJ_MAG,  Get_objMag(),&OutGoing_DataTransBuff);
     
//     ToSend(DEBUG_OBJ_RIGHT_SIDE_ANGLE,LidarOBJ[ObjectsFound].rightSideAngle ,&debugRingBuff);  
//     ToSend(DEBUG_OBJ_RIGHT_SIDE_MAG,LidarOBJ[ObjectsFound].rightSideMAG ,&debugRingBuff);
//     ToSend(DEBUG_OBJ_LEFT_SIDE_ANGLE,LidarOBJ[ObjectsFound].leftSideAngle ,&debugRingBuff);
//     ToSend(DEBUG_OBJ_LEFT_SIDE_MAG,LidarOBJ[ObjectsFound].leftSideMAG ,&debugRingBuff);
//     ToSend(DEBUG_OBJ_AVGmag,LidarOBJ[ObjectsFound].AVG_OBJmag ,&debugRingBuff);  
//     ToSend(DEBUG_OBJ_AVGangle,LidarOBJ[ObjectsFound].AVG_OBJangle ,&debugRingBuff);
//     ToSend(DEBUG_OBJ_HEADING_START_AGNLE,LidarOBJ[ObjectsFound].Heading.StartAngle ,&debugRingBuff);
//     ToSend(DEBUG_OBJ_HEADING_START_MAG,LidarOBJ[ObjectsFound].Heading.StartMag ,&debugRingBuff);
//     ToSend(DEBUG_OBJ_HEADING_END_AGNLE,LidarOBJ[ObjectsFound].Heading.EndAngle ,&debugRingBuff);
//     ToSend(DEBUG_OBJ_HEADING_END_MAG,LidarOBJ[ObjectsFound].Heading.EndMag ,&debugRingBuff);
//     ToSend(DEBUG_OBJ_SLOPE,LidarOBJ[ObjectsFound].ReflectiveSlope ,&debugRingBuff);
     sendData(FastTransferAddressDestination, &OutGoing_DataTransBuff);
}

//At the very beginning or startup of the board during initializations, this function will be run to calculated the look up table for the virtual box that the Lidar will not see outside of.
//To change the size of the box Edit the #defines in the LidarDataInterpreter.h file on lines 16-17-18 (LEFTSIDE_DISTANCE, RIGHTSIDE_DISTANCE, AREA_LENGTH)
void Generate_LookUpTable()
{
//    int _angle = 0;
//    for(_angle = 0; _angle < 180; _angle++)
//    {
//        if(_angle < 90)
//        {
//            //Basic Trig for angles less then 90 degrees
//            MaxMAG_LookUpTable[_angle] = (RightSideDistance / cos((_angle*DEGREE_TO_RAD)));
//        }
//        else
//        {
//            //calculating the max magnitude the is allowed inside the virtual box from angles 91 -180)
//            MaxMAG_LookUpTable[_angle] = (LeftSideDistance / -cos((_angle*DEGREE_TO_RAD)));
//        }
        //Setting a max range
//if( MaxMAG_LookUpTable[_angle] > AREA_LENGTH)

        int _angle = 0;
        int quad = 0;
        MaxMAG_LookUpTable[90] = AREA_LENGTH;
        for(quad = 0; quad < 2; quad++)

       // {
          for(_angle = 0; _angle < 90; _angle++)
          {
  
            if(quad == 0)
            {
              if(_angle <= (int)(atan(AREA_LENGTH/RIGHTSIDE_DISTANCE) * RAD_TO_DEGREE))
              {
                 MaxMAG_LookUpTable[_angle] = (RIGHTSIDE_DISTANCE / cos((_angle*DEGREE_TO_RAD)));
              }
              else
              {
                 MaxMAG_LookUpTable[_angle] = (AREA_LENGTH / cos(((90-_angle)*DEGREE_TO_RAD)));
              }    
            }
            else
            {
                if(MAKE_SAME == true)
                {
                   MaxMAG_LookUpTable[180 - _angle] = MaxMAG_LookUpTable[_angle];
                }
                else
                {
                    if((_angle) <= (int)(atan(AREA_LENGTH/LEFTSIDE_DISTANCE) * RAD_TO_DEGREE))
                    {
                       MaxMAG_LookUpTable[180 - _angle] = (LEFTSIDE_DISTANCE / cos(((_angle)*DEGREE_TO_RAD)));
                    }
                    else
                    {
                       MaxMAG_LookUpTable[180 - _angle] = (AREA_LENGTH / cos(((90-(_angle))*DEGREE_TO_RAD)));
                    }
                }           
            }
        }        
    }
/*
void SlopeOBJ_IdentifierMethod(unsigned short leftAngle, unsigned short rightAngle)
{
    double RunningSlope = 0;
    double SlopeA = 0,SlopeB = 0;
    //to figure out whether or not I am looking at the bucket or a non geometrically flat surface this is what I need to do:
    //Start by looping through the object bound to to find a continuing slope. if we continue to have sectors of calculated slopes that are not correlating with each other then we will 
    //continue until we find a sector of "good" slopes. We will then need to set some sort of flag indicating that we are looking at a side of the object that is not Geometrically smooth.
    int i;
    for(i = 0; i < NUM_OF_INITIAL_SLOPES; i++)
    {
        RunningSlope += getSlope(rightAngle + i, rightAngle + (i+1),distanceReading[rightAngle + i],distanceReading[rightAngle+(i+1)]);
    }
    rightAngle += NUM_OF_INITIAL_SLOPES;
    RunningSlope /= NUM_OF_INITIAL_SLOPES;
    bool SingleSlope = true;
    unsigned short transitionIndex;
    unsigned short badPeriod = 0;
    unsigned short a;
    unsigned short outOfTolerance = 0;
    
    DRV_USART3_WriteByte(0x77);
     DRV_USART3_WriteByte(leftAngle);
     DRV_USART3_WriteByte(rightAngle);
    for(a = (rightAngle + 1) ;a<(leftAngle +1);a++)
    {
       
        //we need to continue calculating the slopes until we start find a continuous set of slopes that would indicate we found the corner of the object
        //then we will calculate the slope of each side by adding all the slopes together and finding the average of them with consideration of some sort of grace period for strange magnitudes
        double tmpSlope = getSlope(a, a + 1, distanceReading[a],distanceReading[a+1]);
        
        if(isWithinTolerance(tmpSlope, RunningSlope, POINT_TO_POINT_SLOPE_TOLERANCE))
        {
            
            outOfTolerance = 0;
            //if we think we have detected another slope(sideB of the OBJ)
            if(SingleSlope == true)
            {
                RunningSlope += tmpSlope;
                RunningSlope /= 2;
                if(isWithinTolerance(RunningSlope, NINETY_DEG_RETURN_VAL, 1))
                {
                   LidarOBJ[ObjectsFound].isSquared = true; 
                   LED0 = ON;
                }
                else 
                {
                    LidarOBJ[ObjectsFound].isSquared = false;
                    LED0 = OFF;
                }
                
            }
            
        }
        else
        { 
            if(outOfTolerance == 0)
            {
                 transitionIndex = a;
            }
            outOfTolerance++;
            if(outOfTolerance > SLOPE_DEVIATION_GRACE_PERIOD)
            {
                SingleSlope = false;
                a = leftAngle +1;
            }
        }
    }
    SlopeA = RunningSlope;
    SlopeB = getSlope(transitionIndex, leftAngle, distanceReading[transitionIndex],distanceReading[leftAngle]);
}
 */

