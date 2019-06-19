/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/



// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include <stdbool.h>
#include <stdio.h>
#include "system_definitions.h"
#include "app.h"
#include "uart_handler.h"
#include "FastTransfer.h"
#include "FastTransfer1.h"
#include "MultiLIDAR.h"
#include "LidarDecoder.h"
#include "LidarDataInterpreter.h"
#include "timers.h"
#include "lidarCalibrate.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/
 static timer_t configDelay,ms100,secondTimer,spindown,threesecond,foursecond;
 timer_t lidar_runtime;

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}

#define MASTER
/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

           
void APP_Tasks ( void )
{

    /* Check the application's current state. */
   switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            setTimerInterval(&ms100,100);
            setTimerInterval(&secondTimer,1000);
            setTimerInterval(&spindown,3000);
            setTimerInterval(&threesecond,3000);
            setTimerInterval(&foursecond,4000);
            setTimerInterval(&lidar_runtime,7000);
            setupLidarTimers();
            
            
            bool appInitialized = true;
            currentOC_val = 1800;
            FastTransTestCounter = 0;
            if(COMPUT_AUTO_DEMENTIONS == false)
            {
                Generate_LookUpTable();
            }
            
            else
            {
                long sumR = 0;
                long sumL = 0;
                int inc;
                for(inc = 0; inc < 30;inc++)
                {
                    decode_LidarData();
                    sumL+=distanceReading[180];
                    sumR+=distanceReading[0];
                }
                sumR = sumR / 30;
                sumL = sumL / 30;
                LeftSideDistance = sumL;
                RightSideDistance = sumR;
                Generate_LookUpTable();                 
            }
            
            
#ifndef MASTER
                LED5 = ON;
                isMaster = false;
                initFTLIDARPair(isMaster); 
#else             

                LED5 = OFF;
                isMaster = true;
                initFTLIDARPair(isMaster);
#endif
             
             //Currently remapped to the CAN header
             begin(receiveArray, sizeof(receiveArray),FastTransferAddress,false, Send_put_2,Receive_get_2,Receive_available_2,Receive_peek_2);
           
           
            if (appInitialized)
            {
                DRV_OCO_Change_PulseWidth(1800);
            
                appData.state = APP_STATE_STOP_LIDAR;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            appData.state = APP_STATE_LIDAR;
            break;
        }
        
       case APP_STATE_STOP_LIDAR:
       {
           DRV_OCO_Change_PulseWidth(0);
           
           //---------------------Wait a bit-------------------------
           resetTimer(&spindown);
           while(!timerDone(&spindown));
           
           appData.state++;
           break;
       }
       case APP_STATE_CONFIGURATION_SETUP:
       {
           
           sweepConfigSettings();
//           printf("Writing Calibration\r\n");
//            while(Receive_available_3())
//            {              
//                printf("%c",Receive_get_3());                
//            }
           //--------------Turn on motor--------------
            DRV_OCO_Change_PulseWidth(1800);
            
            //-------Wait for spin up--------
           resetTimer(&ms100);
           while(!timerDone(&ms100));
           
           appData.state=APP_STATE_LIDAR;
           break;
       }
        
        
        case APP_STATE_LIDAR:                              
        {            
            resetTimer(&lidar_runtime);
            while( appData.state == APP_STATE_LIDAR)
            {
                if(timerDone(&secondTimer))
                {
                    LED6 ^=1;
                }
               
                receiveData();
                
                //DRV_USART2_WriteByte(0x00);
                decode_LidarData();
                startObjectDetection();                
                
                //for continuous transmission of the objects data
                if(timerDone(&ms100))
                {
//                    #ifdef SEND_DATA_TO_DEBUGGER
//                        SendOBJData_debug();
//                    #else       
                        if(ContinuousTranmission == true)
                        {
                            switch(TransmissionMode)
                            {
                                case FASTTRANS_RX_CONTINUOUS_LEAPFROG_OBJ_CAPTURE:
                                    LEAPFROG_OBJ_CAPTURE();
                                    break;
                                case FASTTRANS_RX_CONTINUOUS_OBJ_CAPTURE:
                                    //Sends the object's angle and magnitude
                                    transmitOBJ_Data(true, false, false, true);
                                    break;
                            }
                        }
//                    #endif
                }
//                if(timerDone(&lidar_runtime))
//                {
//                    appData.state=APP_STATE_STOP_LIDAR;
//                }
            }
           
            break;
        }
       case FASTTRANS_TEST:
       {
            if(timerDone(&ms100))
            {
                ToSend(0,0xFFFF,  &OutGoing_DataTransBuff);
                sendData(FastTransTestCounter, &OutGoing_DataTransBuff);
                
                
                if(FastTransTestCounter > 10)
                {
                    FastTransTestCounter = 0;
                }
            }
           break;
       }
        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
