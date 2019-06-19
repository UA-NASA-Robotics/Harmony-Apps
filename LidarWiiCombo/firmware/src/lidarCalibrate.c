
#include "lidarCalibrate.h"

void _mon_putc(char ch);

    timer_t delay100ms;
void sweepConfigSettings(void)
{
    setTimerInterval(&delay100ms,100);
      //Initial values                                  ORIGINAL VALUES
           unsigned long startingValA     =5547000;  //5547000;
           unsigned long startingValB     =6853;     //6853;
           unsigned long startingValC     =0;        //0;
           unsigned long startingValLPT   =24000;    //24000;
           unsigned long startingValLFL   =500;      //230;
           unsigned long startingValLFT   =750;      //272;
           unsigned long startingValLFH   =1000;      //286;
           unsigned long startingValIMX   =10000;        //0;
           unsigned long startingValIB    =0;        //0;
           unsigned long startingValLPI   =4096;     //4096;
           unsigned long startingValLCH   =500;      //434;
           unsigned long startingValLPD   =16384;    //16384        
           //int startingValANG   =140;           //800;
           
            static unsigned long configASetting      =0;       //
            static unsigned long configbSetting      =0;       //Distance calibration
            static unsigned long configCSetting      =0;       //Distance calibration
            static unsigned long configLPTSetting    =0;       //
            static unsigned long configLFLSetting    =0;       //
            static unsigned long configLFTSetting    =0;       //BIGG DIFFERENCE WHEN LFH IS HIGH (LFT-1200-3000 and LFH 10000)
            static unsigned long configLFHSetting    =0;       //NO reasonsable difference when changed while LFT and LFL at stock
            static unsigned long configIMXSetting    =0;       //
            static unsigned long configIBSetting     =0;       //--May have an effect ('jumpy data' possibly more range) (best at 4k)
            static unsigned long configLPISetting    =0;       //--May have an effect 
            static unsigned long configLCHSetting    =0;       //--May have an effect R[~400-?]
            static unsigned long configLPDSetting    =0;       //??
            
            if(configASetting==0)       configASetting      =startingValA; 
            if(configbSetting==0)       configbSetting      =startingValB;
            if(configCSetting==0)       configCSetting      =startingValC;
            if(configLPTSetting==0)     configLPTSetting    =startingValLPT;
            if(configLFLSetting==0)     configLFLSetting    =startingValLFL; 
            if(configLFTSetting==0)     configLFTSetting    =startingValLFT;
            if(configLFHSetting==0)     configLFHSetting    =startingValLFH;
            if(configIMXSetting==0)     configIMXSetting    =startingValIMX;
            if(configIBSetting==0)      configIBSetting     =startingValIB;
            if(configLPISetting==0)     configLPISetting    =startingValLPI;
            if(configLCHSetting==0)     configLCHSetting    =startingValLCH;
            if(configLPDSetting==0)     configLPDSetting    =startingValLPD;
            
            //-------------------Clear the buffer-------------------
            while(Receive_available_3())
            {
                Receive_get_3();
            }
           
            setCal(configASetting,
                    configbSetting,
                    configCSetting,
                    configLPTSetting,
                    configLFLSetting,
                    configLFTSetting,
                    configLFHSetting,
                    configIMXSetting,
                    configIBSetting,
                    configLPISetting,
                    configLCHSetting,
                    configLPDSetting);
        
            
//           if(configASetting < 9500000)     configASetting  +=200000;
//           else                             configASetting  = startingValA;
//           if(configbSetting < 15000)       configbSetting  +=250;
//           else                             configbSetting  = startingValB;
//            if(configCSetting < 10000)        configCSetting  +=1000;
//            else                            configCSetting  = startingValC;
//           if(configLPTSetting < 55000)     configLPTSetting+= 1000;
//           else                             configLPTSetting = startingValLPT;
           if(configLFLSetting <= configLFTSetting/2)       configLFLSetting+= 50;          
           else                                           configLFLSetting = startingValLFL;
//            if(configLFTSetting < 3000)       configLFTSetting+= 250;          
//            else                             configLFTSetting = startingValLFT;     //40
//           if(configLFHSetting < 10000)       configLFHSetting+= 1000;
//           else                             configLFHSetting = startingValLFH;     //10
//           if(configIMXSetting < 100000)         configIMXSetting+= 10000;
//           else                             configIMXSetting = startingValIMX;  
//           if(configIBSetting < 10000)          configIBSetting+= 1000;
//           else                             configIBSetting = startingValIB; 
//           if(configLPISetting < 100000)      configLPISetting+= 10000;
//           else                             configLPISetting = startingValLPI; 
//           if(configLCHSetting < 700)       configLCHSetting+= 25;
//           else                             configLCHSetting = startingValLCH;  
//           if(configLPDSetting < 24000)     configLPDSetting+= 1000;
//           else                             configLPDSetting = startingValLPD;
           //---------------------Wait a bit-------------------------
           resetTimer(&delay100ms);
           while(!timerDone(&delay100ms));
            
           //--------------Print the response-------------------
           
//            while(Receive_available_3())
//            {              
//                printf("%c",Receive_get_3());                
//            }
      
            //---------------------Clear the angles-------------------------
            //clearLidarData();
           
}

timer_t configDelay;
void setCal(unsigned long a, unsigned long b, unsigned long c, 
                unsigned long lpt, unsigned long lfl, unsigned long lft, 
                unsigned long lfh, unsigned long imx, unsigned long ib, 
                unsigned long lpi, unsigned long lch, unsigned long lpd)
{
            setTimerInterval(&configDelay,100);
     //--------------------Send the config--------------------
            char printable[50];
            
//            getprintcal();
//            while(1);
            
            LED8 = ON;
            
            sprintf(printable,"setcal a %d\r\n",a);
            myprintf(printable); 
            resetTimer(&configDelay);
            while(!timerDone(&configDelay));
            ToSend(5,a/1000,&debugRingBuff);
            
            sprintf(printable,"setcal b %d\r\n",b);
            myprintf(printable); 
            resetTimer(&configDelay);
            while(!timerDone(&configDelay));
            ToSend(6,b,&debugRingBuff);
            
            sprintf(printable,"setcal c %d\r\n",c);
            myprintf(printable); 
            resetTimer(&configDelay);
            while(!timerDone(&configDelay));
            ToSend(16,c,&debugRingBuff);
            
            sprintf(printable,"setcal LPT %d\r\n",lpt);
            myprintf(printable); 
            resetTimer(&configDelay);
            while(!timerDone(&configDelay));
            ToSend(7,lpt,&debugRingBuff);
            
            sprintf(printable,"setcal LFL %d\r\n",lfl);
            myprintf(printable); 
            resetTimer(&configDelay);
            while(!timerDone(&configDelay));
            ToSend(8,lfl,&debugRingBuff);
            
            sprintf(printable,"setcal LFT %d\r\n",lft);
            myprintf(printable); 
            resetTimer(&configDelay);
            while(!timerDone(&configDelay));
            ToSend(9,lft,&debugRingBuff);
            
            sprintf(printable,"setcal LFH %d\r\n",lfh);
            myprintf(printable); 
            resetTimer(&configDelay);
            while(!timerDone(&configDelay));
            ToSend(10,lfh,&debugRingBuff);
            
            sprintf(printable,"setcal IMX %d\r\n",imx);
            myprintf(printable); 
            resetTimer(&configDelay);
            while(!timerDone(&configDelay));
            ToSend(11,imx,&debugRingBuff);
            
            sprintf(printable,"setcal ib %d\r\n",ib);
            myprintf(printable); 
            resetTimer(&configDelay);
            while(!timerDone(&configDelay));
            ToSend(12,ib,&debugRingBuff);
            
            sprintf(printable,"setcal lpi %d\r\n",lpi);
            myprintf(printable); 
            resetTimer(&configDelay);
            while(!timerDone(&configDelay));
            ToSend(13,lpi,&debugRingBuff);
            
            sprintf(printable,"setcal LCH %d\r\n",lch);
            myprintf(printable); 
            resetTimer(&configDelay);
            while(!timerDone(&configDelay));
            ToSend(14,lch,&debugRingBuff);
            
            sprintf(printable,"setcal LPD %d\r\n",lpd);
            myprintf(printable); 
            resetTimer(&configDelay);
            while(!timerDone(&configDelay));
            ToSend(15,lpd,&debugRingBuff);
            sendData(4,&debugRingBuff);
            
            
}


void getprintcal(void)
{
    char printable[50];
    timer_t ms100;
    setTimerInterval(&ms100,100);
    
    sprintf(printable,"getcal a\r\n");
    myprintf(printable); 
    resetTimer(&ms100);
    while(!timerDone(&ms100));
    while(Receive_available_3())
    {              
        printf("%c",Receive_get_3());                
    }
    
    sprintf(printable,"getcal b\r\n");
    myprintf(printable); 
    resetTimer(&ms100);
    while(!timerDone(&ms100));
    while(Receive_available_3())
    {              
        printf("%c",Receive_get_3());                
    }
    
    sprintf(printable,"getcal c\r\n");
    myprintf(printable); 
    resetTimer(&ms100);
    while(!timerDone(&ms100));
    while(Receive_available_3())
    {              
        printf("%c",Receive_get_3());                
    }
    
    
    sprintf(printable,"getcal lpt\r\n");
    myprintf(printable); 
    resetTimer(&ms100);
    while(!timerDone(&ms100));
    while(Receive_available_3())
    {              
        printf("%c",Receive_get_3());                
    }
    
    sprintf(printable,"getcal lfl\r\n");
    myprintf(printable); 
    resetTimer(&ms100);
    while(!timerDone(&ms100));
    while(Receive_available_3())
    {              
        printf("%c",Receive_get_3());                
    }
    
    sprintf(printable,"getcal lft\r\n");
    myprintf(printable); 
    resetTimer(&ms100);
    while(!timerDone(&ms100));
    while(Receive_available_3())
    {              
        printf("%c",Receive_get_3());                
    }
    
    sprintf(printable,"getcal lfh\r\n");
    myprintf(printable); 
    resetTimer(&ms100);
    while(!timerDone(&ms100));
    while(Receive_available_3())
    {              
        printf("%c",Receive_get_3());                
    }
    
    sprintf(printable,"getcal imx\r\n");
    myprintf(printable); 
    resetTimer(&ms100);
    while(!timerDone(&ms100));
    while(Receive_available_3())
    {              
        printf("%c",Receive_get_3());                
    }
    
    sprintf(printable,"getcal ib\r\n");
    myprintf(printable); 
    resetTimer(&ms100);
    while(!timerDone(&ms100));
    while(Receive_available_3())
    {              
        printf("%c",Receive_get_3());                
    }
    
    sprintf(printable,"getcal lpi\r\n");
    myprintf(printable); 
    resetTimer(&ms100);
    while(!timerDone(&ms100));
    while(Receive_available_3())
    {              
        printf("%c",Receive_get_3());                
    }
    
    sprintf(printable,"getcal lch\r\n");
    myprintf(printable); 
    resetTimer(&ms100);
    while(!timerDone(&ms100));
    while(Receive_available_3())
    {              
        printf("%c",Receive_get_3());                
    }
    
    sprintf(printable,"getcal lpd\r\n");
    myprintf(printable); 
    resetTimer(&ms100);
    while(!timerDone(&ms100));
    while(Receive_available_3())
    {              
        printf("%c",Receive_get_3());                
    }
    
    sprintf(printable,"getcal ang\r\n");
    myprintf(printable); 
    resetTimer(&ms100);
    while(!timerDone(&ms100));
    while(Receive_available_3())
    {              
        printf("%c",Receive_get_3());                
    }
}