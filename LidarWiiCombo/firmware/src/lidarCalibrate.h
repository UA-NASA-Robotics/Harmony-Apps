
#ifndef _LIDAR_CALIBRATE_H    /* Guard against multiple inclusion */
#define _LIDAR_CALIBRATE_H

#include <stdio.h>
#include "app.h"
#include "timers.h"
#include "FastTransfer.h"
#include "uart_handler.h"

void sweepConfigSettings(void);
void setCal(unsigned long a, unsigned long b, unsigned long c, 
        unsigned long lpt, unsigned long lfl, unsigned long lft, 
        unsigned long lfh, unsigned long imx, unsigned long ib, 
        unsigned long lpi, unsigned long lch, unsigned long lpd);
void getprintcal(void);


#endif