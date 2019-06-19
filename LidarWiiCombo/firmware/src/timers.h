
#ifndef _TIMERS_H    /* Guard against multiple inclusion */
#define _TIMERS_H


#include <stdint.h>
#include "system_definitions.h"
#include "app.h"
    
    typedef struct{
        unsigned long timerInterval;
        unsigned long lastMillis;
    }timer_t;
    
bool timerDone(timer_t * t);
void setTimerInterval(timer_t * t, unsigned long interval);
void resetTimer(timer_t * t);
void globalTimerTracker( );
unsigned long millis(void);


    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif 