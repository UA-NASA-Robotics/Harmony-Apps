#include "timers.h"

unsigned long globalTime;

unsigned long millis(void)
{
    return globalTime;
}

bool timerDone(timer_t * t)
{
    if(abs(millis()-t->lastMillis)>t->timerInterval)
    {
        t->lastMillis=millis();
        return true;
    }
    else
    {
        return false;
    }
}

void setTimerInterval(timer_t * t, unsigned long interval)
{
    t->timerInterval= interval;
}

void resetTimer(timer_t * t)
{
    t->lastMillis=millis();
}

void globalTimerTracker( )
{
    globalTime++;
}