#ifndef LOOP_TIME_
#define LOOP_TIME_

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"

// ---------------------------------------
// Loop_time struct
// ---------------------------------------
typedef struct 
{
    uint32_t sysSpeed; // should be 220000000 but can be changed to up around 250MHZ around that value it starts to tweak and shows around 1500MHZ? 
    uint32_t currentTime; // current time for correct loop times
    uint16_t loopSpeed; // speed of outer loop should be 4000 (250Hz) 
    float    dt; // set to 0.004 for other functions  
}loop_time;

// ---------------------------------------
// Public API
// ---------------------------------------
void LOOP_TIME_setSystemClockSpeed(loop_time* loopTime);
void LOOP_TIME_printSystemClockSpeed(loop_time* loopTime); 
void LOOP_TIME_startLoop(loop_time* loopTime); 
void LOOP_TIME_endLoop(loop_time* loopTime);  
void LOOP_TIME_checkLoop(loop_time* loopTime);

// ---------------------------------------
// Internal functions
// ---------------------------------------

#endif // LOOP_TIME_

