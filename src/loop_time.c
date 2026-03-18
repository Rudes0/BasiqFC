#include "loop_time.h"

// ---------------------------------------
// Public API
// ---------------------------------------
void LOOP_TIME_setSystemClockSpeed(loop_time* loopTime) // sets systems clock speed
{
    set_sys_clock_hz(loopTime->sysSpeed, true);
}

void LOOP_TIME_printSystemClockSpeed(loop_time* loopTime) // prints wanted speed and actuall clock speed
{
    printf("Systems clock speed should be %d\n", loopTime->sysSpeed);
    printf("Systems clock speed is set to %d\n", clock_get_hz(clk_sys));
}

void LOOP_TIME_startLoop(loop_time* loopTime) // used at the beginning of the loop starts loop counting 
{
    loopTime->currentTime = time_us_32();
}

void LOOP_TIME_endLoop(loop_time* loopTime) // used at the end of the loop so that the loop is not shorter than loopSpeed
{
    while(time_us_32() - loopTime->currentTime < loopTime->loopSpeed);
}

void LOOP_TIME_checkLoop(loop_time* loopTime) // checks current time it took to reach this place in code
{
    printf("Current loops time = %d\n", time_us_32() - loopTime->currentTime);
}

// ---------------------------------------
// Internal functions
// ---------------------------------------