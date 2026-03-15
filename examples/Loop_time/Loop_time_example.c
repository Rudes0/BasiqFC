#include "loop_time.h"
int main()
{
    loop_time LoopTime =
    {
        .sysSpeed = 220000000,
        .loopSpeed = 4000,
        .dt = 0.004f
    };
    LOOP_TIME_setSystemClockSpeed(&LoopTime);
    stdio_init_all();
    
    while(1)
    {
        LOOP_TIME_startLoop(&LoopTime);
        LOOP_TIME_printSystemClockSpeed(&LoopTime);
        LOOP_TIME_checkLoop(&LoopTime);
        LOOP_TIME_endLoop(&LoopTime);
        sleep_ms(100); // should't be used here but it is here so that the serial is not bloated with ton of prints
    }
}