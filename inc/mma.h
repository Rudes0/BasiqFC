#ifndef MMA_
#define MMA_

#include "pico/stdlib.h"
#include "pid.h"

// ---------------------------------------
// motor mixing algorythm struct
// ---------------------------------------
typedef struct 
{
    float motorRF; // motor right front (throttle - roll - pitch - yaw)
    float motorRB; // motor right back (throttle - roll + pitch + yaw)
    float motorLB; // motor left back (throttle + roll + pitch - yaw)
    float motorLF; // motor left front (throttle + roll - pitch + yaw)

}mma;

// ---------------------------------------
// Public API
// ---------------------------------------
void MMA_CalculateOutput(mma* MMA, float roll, float pitch, float throttle, float yaw);
void MMA_LimitOutput(mma* MMA);
#endif // MMA_