#include "mma.h"

// ---------------------------------------
// Public API
// ---------------------------------------
void MMA_CalculateOutput(mma* MMA, float roll, float pitch, float throttle, float yaw)
{
    MMA->motorRF = throttle - roll - pitch - yaw;
    MMA->motorRB = throttle - roll + pitch + yaw;
    MMA->motorLB = throttle + roll + pitch - yaw;
    MMA->motorLF = throttle + roll - pitch + yaw;


}

void MMA_LimitOutput(mma* MMA)
{
    // not allowing for motors to get to much power 
    if(MMA->motorRF > 1950) MMA->motorRF = 1950; 
    if(MMA->motorRB > 1950) MMA->motorRB = 1950; 
    if(MMA->motorLB > 1950) MMA->motorLB = 1950; 
    if(MMA->motorLF > 1950) MMA->motorLF = 1950;
    
    // not alowing for motors to go down 
    if(MMA->motorRF < 1100) MMA->motorRF = 1100; 
    if(MMA->motorRB < 1100) MMA->motorRB = 1100; 
    if(MMA->motorLB < 1100) MMA->motorLB = 1100; 
    if(MMA->motorLF < 1100) MMA->motorLF = 1100; 
}

// ---------------------------------------
// Internal functions
// ---------------------------------------
