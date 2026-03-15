#ifndef INPUT_CONTROL_
#define INPUT_CONTROL_

#include "pico/stdlib.h"
#include "crsf.h"

// ---------------------------------------
// Input_control struct
// ---------------------------------------
typedef struct 
{
    // channel 0 roll
    // channel 1 pitch
    // channel 2 throttle
    // channel 3 yaw
    // all values are mapped from 1000 to 2000
    float roll; // range from -50° to 50°
    float pitch; // range from -50° to 50°
    float throttle; // simply added to motors   
    float yaw; // range from -30°/s to 30°/s
}input_control;

// ---------------------------------------
// Public API
// ---------------------------------------
void INPUT_CONTROL_CalculateInput(input_control* INPUT, crsf_data* CRSF);
uint8_t INPUT_CONTROL_IsArmed(crsf_data* CRSF);
void INPUT_CONTROL_LimitThrottle(input_control* INPUT);

#endif // INPUT_CONTROL_