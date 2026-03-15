#include "input_control.h"

void INPUT_CONTROL_CalculateInput(input_control* INPUT, crsf_data* CRSF) // values for individual PID controllers
{
    
    INPUT->roll =  0.10 * (CRSF->pwmData[0] - 1500); // if in rate mode need to be * 0.15 if angle mode * 0.10 // TO DO: change automaticaly when using different modes 
    INPUT->pitch =  0.10 * (CRSF->pwmData[1] - 1500);
    INPUT->throttle = CRSF->pwmData[2]; // to be changed to 0.3 * (CRSF->PWMData[2] - 1500) for altitude controll but couldnt find a suitable way for reading vertical velocity 
    INPUT->yaw =  0.15 * (CRSF->pwmData[3] - 1500); 
}

uint8_t INPUT_CONTROL_IsArmed(crsf_data* CRSF)
{
    return (CRSF->pwmData[4] == 2000) ? 1 : 0;
}

void INPUT_CONTROL_LimitThrottle(input_control* INPUT) // limit throttle so other PID outputs can control the balance
{
    if(INPUT->throttle > 1800) INPUT->throttle = 1799;
    if(INPUT->throttle < 1100) INPUT->throttle = 1101;
}