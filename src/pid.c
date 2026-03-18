#include "pid.h"

// ---------------------------------------
// Public API
// ---------------------------------------
void PID_Init(pid* PID)
{
    PID->prevError = 0; 
    PID->i = 0;
    PID->output = 0;
}

void PID_Calculate(pid* PID, float InputValue, float ImuMeasurement)
{
    PID->currError = InputValue - ImuMeasurement;

    PID->p  = PID->Kp * PID->currError ;

    PID->i += PID->Ki * PID->dt * PID->currError;

    // antywindup
    if(PID->i > PID->antyWindupMax) PID->i = PID->antyWindupMax;
    if(PID->i < PID->antyWindupMin) PID->i = PID->antyWindupMin;

    PID->d = PID->Kd * ((PID->currError - PID->prevError) / PID->dt);

    PID->output = PID->p + PID->i + PID->d;

    if(PID->output > PID->limit) PID->output = PID->limit;
    if(PID->output < -(PID->limit)) PID->output = -(PID->limit);
    PID->prevError = PID->currError;
}

void PID_Reset(pid* PID)
{
    PID->currError = 0; 
    PID->prevError = 0;
 

    PID->p = 0;  
    PID->i = 0; 
    PID->d = 0; 

    PID->output = 0;
}

// ---------------------------------------
// Internal functions
// ---------------------------------------