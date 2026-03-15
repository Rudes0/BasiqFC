#ifndef PID_
#define PID_

#include <stdio.h>
#include "pico/stdlib.h"

// ---------------------------------------
// PID struct
// ---------------------------------------
typedef struct 
{
    float currError; // current error
    float prevError; // previous error

    float Kp; 
    float Ki; 
    float Kd; 

    float p; // P output 
    float i; // I output and also a memory for I term 
    float d; // D output 

    float antyWindupMax; // max i value
    float antyWindupMin; // min i value 
    float dt; // sampling time 

    float limit; //limit output

    float output;
}pid;

// ---------------------------------------
// Public API
// ---------------------------------------
void PID_Init(pid* PID);
void PID_Calculate(pid* PID, float InputValue, float ImuMeasurement);
void PID_Reset(pid* PID);

#endif // PID_