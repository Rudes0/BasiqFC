#ifndef ONESHOT_CONTROL_
#define ONESHOT_CONTROL_

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "mma.h"

// ---------------------------------------
// Oneshot125 struct
// ---------------------------------------
typedef struct 
{
    // gpio values for initialization
    uint8_t  motorLF; // left front 
    uint8_t  motorLB; // left back
    uint8_t  motorRF; // right front 
    uint8_t  motorRB; // right back
    // GPIO slices for PWM 
    uint8_t  sliceNumLF;
    uint8_t  sliceNumLB;
    uint8_t  sliceNumRF; 
    uint8_t  sliceNumRB;
    // Oneshot values from 125us - 240us (Note that full range is 250 but we cut last 10 to let the ESC breathe)
    uint16_t fillLF;
    uint16_t fillLB;
    uint16_t fillRF;
    uint16_t fillRB;

}oneshot;

// ---------------------------------------
// Public API
// ---------------------------------------
void ONESHOT_InitMotors(oneshot* Oneshot);
void ONESHOT_WriteMotors(oneshot* Oneshot);
void ONESHOT_CalculateOutput(oneshot* Oneshot, mma* MMA);

#endif // ONESHOT_CONTROL_