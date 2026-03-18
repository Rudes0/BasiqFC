#include "oneshot.h"

// ---------------------------------------
// Public API
// ---------------------------------------
void ONESHOT_InitMotors(oneshot* Oneshot) // initialization of all PWM channels needed for all the motors
{
    gpio_set_function(Oneshot->motorLF, GPIO_FUNC_PWM);
    gpio_set_function(Oneshot->motorLB, GPIO_FUNC_PWM);
    gpio_set_function(Oneshot->motorRF, GPIO_FUNC_PWM);
    gpio_set_function(Oneshot->motorRB, GPIO_FUNC_PWM);
    
    // getting the correct slice for each motor that enables basic functionality of PWM 
    Oneshot->sliceNumLF = pwm_gpio_to_slice_num(Oneshot->motorLF);
    Oneshot->sliceNumLB = pwm_gpio_to_slice_num(Oneshot->motorLB); 
    Oneshot->sliceNumRF = pwm_gpio_to_slice_num(Oneshot->motorRF);
    Oneshot->sliceNumRB = pwm_gpio_to_slice_num(Oneshot->motorRB);

    pwm_config config = pwm_get_default_config();
    // setting up the divider for correct HZ (aprox. 4kHz) 
    pwm_config_set_clkdiv_int(&config,220);

    pwm_init(Oneshot->sliceNumLF, &config,true);
    pwm_init(Oneshot->sliceNumLB, &config,true);
    pwm_init(Oneshot->sliceNumRF, &config,true);
    pwm_init(Oneshot->sliceNumRB, &config,true);
    // max value after wich it will wrap around and count again
    pwm_set_wrap(Oneshot->sliceNumLF, 249);
    pwm_set_wrap(Oneshot->sliceNumLB, 249);
    pwm_set_wrap(Oneshot->sliceNumRF, 249);
    pwm_set_wrap(Oneshot->sliceNumRB, 249);

    pwm_set_chan_level(Oneshot->sliceNumLF, PWM_CHAN_A, 125);
    pwm_set_chan_level(Oneshot->sliceNumLB, PWM_CHAN_B, 125);
    pwm_set_chan_level(Oneshot->sliceNumRF, PWM_CHAN_B, 125);
    pwm_set_chan_level(Oneshot->sliceNumRB, PWM_CHAN_A, 125);

    pwm_set_enabled(Oneshot->sliceNumLF, true);
    pwm_set_enabled(Oneshot->sliceNumLB, true);
    pwm_set_enabled(Oneshot->sliceNumRF, true);
    pwm_set_enabled(Oneshot->sliceNumRB, true);
}

void ONESHOT_WriteMotors(oneshot* Oneshot) 
{
    pwm_set_chan_level(Oneshot->sliceNumLF, PWM_CHAN_A, Oneshot->fillLF);
    pwm_set_chan_level(Oneshot->sliceNumLB, PWM_CHAN_B, Oneshot->fillLB);
    pwm_set_chan_level(Oneshot->sliceNumRF, PWM_CHAN_B, Oneshot->fillRF);
    pwm_set_chan_level(Oneshot->sliceNumRB, PWM_CHAN_A, Oneshot->fillRB);
}

void ONESHOT_CalculateOutput(oneshot* Oneshot, mma* MMA)
{
    Oneshot->fillLF = (MMA->motorLF * 125) / 1000;
    Oneshot->fillLB = (MMA->motorLB * 125) / 1000;
    Oneshot->fillRF = (MMA->motorRF * 125) / 1000;
    Oneshot->fillRB = (MMA->motorRB * 125) / 1000;
}

// ---------------------------------------
// Internal functions
// ---------------------------------------