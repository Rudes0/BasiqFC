#include "input_control.h" 
#include "crsf.h"
#include <stdio.h>
#include "hardware/clocks.h"

int main(void)
{
    set_sys_clock_hz(220000000,true);
    stdio_init_all();
    crsf_data CRSF = 
    {
        .CRSFUartCRSFPort = uart0,
        .CRSFUartTxPin = 12,
        .CRSFUartRxPin = 13
        
    };
    input_control INPUT = {0, 0, 0, 0};
    CRSF_Init(&CRSF);
    while(1)
    {
        CRSF_StateMachine(&CRSF);
        INPUT_CONTROL_CalculateInput(&INPUT, &CRSF);
        // simple is armed function should not be copied in to main code 
        while(!(INPUT_CONTROL_IsArmed(&CRSF)))
        {
            CRSF_StateMachine(&CRSF);
            INPUT_CONTROL_CalculateInput(&INPUT, &CRSF);
            printf("Quadcopter is not armed! \n");
            sleep_ms(10);
        }
        INPUT_CONTROL_LimitThrottle(&INPUT);
        printf("Roll value = %f  ", INPUT.roll);
        printf("Pitch value = %f  ", INPUT.pitch);
        printf("throttle value = %f  ", INPUT.throttle);
        printf("yaw value = %f  \n", INPUT.yaw);
        
        sleep_ms(5);
    }
    return 0;
}