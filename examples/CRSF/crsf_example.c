#include "crsf.h"
#include <stdio.h>
#include "hardware/clocks.h"

int main(void)
{
    set_sys_clock_hz(220000000,true);
    stdio_init_all();
    crsf_data CRSF = 
    {
        .UartCRSFPort = uart0,
        .UartTxPin = 12,
        .UartRxPin = 13
    };
    CRSF_Init(&CRSF);
    while(1)
    {
        CRSF_StateMachine(&CRSF);
        printf("ch1 = %d ", CRSF.PWMData[0]);
        printf("ch2 = %d ", CRSF.PWMData[1]);
        printf("ch3 = %d ", CRSF.PWMData[2]);
        printf("ch4 = %d ", CRSF.PWMData[3]);
        printf("ch5 = %d  \n", CRSF.PWMData[4]);
        
        sleep_ms(5);
    }
    return 0;
}