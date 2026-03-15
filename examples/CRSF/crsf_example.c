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
    CRSF_Init(&CRSF);
    while(1)
    {
        CRSF_StateMachine(&CRSF);
        printf("ch1 = %d ", CRSF.pwmData[0]);
        printf("ch2 = %d ", CRSF.pwmData[1]);
        printf("ch3 = %d ", CRSF.pwmData[2]);
        printf("ch4 = %d ", CRSF.pwmData[3]);
        printf("ch5 = %d  \n", CRSF.pwmData[4]);
        
        sleep_ms(5);
    }
    return 0;
}