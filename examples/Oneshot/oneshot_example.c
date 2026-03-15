#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "oneshot.h"
#include "crsf.h"

int main()
{
    set_sys_clock_hz(220000000,true);
    stdio_init_all();
    crsf_data CRSF = {
        .CRSFUartCRSFPort = uart0,
        .CRSFUartTxPin = 12,
        .CRSFUartRxPin = 13
    };
    CRSF_Init(&CRSF);
    oneshot Oneshot = {
        .motorLF = 14,
        //.motorLB = 15,
        //.motorRF = 16,
        //.motorRB = 17
    };
    ONESHOT_InitMotors(&Oneshot);

    while (true) 
    {
        CRSF_StateMachine(&CRSF);
        CRSF_ChannelMaping(&CRSF);
        /*
        printf("ch1 = %d ", Crsf.PWMData[0]);
        printf("ch2 = %d ", Crsf.PWMData[1]);
        printf("ch3 = %d ", Crsf.PWMData[2]);
        printf("ch4 = %d  \n", Crsf.PWMData[3]);
        */
        Oneshot.fillLF = (CRSF.pwmData[0] * 125) / 1000;
        if(Oneshot.fillLF > 240) Oneshot.fillLF = 240;
        //Oneshot.fillLB = (CRSF.PWMData[1] * 125) / 1000;
        //if(Oneshot.fillLB > 240) Oneshot.fillLB = 240;
        //Oneshot.fillRF = (CRSF.PWMData[2] * 125) / 1000;
        //if(Oneshot.fillRF > 240) Oneshot.fillRF = 240;
        //Oneshot.fillRB = (CRSF.PWMData[3] * 125) / 1000;
        //if(Oneshot.fillRB > 240) Oneshot.fillRB = 240;
        printf("LF = %d ", Oneshot.fillLF);
       // printf("LB = %d ", Oneshot.fillLB);
       //printf("RF = %d ", Oneshot.fillRF);
        //printf("RB = %d  \n", Oneshot.fillRB);
        ONESHOT_WriteMotors(&Oneshot);
        sleep_ms(5);
    }
}