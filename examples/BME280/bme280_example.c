#include "bme280.h"
#include "hardware/clocks.h"
// ---------------------------------------
// Main function 
// ---------------------------------------
int main(void)
{
    set_sys_clock_hz(220000000,true);
    stdio_init_all();
    uint32_t timeloop = 4000;
    uint32_t currentTime;
    uint32_t sample_idx = 0;
    bme280 BME280 = 
    {
        .I2cBME280Port = i2c1,
        .BME280SdaPin = 10,
        .BME280SclPin = 11 
    };
    BME280_Init(&BME280);
    BME280_readCalibrationData(&BME280);
    BME280_CalculateReference(200,&BME280);
    sleep_ms(100);
    while(1)
    {
        currentTime = time_us_32();
        BME280_ReadData(&BME280);
        BME280_CalculateAltitude(&BME280);
        float time_s = sample_idx * 0.004f;
        printf("%lu;%.3f;\n",
               (unsigned long)sample_idx,
               BME280.altitudeM);
        sample_idx++;
        while(time_us_32() - currentTime < timeloop);
    }
    
    return 0;
}
