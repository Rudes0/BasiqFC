#include "mpu6500.h"
#include "hardware/clocks.h"
int main()
{
    //set_sys_clock_hz(220000000,true);
    stdio_init_all();
    
    mpu6500 MPU6500 = {
        .I2cMPU6500Port = i2c1,
        .MPU6500SclPin = 3, // i2c pin
        .MPU6500SdaPin = 2, // i2c pin
    };
    uint32_t start;
    uint32_t end;
    uint32_t time;
    uint32_t count = 0;
    sleep_ms(1000);
    MPU6500_Init(&MPU6500);
    while(1)
    {
        //printf("value under register 0x1C = %d\n", MPU6500_ReadRegister(0x1C, &MPU6500));
        MPU6500_ReadData(&MPU6500);
        printf("%d;%.3f;%.3f;%.3f;%.2f;%.2f;%.2f;",count,MPU6500.fAccelX, MPU6500.fAccelY, MPU6500.fAccelZ, MPU6500.fGyroX, MPU6500.fGyroY, MPU6500.fGyroZ);
        MPU6500_CalibrateData(&MPU6500);
        //printf("X value %6.3f, Y value  %6.3f, Z value  %6.3f, X gyro value %7.2f, Y gyro value  %7.2f, Z gyro value  %7.2f, The temperature is %5.2f\n",
        //MPU6500.fAccelX, MPU6500.fAccelY, MPU6500.fAccelZ, MPU6500.fGyroX, MPU6500.fGyroY, MPU6500.fGyroZ, MPU6500.tempOut);
        printf("%.3f;%.3f;%.3f;%.2f;%.2f;%.2f\n",MPU6500.fAccelX, MPU6500.fAccelY, MPU6500.fAccelZ, MPU6500.fGyroX, MPU6500.fGyroY, MPU6500.fGyroZ);
        count++;
        sleep_ms(1);
    }
}