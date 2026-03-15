/*
Filip Michalak 2025

BME280 Driver for Drone project 

This driver is only suitable for drone application and it should be only used for this purpose

This driver assumes you are using only one BME280 module as it wasnt tested with multiple ones
*/

#include "mpu6500.h"
// ---------------------------------------
// Public API
// ---------------------------------------
void MPU6500_Init(mpu6500* MPU6500) // initialization of MPU6500 module 
{   
    
    MPU6500_I2cInnit(MPU6500);
    MPU6500_WriteSingleData(0x6B, 0x80, MPU6500);
    sleep_ms(100);

    MPU6500_WriteSingleData(0x6B, 0x01, MPU6500);
    sleep_ms(10);
    if(!(MPU6500_I2cScanner(MPU6500)))
    {
        while(1)
        {
            printf("wrong chip! Check pins or module!\n");
            sleep_ms(500);
        }
    }
    
    MPU6500_WriteSingleData(0x6B, 0x01, MPU6500); // waking up the MPU and setting internal oscilator to best available
    MPU6500_WriteSingleData(0x6C, 0x00, MPU6500); // ensuring that all axis are working 
    MPU6500_WriteSingleData(0x19, 0x00, MPU6500); // set the sample rate to 1000Hz 1Khz / 1 = 1kHz
    MPU6500_WriteSingleData(0x1A, 0x05, MPU6500); // DLPF 
    MPU6500_WriteSingleData(0x1B, 0x08, MPU6500); // range +/-500 dps
    MPU6500_WriteSingleData(0x1C, 0x10, MPU6500); // range +/- 8g
    MPU6500_WriteSingleData(0x1D, 0x05, MPU6500); // DLPF 
    printf("value under register 0x1C = %d", MPU6500_ReadRegister(0x1C, MPU6500));
    printf("Begining calibration samples");
    MPU6500_CalibrationSamples(MPU6500);
    printf("Calibration finieshed!\n");
}

void MPU6500_ReadData(mpu6500* MPU6500) // reading all the needed data and storing in MPU6500 struct 
{
        i2c_write_blocking(MPU6500->MPU6500I2cPort, MPU6500_address, &MPU6500->val, 1, true);
        i2c_read_blocking(MPU6500->MPU6500I2cPort, MPU6500_address, MPU6500->buff, 14, false);

        MPU6500->accelX = ((MPU6500->buff[0]<<8) | MPU6500->buff[1]);
        MPU6500->accelY = ((MPU6500->buff[2]<<8) | MPU6500->buff[3]);
        MPU6500->accelZ = ((MPU6500->buff[4]<<8) | MPU6500->buff[5]);

        MPU6500->temperature = ((MPU6500->buff[6] << 8) | MPU6500->buff[7]);
        MPU6500->tempOut = (MPU6500->temperature / 333.87f) + 21.0f;

        MPU6500->gyroX = ((MPU6500->buff[8]<<8) | MPU6500->buff[9]);
        MPU6500->gyroY = ((MPU6500->buff[10]<<8) | MPU6500->buff[11]);
        MPU6500->gyroZ = ((MPU6500->buff[12]<<8) | MPU6500->buff[13]);
        
        MPU6500->fAccelX = MPU6500->accelX / 4096.0f;
        MPU6500->fAccelY = MPU6500->accelY / 4096.0f;
        MPU6500->fAccelZ = MPU6500->accelZ / 4096.0f;

        MPU6500->fGyroX = (MPU6500->gyroX / 65.5f);
        MPU6500->fGyroY = (MPU6500->gyroY / 65.5f);
        MPU6500->fGyroZ = (MPU6500->gyroZ / 65.5f);
}

void MPU6500_CalibrateData(mpu6500* MPU6500)
{
    MPU6500->fGyroX -= MPU6500->fGyroCalibX;
    MPU6500->fGyroY -= MPU6500->fGyroCalibY;
    MPU6500->fGyroZ -= MPU6500->fGyroCalibZ;

    MPU6500->fAccelX -= fAccelCalibX;
    MPU6500->fAccelY -= fAccelCalibY;
    MPU6500->fAccelZ -= fAccelCalibZ;
}

// ---------------------------------------
// Internal functions
// ---------------------------------------
void MPU6500_I2cInnit(mpu6500* MPU6500) // initialiaze MPU6500 i2c communication protocol 
{
    i2c_init(MPU6500->MPU6500I2cPort, 100 * 1000);
    gpio_set_function(MPU6500->MPU6500SclPin, GPIO_FUNC_I2C);
    gpio_set_function(MPU6500->MPU6500SdaPin, GPIO_FUNC_I2C);
    gpio_pull_up(MPU6500->MPU6500SclPin);
    gpio_pull_up(MPU6500->MPU6500SdaPin);
    MPU6500->val = 0x3B;
}

uint8_t MPU6500_I2cScanner(mpu6500* MPU6500) // scanning if correct module is being used
{
    sleep_ms(1000); // waiting for I2C to set up 
    uint8_t chipID[1];
    uint8_t reg = 0x75;
    i2c_write_blocking(MPU6500->MPU6500I2cPort, MPU6500_address, &reg, 1, true);
    i2c_read_blocking(MPU6500->MPU6500I2cPort, MPU6500_address, &chipID[0], 1, false);
    printf("The chip's ID is 0x%X",chipID[0]); 
    if(chipID[0] == 0x70) 
    {
        return 1;
    }
    return 0;
}

static void MPU6500_WriteSingleData(uint8_t reg, uint8_t value, mpu6500* MPU6500) // sending single data 
{
    uint8_t data[2] = {reg, value};
    i2c_write_blocking(MPU6500->MPU6500I2cPort, MPU6500_address, data, 2, false);
} 

uint8_t MPU6500_ReadRegister(uint8_t reg, mpu6500* MPU6500) // reading data for debbuging purposes
{
    uint8_t val;
    i2c_write_blocking(MPU6500->MPU6500I2cPort, MPU6500_address, &reg, 1, true);
    i2c_read_blocking(MPU6500->MPU6500I2cPort, MPU6500_address, &val, 1, false);
    return val;
}


void MPU6500_CalibrationSamples(mpu6500* MPU6500) // creating 2000 samples (waiting 2s) for calibrating gyro build-in inaccuracy
{
    float gyroXCalib = 0;
    float gyroYCalib = 0;
    float gyroZCalib = 0;
    for(uint16_t i = 0; i < 2000; i++)
    {
        MPU6500_ReadData(MPU6500);
        gyroXCalib += MPU6500->fGyroX;
        gyroYCalib += MPU6500->fGyroY;
        gyroZCalib += MPU6500->fGyroZ;
        sleep_ms(1);
    }
    MPU6500->fGyroCalibX = gyroXCalib / 2000;    
    MPU6500->fGyroCalibY = gyroYCalib / 2000;
    MPU6500->fGyroCalibZ = gyroZCalib / 2000;
}