#ifndef MPU6500_DRIVER_
#define MPU6500_DRIVER_

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// ---------------------------------------
// I2C setup
// ---------------------------------------
#define MPU6500_address 0x68

// ---------------------------------------
// MPU6500 Accelerometer calibration values
// ---------------------------------------
#define fAccelCalibX    0.012 // dla wykresow na 0.039 dla drona 0.012 
#define fAccelCalibY    0.009 // dla wykresow na 0.030 dla drona 0.009
#define fAccelCalibZ    0.000

// ---------------------------------------
// MPU6500 structs
// ---------------------------------------
typedef struct
{
    i2c_inst_t* MPU6500I2cPort;
    uint8_t     MPU6500SclPin;
    uint8_t     MPU6500SdaPin;

    uint8_t     val;
    uint8_t     buff[14]; // buf that is used for reading data
    int16_t     accelX, accelY, accelZ; // raw accelerometer values
    int16_t     gyroX, gyroY, gyroZ; // raw gyroscope values
    int16_t     temperature; // raw temperature value
    float       tempOut; // output temperature value
    float       fAccelX, fAccelY, fAccelZ; // output accelerometer values
    float       fGyroX, fGyroY, fGyroZ; // output gyroscope values
    float       fGyroCalibX, fGyroCalibY, fGyroCalibZ; // gyroscope calibration values 
    
}mpu6500;

// ---------------------------------------
// Public API
// ---------------------------------------
void MPU6500_Init(mpu6500* MPU6500);
void MPU6500_ReadData(mpu6500* MPU6500);
void MPU6500_CalibrateData(mpu6500* MPU6500);

// ---------------------------------------
// Internal functions
// ---------------------------------------
void MPU6500_I2cInnit(mpu6500* MPU6500);
uint8_t MPU6500_I2cScanner(mpu6500* MPU6500);
static void MPU6500_WriteSingleData(uint8_t reg, uint8_t value, mpu6500* MPU6500);
uint8_t MPU6500_ReadRegister(uint8_t reg, mpu6500* MPU6500);
void MPU6500_CalibrationSamples(mpu6500* MPU6500);

#endif // MPU6500_DRIVER_