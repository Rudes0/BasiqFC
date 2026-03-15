#ifndef BME280_DRIVER_
#define BME280_DRIVER_

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// ---------------------------------------
// I2C setup
// ---------------------------------------
#define address 0x76

// ---------------------------------------
// BME 280 structs 
// ---------------------------------------
typedef struct 
{
    i2c_inst_t*  BME280I2cPort; // taking this from raspberrys pi pico hardware/i2c library look in datasheet page 92
    uint8_t      BME280SclPin; 
    uint8_t      BME280SdaPin; 

    uint32_t     PPa;
    uint32_t     P0Pa;
    uint32_t     adcP;
    uint32_t     adcT;
    uint32_t     tempPa;
    uint32_t     hPa;
    float        altitudeM;
    int32_t      altitudeCM; 
}bme280;

// ---------------------------------------
// BME 280 Bosch's structs 
// for reference look at https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf chapter 4 
// ---------------------------------------
typedef struct 
{
    // for temperature calibration 
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    // for pressure calibration 
    uint16_t dig_P1;
    int16_t  dig_P2;    
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
}bme280_calib_t;

// ---------------------------------------
// Public API
// ---------------------------------------
void BME280_Init(bme280* BME280);
void BME280_ReadData(bme280* BME280);

// ---------------------------------------
// Internal functions
// ---------------------------------------
void BME280_I2cInnit(bme280* BME280);
uint8_t BME280_I2cScanner (bme280* BME280);
void BME280_WriteSingleData(uint8_t reg, uint8_t value, bme280* BME280);
void BME280_ReadCalibrationData(bme280* BME280);
void BME280_CalculateReference (uint8_t NRef, bme280* BME280); 
int32_t BME280_compensate_T_int32(int32_t adc_T);
uint32_t BME280_compensate_P_int64(int32_t adc_P);

// TO DO: do modułu IMU 
void BME280_CalculateAltitude(bme280* BME280); // to do wyrzucenia do innego bloku aby obliczać wysokość w imu.c
float BME280_CalculateAltitudeTaylor(float P_Pa, float P0_Pa); // i to raz z tamtą funkcją 

#endif // BME280_DRIVER_