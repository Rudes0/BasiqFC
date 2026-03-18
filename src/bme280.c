/*
Filip Michalak 2025

BME280 Driver for Drone project 

If you want to use this driver for any other use than drone you should write own BME280Init function as it assumes you will use this driver for drone application

In order to use this Drvier you have to fill BME280 variables with correct data 

This driver assumes you are using only one BME280 module
*/

#include "bme280.h"

// ---------------------------------------
// Global variables for calibration 
// ---------------------------------------
bme280_calib_t calib; // calibration struct
static int32_t t_fine; // global as Bosch sugests  

// ---------------------------------------
// Public API
// ---------------------------------------
void BME280_Init(bme280* BME280) // initialization of BME280 Module 
{
    BME280_I2cInnit(BME280);
    sleep_ms(10);
    if(!(BME280_I2cScanner(BME280))) 
    {
        while(1)
        {
            printf("Wrong chip!");
            sleep_ms(500);
        }
    }
    
    BME280_WriteSingleData(0xF2,0x00, BME280); // 0b00000000 humidity off 
    BME280_WriteSingleData(0xF4,0x37, BME280); // 0b00110111 oversampling for temperature sensor = 1, oversampling for pressure sensor = 16, mode = normal 
    BME280_WriteSingleData(0xF5,0x10, BME280); // 0b00010000 standby time = 0.5ms, fillter coefficient = 16

    BME280->PPa = 0;
    BME280->P0Pa = 0;
    BME280->adcP = 0;
    BME280->adcT = 0;
    BME280->tempPa = 0;
    BME280->hPa = 0;
    BME280->altitudeM = 0;
    BME280->altitudeCM = 0; 

    BME280_ReadCalibrationData(BME280);
    sleep_ms(50); // it needs to be there for some reason but if its not there then altitude is read wrong 
    BME280_CalculateReference(200, BME280);
}

void BME280_ReadData(bme280* BME280)
{
    uint8_t buf[6];
    uint8_t start = 0xF7;
    i2c_write_blocking(BME280->BME280I2cPort, address, &start, 1, true);
    i2c_read_blocking(BME280->BME280I2cPort, address, buf, 6, false);
    BME280->adcP = ((int32_t)buf[0] << 12| (int32_t)buf[1] << 4 | (int32_t)buf[2] >> 4);
    BME280->adcT = ((int32_t)buf[3] << 12| (int32_t)buf[4] << 4 | (int32_t)buf[5] >> 4);
    BME280_compensate_T_int32(BME280->adcT);
    BME280->tempPa = BME280_compensate_P_int64(BME280->adcP);
    BME280->PPa = BME280->tempPa >> 8;
    BME280->hPa = BME280->PPa;
}

// ---------------------------------------
// Internal Functions
// ---------------------------------------
void BME280_I2cInnit(bme280* BME280) // initialization of i2c communication 
{
    i2c_init(BME280->BME280I2cPort, 400 * 1000);
    gpio_set_function(BME280->BME280SclPin, GPIO_FUNC_I2C);
    gpio_set_function(BME280->BME280SdaPin, GPIO_FUNC_I2C);
    gpio_pull_up(BME280->BME280SclPin);
    gpio_pull_up(BME280->BME280SdaPin);
}

uint8_t BME280_I2cScanner(bme280* BME280) // scanning if correct module is being used
{
    sleep_ms(1000); // waiting for I2C to set up 
    uint8_t chipID[1];
    uint8_t reg = 0xD0;
    i2c_write_blocking(BME280->BME280I2cPort, address, &reg, 1, true);
    i2c_read_blocking(BME280->BME280I2cPort, address, &chipID[0], 1, false);
    printf("The chip's ID is 0x%X",chipID[0]); 
    if(chipID[0] == 0x60) 
    {
        return 1;
    }
    return 0;
}

void BME280_WriteSingleData(uint8_t reg, uint8_t value, bme280* BME280) // sending single data 
{
    uint8_t data[2] = {reg, value};
    i2c_write_blocking(BME280->BME280I2cPort, address, data, 2, false);
}

void BME280_ReadCalibrationData(bme280* BME280) // reading calibration data needed for temperature and pressure   
{
    uint8_t buf[24];
    uint8_t reg = 0x88;
    i2c_write_blocking(BME280->BME280I2cPort, address, &reg, 1, true);
    i2c_read_blocking(BME280->BME280I2cPort, address, buf, 24, false);
    // for temperature calibration
    calib.dig_T1 = (uint16_t)(buf[1] << 8 | buf[0]);
    calib.dig_T2 = (int16_t)(buf[3] << 8 | buf[2]);
    calib.dig_T3 = (int16_t)(buf[5] << 8 | buf[4]);
    // for pressure calibration
    calib.dig_P1 = (uint16_t)(buf[7] << 8 | buf[6]);
    calib.dig_P2 = (int16_t)(buf[9] << 8 | buf[8]);
    calib.dig_P3 = (int16_t)(buf[11] << 8 | buf[10]);
    calib.dig_P4 = (int16_t)(buf[13] << 8 | buf[12]);
    calib.dig_P5 = (int16_t)(buf[15] << 8 | buf[14]);
    calib.dig_P6 = (int16_t)(buf[17] << 8 | buf[16]);
    calib.dig_P7 = (int16_t)(buf[19] << 8 | buf[18]);
    calib.dig_P8 = (int16_t)(buf[21] << 8 | buf[20]);
    calib.dig_P9 = (int16_t)(buf[23] << 8 | buf[22]);
}

void BME280_CalculateReference(uint8_t NRef, bme280* BME280) // calculating reference value for later use of calculating altitude 
{
    int64_t sumPa = 0;
    int32_t rawPa, rawT, Pa;
    uint32_t tempPa;
    printf("Collecting sample Reference...\n");
    for(uint8_t i = 0; i < NRef; i++)
    {
        uint8_t buf[6];
        uint8_t reg = 0xF7;
        i2c_write_blocking(BME280->BME280I2cPort, address, &reg, 1, true);
        if(i2c_read_blocking(BME280->BME280I2cPort, address, buf, 6, false) != 6) // repeat if couldnt read 
        {
            NRef--;
            continue;
        }
        rawPa = ((int32_t)buf[0] << 12| (int32_t)buf[1] << 4 | (int32_t)buf[2] >> 4);
        rawT = ((int32_t)buf[3] << 12| (int32_t)buf[4] << 4 | (int32_t)buf[5] >> 4);
        BME280_compensate_T_int32(rawT);
        tempPa = BME280_compensate_P_int64(rawPa);
        Pa = tempPa >> 8; // we shift 8 bits so we do not have to divide by 256
        sumPa += Pa;
        sleep_ms(10);
    }
    printf("The reference pressure is going to be %ld.%02ld hPa.\n", sumPa / NRef, (sumPa / NRef) % 100);
    BME280->P0Pa = sumPa / NRef;
}

int32_t BME280_compensate_T_int32(int32_t adc_T) // formula compensating the temperature reading 
{
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) *
              ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) *
            ((int32_t)calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;  // houndredths °C (not used later but requaired to be made because of pressure compensation)
    return T;
}

uint32_t BME280_compensate_P_int64(int32_t adc_P) // formula compensating the pressure reading
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) +
           ((var1 * (int64_t)calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;
    if (var1 == 0) return 0; // check if we divide by 0

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);

    return (uint32_t)p; // Pa * 256
}



void BME280_CalculateAltitude(bme280* BME280)
{
    BME280->altitudeM = BME280_CalculateAltitudeTaylor((float)BME280->PPa, (float)BME280->P0Pa);
    BME280->altitudeCM = (int32_t)(BME280->altitudeM * 100.0f);
}

// to raczej usunac nie ma sensu tego w ten sposob robic  i lepiej policzyc to normalnie ze wzoru i z math.h
float BME280_CalculateAltitudeTaylor(float P_Pa, float P0_Pa) // altitude aproximation with Taylor series  
{
    // we are using this formula h = 44330 * [1 - (P / P0)^0.1903]
    // the power operation is expensive to CPU so we are going to aproximate the power function of (P / P0)^0.1903 
    // we assume change of P is going to be small (0.001 to 0.005) so P / P0 is going to be equal to aprox. 1 so we aproximate 1^0.1903 
    // this function returns value in meters  
    if (P_Pa <= 0.0f || P0_Pa <= 0.0f) return 0;
    float change = P_Pa/P0_Pa;
    float dx = 1 - change;
    return 44330.0f * (0.1903f * dx + 0.0777f * dx * dx);
}