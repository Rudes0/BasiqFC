#include "imu.h"
#include "hardware/clocks.h"

int main()
{
    set_sys_clock_hz(220000000,true);
    stdio_init_all();
    angleKalmanFilter RollKalmanFilter;
    angleKalmanFilter PitchKalmanFilter;
    // velocityKalmanFilter VelocityKalmanFilter;
    imu IMU;
    mpu6500 MPU6500 = {
        .I2cMPU6500Port = i2c1,
        .MPU6500SclPin = 3, // i2c pin
        .MPU6500SdaPin = 2, // i2c pin
    };
    /* bme280 BME280 = 
    {
        .I2cBME280Port = i2c0,
        .BME280SclPin = 5,
        .BME280SdaPin = 4
    };
    */

    uint32_t timeloop = 4000;
    uint32_t currentTime;
    uint32_t sample_idx = 0;

    uint32_t start;
    uint32_t end;
    uint32_t time;

    

    // 3*   
    MPU6500_Init(&MPU6500);
    // BME280_Init(&BME280);

    IMU_InitializeIMU(&IMU);

    IMU_AngleInitializeKalman(&RollKalmanFilter);
    IMU_AngleInitializeKalman(&PitchKalmanFilter);
    // MU_VelocityInitializeKalman(&VelocityKalmanFilter);

    // maybe only this into init input data
    MPU6500_ReadData(&MPU6500);
    MPU6500_CalibrateData(&MPU6500);

    // BME280_readCalibrationData(&BME280);
    // BME280_CalculateReference(200,&BME280);

    IMU_AngleGetInput(&IMU, &MPU6500);
    // IMU_VelocityGetInput(&IMU, &MPU6500, &BME280);

    IMU_AngleSetKalmanInput(IMU.RollRaw,  &RollKalmanFilter);
    IMU_AngleSetKalmanInput(IMU.PitchRaw,  &PitchKalmanFilter);
    // IMU_VelocitySetKalmanInput(IMU.VelocityRaw, &VelocityKalmanFilter);
    float gyroXangle = IMU.RollRaw;
    float gyroYangle = IMU.PitchRaw;
    float compAngleX = IMU.RollRaw;
    float compAngleY = IMU.PitchRaw;
    // to this init input data
    // 3* put into initialize maybe? or not idk propably not 
    while(1)
    {
        currentTime = time_us_32();
        MPU6500_ReadData(&MPU6500);
        MPU6500_CalibrateData(&MPU6500);
        // BME280_ReadData(&BME280);
        // BME280_CalculateAltitude(&BME280);

        IMU_AngleGetInput(&IMU, &MPU6500);
        // IMU_VelocityGetInput(&IMU, &MPU6500, &BME280);
        IMU.RollKal =  IMU_AngleGetKalmanOutput(IMU.GyroX, IMU.RollRaw,  &RollKalmanFilter, 0.004f);
        IMU.PitchKal =  IMU_AngleGetKalmanOutput(IMU.GyroY, IMU.PitchRaw,  &PitchKalmanFilter, 0.004f);
        // IMU.VelocityKal = IMU_VelocityGetKalmanOutput(IMU.AccelerationRaw, IMU.Altitude, &VelocityKalmanFilter, 0.004f);
        // printf("Roll value [°] %2.2f | Pitch value [°] %2.2f\n", IMU.RollRaw, IMU.PitchRaw);
        // printf("Roll Kalman Value [°] %2.2f | Pitch Kalman value [°] %2.2f\n", IMU.RollKal, IMU.PitchKal);
        float time_s = sample_idx * 0.004f;
        // gyro angle without any filter
        gyroXangle += IMU.GyroX * 0.004f;
        gyroYangle += IMU.GyroY * 0.004f;
        // complementary filter output 
        compAngleX = 0.93 * (compAngleX + IMU.GyroX * 0.004f) + 0.07 * IMU.RollRaw; // Calculate the angle using a Complimentary filter
        compAngleY = 0.93 * (compAngleY + IMU.GyroY * 0.004f) + 0.07 * IMU.PitchRaw;
        // Exel format
          printf("%lu;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;\n",
            (unsigned long)sample_idx,
            IMU.RollRaw,
            gyroXangle,
            IMU.RollKal, 
            IMU.PitchRaw,
            gyroYangle,
            IMU.PitchKal
            );

        sample_idx++;
        while(time_us_32() - currentTime < timeloop);
    }
}