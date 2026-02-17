#ifndef IMU_MODULE_
#define IMU_MODULE_

#include <stdio.h>
#include "pico/stdlib.h"
#include "mpu6500.h"
#include "bme280.h"
#include <math.h>

// ---------------------------------------
// const Calibration data
// ---------------------------------------
#define Q_angle 0.005f // tested from 0.001f -> 0.0005f -> 0.0001f and its the best
#define Q_bias 0.003f // tested from 0.003f t-> 0.002f and its the best
#define R_measure 0.04f // tested from 0.05f -> 0.04f -> 0.03f and its the best 

#define Q_accel 0.001f
#define R_altitude 0.03f

#define DEG_TO_RAD (3.142 / 180)
#define RAD_TO_DEG (180.0f / 3.142)
// ---------------------------------------
// IMU struct
// ---------------------------------------
typedef struct 
{
    // rate of quadcopter 
    float GyroX, GyroY;
    // angles of quadcopter
    float RollRaw, PitchRaw, YawRaw; 
    // kalman output 
    float RollKal, PitchKal;  

    // altitude of quadcopter 
    float Altitude; 
    // velocity of quadcopter 
    float VelocityRaw;
    // acceleration in Z axis of quadcopter
    float AccelerationRaw;
    // kalman output
    float VelocityKal;

}imu;

typedef struct 
{
    float Angle , Bias, Rate, P[2][2]; 
}angleKalmanFilter;

typedef struct 
{
    float Altitude , Velocity,  P[2][2]; 
}velocityKalmanFilter;


// ---------------------------------------
// Public API
// ---------------------------------------
void IMU_InitializeIMU(imu* IMU);
void IMU_AngleInitializeKalman(angleKalmanFilter* KalmanFilter);
void IMU_AngleGetInput(imu* IMU, const mpu6500* MPU6500); 
float IMU_AngleGetKalmanOutput(float GyroReading , float RawValue, angleKalmanFilter* KalmanFilter, float dt);
void IMU_AngleSetKalmanInput(float RawValue, angleKalmanFilter* KalmanFilter);
void IMU_VelocityInitializeKalman(velocityKalmanFilter* KalmanFilter);
void IMU_VelocityGetInput(imu* IMU, const mpu6500* MPU6500, const bme280* BME280); 
float IMU_VelocityGetKalmanOutput(float Acceleration , float RawAltitude, velocityKalmanFilter* KalmanFilter, float dt);
void IMU_VelocitySetKalmanInput(float RawValue, velocityKalmanFilter* KalmanFilter);
// ---------------------------------------
// Internal functions
// ---------------------------------------

#endif // IMU_MODULE_