#include "imu.h"

// ---------------------------------------
// Public API
// ---------------------------------------
void IMU_InitializeIMU(imu* IMU)
{
    IMU->gyroX = 0;
    IMU->gyroY = 0;
    IMU->rollRaw = 0;
    IMU->pitchRaw = 0;
    IMU->yawRaw = 0;
    IMU->rollKal = 0;
    IMU->pitchKal = 0;
    IMU->accelerationRaw = 0;
}

void IMU_AngleInitializeKalman(angleKalmanFilter* KalmanFilter)
{
    KalmanFilter->angle = 0;
    KalmanFilter->bias = 0;
    KalmanFilter->rate = 0;
    KalmanFilter->p[0][0] = 0; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    KalmanFilter->p[0][1] = 0;
    KalmanFilter->p[1][0] = 0;
    KalmanFilter->p[1][1] = 0;

}

void IMU_AngleGetInput(imu* IMU, const mpu6500* MPU6500)
{

    // double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    // double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
    IMU->rollRaw = atan2(MPU6500->fAccelY, MPU6500->fAccelZ) * RAD_TO_DEG;
    IMU->pitchRaw = atan(-(MPU6500->fAccelX) / sqrt(MPU6500->fAccelY * MPU6500->fAccelY + MPU6500->fAccelZ * MPU6500->fAccelZ)) * RAD_TO_DEG;

    // could not do this but i want to seperate MPU from Kalman filter and let IMU hold all the values  
    IMU->gyroX = MPU6500->fGyroX;
    IMU->gyroY = MPU6500->fGyroY; 

    IMU->yawRaw = MPU6500->fGyroZ;  
    
}

void IMU_AngleSetKalmanInput(float RawValue, angleKalmanFilter* KalmanFilter) // starting input for kalman filter
{
    KalmanFilter->angle = RawValue;
}

float IMU_AngleGetKalmanOutput(float GyroValue , float RawValue, angleKalmanFilter* KalmanFilter, float dt) // dt needs to be replaced by loop_time struct 
{   
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it
    
    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    KalmanFilter->rate = GyroValue - KalmanFilter->bias;
    KalmanFilter->angle += dt * KalmanFilter->rate;  

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    KalmanFilter->p[0][0] += dt * (dt * KalmanFilter->p[1][1] - KalmanFilter->p[0][1] - KalmanFilter->p[1][0] + Q_angle);
    KalmanFilter->p[0][1] -= dt * KalmanFilter->p[1][1];
    KalmanFilter->p[1][0] -= dt * KalmanFilter->p[1][1];
    KalmanFilter->p[1][1] += Q_bias * dt;


    // Calculate angle and bias - Update estimate with measurement zk (RawValue)
    /* Step 3 */
    float Y = RawValue - KalmanFilter->angle;  //  Angle difference
    
    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = KalmanFilter->p[0][0] + R_measure; //  Estimate error

    /* Step 5 */
    float K[2]; // Roll Kalman gain - This is a 2x1 vector
    K[0] = KalmanFilter->p[0][0] / S;
    K[1] = KalmanFilter->p[1][0] / S;

    /* Step 6 */
    KalmanFilter->angle  += K[0] * Y;
    KalmanFilter->bias += K[1] * Y;


    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = KalmanFilter->p[0][0];
    float P01_temp = KalmanFilter->p[0][1];

    KalmanFilter->p[0][0] -= K[0] * P00_temp;
    KalmanFilter->p[0][1] -= K[0] * P01_temp;
    KalmanFilter->p[1][0] -= K[1] * P00_temp;
    KalmanFilter->p[1][1] -= K[1] * P01_temp;


    /* Output */
    return KalmanFilter->angle;
}

void IMU_VelocityInitializeKalman(velocityKalmanFilter* KalmanFilter)
{
    KalmanFilter->altitude = 0;
    KalmanFilter->velocity = 0;
    KalmanFilter->p[0][0] = 1.0f; 
    KalmanFilter->p[0][1] = 0.0f;
    KalmanFilter->p[1][0] = 0.0f;
    KalmanFilter->p[1][1] = 1.0f;
}

void IMU_VelocityGetInput(imu* IMU, const mpu6500* MPU6500, const bme280* BME280)
{
    IMU->altitude = BME280->altitudeM;
    IMU->accelerationRaw = -sin(IMU->pitchRaw * DEG_TO_RAD) * MPU6500->fAccelX + cos(IMU->pitchRaw * DEG_TO_RAD) * sin(IMU->rollRaw * DEG_TO_RAD) * MPU6500->fAccelY + cos(IMU->rollRaw * DEG_TO_RAD) * cos(IMU->pitchRaw * DEG_TO_RAD) * MPU6500->fAccelZ;
    IMU->accelerationRaw = (IMU->accelerationRaw - 1) * 9.81f;
    IMU->velocityRaw += IMU->accelerationRaw * 0.004f;
}

void IMU_VelocitySetKalmanInput(float RawValue, velocityKalmanFilter* KalmanFilter) // starting input for kalman filter
{
    KalmanFilter->velocity = RawValue;
}

// to jest do naprawy nie wiem o co chodzi możliwe że trzeba dodać GPS i z niego liczyć wysokość drona albo jakąś kamere 
float IMU_VelocityGetKalmanOutput(float RawAcceleration , float RawAltitude, velocityKalmanFilter* KalmanFilter, float dt) // dt needs to be replaced by loop_time struct 
{   
    // Modified by Filip Michalak   
    // Based on http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it and https://github.com/rblilja/AltitudeKF/blob/master/altitude_kf.cpp 
    KalmanFilter->altitude += KalmanFilter->velocity * dt +  0.5f * RawAcceleration * dt * dt; // kalman rate = gyrovalue - kalman bias 
    KalmanFilter->velocity += dt * RawAcceleration;  // angle += dt * rate

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    KalmanFilter->p[0][0] += dt * (dt * KalmanFilter->p[1][1] - KalmanFilter->p[0][1] - KalmanFilter->p[1][0] + Q_accel);
    KalmanFilter->p[0][1] -= dt * KalmanFilter->p[1][1];
    KalmanFilter->p[1][0] -= dt * KalmanFilter->p[1][1];
    KalmanFilter->p[1][1] += Q_accel * dt;


    // Calculate angle and bias - Update estimate with measurement zk (RawValue)
    /* Step 3 */
    float Y = RawAltitude - KalmanFilter->altitude;  //  raw value - kalman angle  
    
    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = KalmanFilter->p[0][0] + R_measure; //  Estimate error

    /* Step 5 */
    float K[2]; // Roll Kalman gain - This is a 2x1 vector
    K[0] = KalmanFilter->p[0][0] / S;
    K[1] = KalmanFilter->p[1][0] / S;

    /* Step 6 */
    KalmanFilter->altitude  += K[0] * Y; // angle 
    KalmanFilter->velocity += K[1] * Y; // bias


    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = KalmanFilter->p[0][0];
    float P01_temp = KalmanFilter->p[0][1];

    KalmanFilter->p[0][0] -= K[0] * P00_temp;
    KalmanFilter->p[0][1] -= K[0] * P01_temp;
    KalmanFilter->p[1][0] -= K[1] * P00_temp;
    KalmanFilter->p[1][1] -= K[1] * P01_temp;


    /* Output */
    return KalmanFilter->velocity;
}

// ---------------------------------------
// Internal functions
// ---------------------------------------