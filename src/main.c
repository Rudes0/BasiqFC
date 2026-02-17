#include <stdio.h>
#include "pico/stdlib.h"
#include "crsf.h"
#include "imu.h"
#include "input_control.h"
#include "loop_time.h"
#include "mma.h"
#include "mpu6500.h"
#include "oneshot.h"
#include "pid.h"
// mix multiplier for correcting values from (deg) to (us) 
#define MIX_mulitplier 1 

enum logs
{
    LogsOff,
    LogsOn
};

enum mode
{
    RateMode,
    AngleMode
};

enum state  
{
    NotArmed,
    Armed 
};

int main()
{
    // settup enums for debugging
    enum logs LOG = LogsOff; 
    enum mode MODE = RateMode;

    // loopTime struct
    loop_time LOOP_TIME =
    {
        .sysSpeed = 220000000,
        .loopSpeed = 4000,
        .dt = 0.004f
    };
    // overclocking raspberry pi pico
    setSystemClockSpeed(&LOOP_TIME);
    stdio_init_all();

    enum state State = NotArmed;
    // MPU6500 struct
    mpu6500 MPU6500 = 
    {
        .I2cMPU6500Port = i2c1,
        .MPU6500SclPin = 3,
        .MPU6500SdaPin = 2
    }; 

    // CRSF struct 
    crsf_data CRSF = 
    {
        .UartCRSFPort = uart0,
        .UartTxPin = 12,
        .UartRxPin = 13
    };

    // Oneshot struct
    oneshot ONESHOT =
    {
        .motorLF = 14,
        .motorLB = 15,
        .motorRF = 17,
        .motorRB = 16,
        .fillLB = 125,
        .fillLF = 125,
        .fillRB = 125,
        .fillRF = 125
    };

    // IMU struct 
    imu IMU;
    angleKalmanFilter RollKalmanFilter;
    angleKalmanFilter PitchKalmanFilter;

    // input controll struct
    input_control INPUT_CONTROL = {0, 0, 0, 0};

    // PID structs for RateRoll, RatePitch, AngleRoll, AnglePitch, Yaw 
    pid PIDRateRoll = 
    {
        .Kp = 0.3,
        .Ki = 0.05,
        .Kd = 0.01,
        .antyWindupMax = 300,
        .antyWindupMin = -300,
        .dt = 0.004f,
        .limit = 500
    };
    pid PIDRatePitch = 
    {
        .Kp = 0.3,
        .Ki = 0.05,
        .Kd = 0.01,
        .antyWindupMax = 300,
        .antyWindupMin = -300,
        .dt = 0.004f,
        .limit = 500
    };
    pid PIDAngleRoll = 
    {
        .Kp = 2,
        .Ki = 0,
        .Kd = 0,
        .antyWindupMax = 100,
        .antyWindupMin = -100,
        .dt = 0.004f,
        .limit = 200
    };
    pid PIDAnglePitch = 
    {
        .Kp = 2,
        .Ki = 0,
        .Kd = 0,
        .antyWindupMax = 100,
        .antyWindupMin = -100,
        .dt = 0.004f,
        .limit = 200
    };
    pid PIDYaw = 
    {
        .Kp = 0.5,
        .Ki = 0.01,
        .Kd = 0.001,
        .antyWindupMax = 100,
        .antyWindupMin = -100,
        .dt = 0.004f,
        .limit = 200
    };

    // mma struct 
    mma MMA;


    // initialization of MPU6500  
    MPU6500_Init(&MPU6500);

    // initialization of CRSF protocol 
    CRSF_Init(&CRSF);

    // initialization of Oneshot protocol 
    ONESHOT_initMotors(&ONESHOT);

    // initialization of imu
    IMU_InitializeIMU(&IMU);

    IMU_AngleInitializeKalman(&RollKalmanFilter);
    IMU_AngleInitializeKalman(&PitchKalmanFilter);
    
    // initialization of pid
    PID_init(&PIDRateRoll);
    PID_init(&PIDRatePitch);
    PID_init(&PIDAngleRoll);
    PID_init(&PIDAnglePitch);
    PID_init(&PIDYaw); 

    // Calibration of all modules 
    // MPU6500
    MPU6500_ReadData(&MPU6500);
    MPU6500_CalibrateData(&MPU6500);

    // IMU
    IMU_AngleGetInput(&IMU, &MPU6500);
    // Kalmans 
    IMU_AngleSetKalmanInput(IMU.RollRaw,  &RollKalmanFilter);
    IMU_AngleSetKalmanInput(IMU.PitchRaw,  &PitchKalmanFilter);
    // maybe put it in some other file to manage it 
    printSystemClockSpeed(&LOOP_TIME);
    
    // while loop 
    while(1)
    {   
        // start of main loop control 
        startLoop(&LOOP_TIME);
        
        // read IMU data
        MPU6500_ReadData(&MPU6500);
        MPU6500_CalibrateData(&MPU6500);

        IMU_AngleGetInput(&IMU, &MPU6500);

        IMU.RollKal =  IMU_AngleGetKalmanOutput(IMU.GyroX, IMU.RollRaw,  &RollKalmanFilter, LOOP_TIME.dt);
        IMU.PitchKal =  IMU_AngleGetKalmanOutput(IMU.GyroY, IMU.PitchRaw,  &PitchKalmanFilter, LOOP_TIME.dt);

        // printf("IMU ROLL = %.4f, IMU PITCH = %.4f\n", IMU.RollKal, IMU.PitchKal);
        // read transmiter data
        CRSF_StateMachine(&CRSF);
        INPUT_CONTROL_CalculateInput(&INPUT_CONTROL, &CRSF);
        // printf("Input throttle = %f", INPUT_CONTROL.throttle);
        // check if we are ready to fly 
        if(INPUT_CONTROL_isArmed(&CRSF) && State == NotArmed && INPUT_CONTROL.throttle < 1050)
        {
            State = Armed;
        }
        // check if we want to stop flying 
        if(!(INPUT_CONTROL_isArmed(&CRSF)))
        {
            State = NotArmed;
            
        }
        
        if(State == NotArmed)
        {
            // Reset all earlier values for again takeoff
            PID_reset(&PIDAngleRoll);
            PID_reset(&PIDAnglePitch);
            PID_reset(&PIDRateRoll);
            PID_reset(&PIDRatePitch);
            PID_reset(&PIDYaw);
            ONESHOT.fillLB = 125;
            ONESHOT.fillLF = 125;
            ONESHOT.fillRB = 125;
            ONESHOT.fillRF = 125;
            if(LOG == LogsOff){
                ONESHOT_writeMotors(&ONESHOT);
            }
            
            printf("State not Armed!");
            continue;
        }
        // PID calculations 
        if(INPUT_CONTROL.throttle < 1150)
        {
            PIDAngleRoll.i = 0;
            PIDAnglePitch.i = 0;
            PIDRateRoll.i = 0;
            PIDRatePitch.i = 0;
            PIDYaw.i = 0;
        }
        if(MODE == RateMode)
        {
            PID_calculate(&PIDRateRoll, INPUT_CONTROL.roll, MPU6500.fGyroX);
            PID_calculate(&PIDRatePitch, INPUT_CONTROL.pitch, MPU6500.fGyroY);
            PID_calculate(&PIDYaw, INPUT_CONTROL.yaw, IMU.YawRaw);
        }
        if(MODE == AngleMode) // Needs tests
        {
            PID_calculate(&PIDAngleRoll, INPUT_CONTROL.roll, IMU.RollKal); 
            PID_calculate(&PIDAnglePitch, INPUT_CONTROL.pitch, IMU.PitchKal);

            PID_calculate(&PIDRateRoll, PIDAngleRoll.output, MPU6500.fGyroX);
            PID_calculate(&PIDRatePitch, PIDAnglePitch.output, MPU6500.fGyroY);
            PID_calculate(&PIDYaw, INPUT_CONTROL.yaw, IMU.YawRaw);
        }

        // limit throttle for other calculations to balance quadcopter
        INPUT_CONTROL_LimitThrottle(&INPUT_CONTROL);
        // caluclations for mototrs
        MMA_calculateOutput(&MMA, PIDRateRoll.output * MIX_mulitplier , PIDRatePitch.output * MIX_mulitplier , INPUT_CONTROL.throttle, PIDYaw.output * MIX_mulitplier );
        MMA_LimitOutput(&MMA);
        //printf("MMA LB = %f, MMA LF = %f, MMA RB = %f, MMA RF = %f ", MMA.motorLB, MMA.motorLF, MMA.motorRB, MMA.motorRF);
        ONESHOT_CalculateOutput(&ONESHOT, &MMA);
        //printf("SetPitch = %f, SetRoll = %f, IMUpitch = %f, IMUroll = %f, PIDpitchoutput = %f, PIDrolloutput = %f, OS LB = %d, OS LF = %d, OS RB = %d, OS RF = %d \n", INPUT_CONTROL.pitch, INPUT_CONTROL.roll, IMU.PitchKal, IMU.RollKal, PIDAnglePitch.output, PIDAngleRoll.output, ONESHOT.fillLB, ONESHOT.fillLF, ONESHOT.fillRB, ONESHOT.fillRF); 
        printf("ONESHOT LB = %d; ONESHOT LF = %d; ONESHOT RB = %d; ONESHOT RF = %d\n", ONESHOT.fillLB, ONESHOT.fillLF, ONESHOT.fillRB, ONESHOT.fillRF);
        //printf("imu.rollkal = %f, PIDRoll.output = %f, imu.pitchkal = %f, PIDPitch.currError = %f, PIDPItch.output = %f, ONESHOT LB = %d; ONESHOT LF = %d; ONESHOT RB = %d; ONESHOT RF = %d\n",IMU.RollKal,PIDRateRoll.output, IMU.PitchKal, PIDRatePitch.currError, PIDRatePitch.output, ONESHOT.fillLB, ONESHOT.fillLF, ONESHOT.fillRB, ONESHOT.fillRF);
        //printf("Yaw setpoint = %f, Yaw IMU = %f yaw PID = %f\n",INPUT_CONTROL.yaw, IMU.YawRaw, PIDYaw.output);
        if(LOG == LogsOff){
            ONESHOT_writeMotors(&ONESHOT);
        }
        // checkLoop(&LOOP_TIME); 
        // end of loop to match 250Hz
        endLoop(&LOOP_TIME);
    }
    // get input from mpu6500 
    // IMU calculation 
    // CRSF protocol reading 
    // Input control
    // Ask we are armed if not we want to reset earlier values 
    // PID calculations 
    // mma calculations 
    // oneshot protocol to power motors 
    // check how much time have passed to see if are good with timings 
    // wait for loop time to reach 250hz 
    

    return 0;
}
