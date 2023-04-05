#include <stdio.h>
#include <stdlib.h>
#include "gnc.h"
#include "mpu9250.h" //install zip file from bolder github page: https://github.com/bolderflight/invensense-imu
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>


/**** PARAMETERS *****/

/* Timer parameter */
#define TIMER 0.0f

/* PID controller parameters */
#define PID_KP  2.0f
#define PID_KI  0.5f
#define PID_LIM_MIN -0.5f // minimum motor positon (turns)
#define PID_LIM_MAX  0.5f // maximum motor position (turns)
#define PID_INTEGRAL_LIM_MIN -5.0f
#define PID_INTEGRAL_LIM_MAX  5.0f
#define TIME_STEP 0.01f

/********* IMU **********/

/* MPU9250 object */
bfs::Mpu9250 imu;

/****** ODRIVE ARDUINO *******/
HardwareSerial& odrive_serial1 = Serial1;
HardwareSerial& odrive_serial2 = Serial2;
HardwareSerial& odrive_serial3 = Serial3;
HardwareSerial& odrive_serial4 = Serial4;
ODriveArduino odrive1(odrive_serial1);
ODriveArduino odrive2(odrive_serial2);
ODriveArduino odrive3(odrive_serial3);
ODriveArduino odrive4(odrive_serial4);

int main()
{

    /* ODRIVE */
    Serial.begin(115200);
    odrive_serial1.begin(115200);
    odrive_serial2.begin(115200);
    odrive_serial3.begin(115200);
    odrive_serial4.begin(115200);

    /* Initialize PID controller and attitude heading reference system */
    PIDController pid = {TIME_STEP, PID_KP, PID_KI, PID_LIM_MIN, PID_LIM_MAX, PID_INTEGRAL_LIM_MIN, PID_INTEGRAL_LIM_MAX,0.0f,0.0f,0.0f};
    Attitude atd = {TIME_STEP,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f}

    float setpoint = 0.0f; // Zero radians
    float P = 0.0f;
    float Q = 0.0f;
    float R = 0.0f;

    while(1) {
        
        if (imu.Read()) {
            P = imu.gyro_x_radps();
            Q = imu.gyro_y_radps();
            R = imu.gyro_z_radps();
        }       

        Attitude_Update(&atd,P,Q,R);

        float measured_theta = atd.theta;

        PID_Controller_Update(&pid, setpoint, measured_theta);
        
        odrive1.SetPosition(0, pid.output);
        odrive2.SetPosition(0, pid.output);

        TIMER += TIME_STEP;

    }

    return 0;
}