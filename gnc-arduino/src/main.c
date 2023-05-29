#include <stdio.h>
#include <stdlib.h>
#include "gnc.h"
#include "mpu9250.h" //install zip file from bolder github page: https://github.com/bolderflight/invensense-imu
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

/**** PARAMETERS *****/
#define PID_KP  2.0f // proportional gain
#define PID_KI  0.5f // integral gain
#define PID_LIM_MIN -0.4f // minimum motor positon (turns)
#define PID_LIM_MAX  0.4f // maximum motor position (turns)
#define PID_INTEGRAL_LIM_MIN -5.0f // min integral term
#define PID_INTEGRAL_LIM_MAX  5.0f // max integral term
#define TIME_STEP 0.01f

int main()
{

    /********** INITIALIZE SERIAL COMMS AND PERIPHERALS ************/

    /* MPU9250 object */
    bfs::Mpu9250 imu;

    /* ODrive arduino */
    HardwareSerial& odrive_serial1 = Serial1;
    HardwareSerial& odrive_serial2 = Serial2;
    HardwareSerial& odrive_serial3 = Serial3;
    HardwareSerial& odrive_serial4 = Serial4;
    ODriveArduino odrive1(odrive_serial1);
    ODriveArduino odrive2(odrive_serial2);
    ODriveArduino odrive3(odrive_serial3);
    ODriveArduino odrive4(odrive_serial4);

    Serial.begin(115200);
    odrive_serial1.begin(115200);
    odrive_serial2.begin(115200);
    odrive_serial3.begin(115200);
    odrive_serial4.begin(115200);
    while(!Serial) {}
    Wire.begin();
    Wire.setClock(400000);
    imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
    if (!imu.Begin()) {
        while(1) {}
    }
    if (!imu.ConfigSrd(19)) {
        while(1) {}
    }

    /**************** INITIALIZE CONTROLLER AND STATES *******************/
    PIDController pid = {TIME_STEP, PID_KP, PID_KI, PID_LIM_MIN, PID_LIM_MAX, PID_INTEGRAL_LIM_MIN, PID_INTEGRAL_LIM_MAX,0.0f,0.0f,0.0f};
    Attitude atd = {TIME_STEP,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    float setpoint_theta = 0.0f; // setpoint for pitch
    float setpoint_psi = 0.0f; // setpoint for yaw
    float X_accel = 0.0f;
    float Y_accel = 0.0f;
    float Z_accel = 0.0f;
    float P = 0.0f; // roll rate abt body x axis wrt intertial frame
    float Q = 0.0f; // pitch rate abt body y axis wrt intertial frame
    float R = 0.0f; // yaw rate abt body z axis wrt intertial frame
    float theta_out = 0; // control output (motor position) for stabilizing pitch/theta
    float psi_out = 0; // control output (motor position) for stabilizati yaw/psi
    int iteration = 0;

    delay(TIME_STEP*1000.0);

    /********* ENTER CONTROL LOOP **********/
    while(1) {

        iteration += 1;
        
        /* Read IMU values */
        // apply moving average filter
        if (imu.Read()) {
            X_accel = imu.accel_x_mps2();
            Y_accel = imu.accel_y_mps2();
            Z_accel = imu.accel_z_mps2();
            P = imu.gyro_x_radps();
            Q = imu.gyro_y_radps();
            R = imu.gyro_z_radps();
        }

        /* Update attitude (euler angles) using rate gyro measurements */
        Attitude_Update(&atd,P,Q,R);

        /* Calculate control outputs for pitch and yaw stabilization */
        float measured_theta = atd.theta;
        theta_out = PID_Controller_Update(&pid, setpoint_theta, measured_theta);
        float measured_psi = atd.psi;
        psi_out = PID_Controller_Update(&pid, setpoint_psi, measured_psi);

        /* Send position cmds */
        odrive1.SetPosition(0, theta_out);
        odrive2.SetPosition(0, theta_out);
        odrive3.SetPosition(0, psi_out);
        odrive4.SetPosition(0, psi_out);

        Serial.println("Iteration %d: %f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",iteration,pid.out,atd.phi,atd.theta,atd.psi,atd.prev_phi,atd.prev_theta,atd.prev_psi,X_accel,Y_accel,Z_accel,P,Q,R);     

        delay(TIME_STEP*1000.0);

    }

    return 0;
}