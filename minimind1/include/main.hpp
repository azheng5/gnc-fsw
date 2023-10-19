#include <stdio.h>
#include <stdlib.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
// #include <HardwareSerial.h>

///////// main.h /////////
Adafruit_MPU6050 mpu; // initialize imu
Adafruit_BMP280 bmp; // initialize bmp
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();



typedef enum StateMachine_t {
    Armed=0, // arm TVC?
    FastAscent=1, // enable TVC
    SlowAscent=2, // disable TVC
    FreeFall=3,
    Chute=4,
    Landed=5
} StateMachine_t;

typedef struct RocketState_t {
    double pos_x; // m
    double pos_y;
    double pos_z;
    double vel_x; // m/s
    double vel_y;
    double vel_z;
    double pitch; // rad
    double yaw;
    double roll;
    double pitch_rate // rad/s
    double roll_rate;
    double yaw_rate;
} RocketState_t;

Log_SD_Card();
// Log 12 rocket states, timestamp, 2 servo angle cmds, 3 raw gyro, 3 raw accelo, 1 raw altitude

/////////// PID.h ////////////////

typedef struct {
    /* Time of each iteration */
    float time_step;

    /* Controller gains */
	float Kp; // proportional term
	float Ki; // integral term
    float Kd; // derivative term

    /* Control limits */
	float lim_min; // min controller output limit
	float lim_max; // max controller output limit
	float integral_lim_min; // min integral output limit
	float integral_lim_max; // max integral output limit

    /* Integrator values */
	float integral;
	float prev_error; // for calculating integral term

    float derivative;

    /* Controller outputs */
	float out;
} PIDController;


#define PID_KP  1.0 // proportional gain
#define PID_KI  0.0 // integral gain
#define PID_KD  1.0 // integral gain
#define PID_INTEGRAL_MIN -5.0 // min integral term (Nm)
#define PID_INTEGRAL_MAX 5.0 // max integral term (Nm)
#define PID_TIME_STEP 0.05 // delay (longer than actuation response)
#define SERVO1_MIN -10.0 // minimum motor positon (degrees)
#define SERVO1_MAX  10.0 // maximum motor position
#define SERVO2_MIN -10.0 // minimum motor positon
#define SERVO2_MAX  10.0 // maximum motor position

#define MOMENT_ARM 0.3 // (m)

Servo servo_pitch;
Servo servo_yaw;

PIDController pid_pitch;
PIDController pid_yaw;

Control_TVC();

PIDController_Update();

PIDQuat();

/////////// LKF.h /////////////////
#define LKF_TIME_STEP 0.01
#define DIM_STATE 4 // quat
#define DIM_MEASUREMENT 4 // estimated quat from prev iteration (NOT gyro)
#define DIM_INPUT 2 // control inputs (pitch/yaw gimbal angles)

Eigen::MatrixXd::Identity(DIM_STATE, DIM_STATE) identity;

Eigen::VectorXd state_current(DIM_STATE); // x_n,n
Eigen::MatrixXd covariance_current(DIM_STATE,DIM_STATE); // current estimate covariance P_n,n
Eigen::MatrixXd state_transition(DIM_STATE,DIM_STATE); // state transition F
Eigen::MatrixXd process_noise(DIM_STATE,DIM_STATE); // process noise covariance Q
Eigen::VectorXd control_input(DIM_INPUT); // control input u_n
Eigen::MatrixXd control_matrix(DIM_STATE,DIM_INPUT); // control matrix G
Eigen::MatrixXd observation(DIM_MEASUREMENT,DIM_STATE); // observation matrix H
Eigen::VectorXd measurement(DIM_MEASUREMENT);
Eigen::MatrixXd measurement_covariance(DIM_MEASUREMENT,DIM_MEASUREMENT); // measurement covariance R

Eigen::VectorXd state_prev(DIM_STATE); // previously predicted state x_n,n-1
Eigen::MatrixXd covariance_prev(DIM_STATE,DIM_STATE); //previously predicted covariance matrix P_n,n-1

Eigen::VectorXd state_future(DIM_STATE); // predicted future state x_n+1,n
Eigen::MatrixXd covariance_future(DIM_STATE,DIM_STATE); // predicted future covariance P_n+1,n

Eigen::MatrixXd kalman_gain(DIM_STATE,DIM_MEASUREMENT);

state_current << 1,0,0,0; //TODO confirm this is 0 angle
covariance_current = identity;
// state transition matrix gets updated each iteration
process_noise << 0.001*identity;
// control input gets updated with each iteration
// control_matrix << ???? // TODO figure this out
observation << 1,0,0,0,
               0,1,0,0,
               0,0,1,0,
               0,0,0,1;
measurement_covariance << 10*identity;



Eigen::MatrixXd compute_kalman_gain() {

    kalman_gain = (covariance_prev*observation.transpose()) * (( (observation*covariance_prev)*observation.transpose() + measurement_covariance).inverse());
    return kalman_gain;

};

Eigen::VectorXd update_state() {
    
    // compute kalman gain
    Eigen::MatrixXd kalman_gain(DIM_STATE,DIM_MEASUREMENT);
    kalman_gain = compute_kalman_gain(covariance_prev);

    // compute current state
    state_current = state_prev + kalman_gain*(measurement - observation*state_prev);
    
    return state_current;
};

Eigen::MatrixXd update_covariance() {
    
    // compute kalman gain
    Eigen::MatrixXd kalman_gain(DIM_STATE,DIM_MEASUREMENT);
    kalman_gain = compute_kalman_gain(covariance_prev);

    // compute current covariance
    covariance_current = (identity - kalman_gain * observation) * covariance_prev * (identity - kalman_gain * observation).transpose() + kalman_gain * measurement_covariance * kalman_gain.transpose();

    return covariance_current;

};

Eigen::MatrixXd predict_state() {

    state_future = state_transition * state_current + control_matrix * control_input;
    return state_future;
};

Eigen::MatrixXd predict_covariance() {
    covariance_future = (state_transition*covariance_current)*state_transition.transpose() + process_noise;
    return covariance_future;
};


Eigen::MatrixXd update_state_transition(double w1, double w2, double w3) {
    /**
     * complementary fn for predict_state() and predict_covariance()
     * state transition matrix needs to be redefined each iteration bc its a fn of rate gyro values
     * special fn applicable only for quaternion kinematics rn
    */

   Eigen::Matrix4f omega;
    omega <<  0, -w1, -w2, -w3,
              w1,   0,  w3, -w2,
              w2, -w3,   0,  w1,
              w3,  w2, -w1,   0;
    state_transition = identity + LKF_TIME_STEP*0.5*omega;

    return state_transition;
}