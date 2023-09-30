#include <stdio.h>
#include <stdlib.h>
#include <Servo.h>

#include <HardwareSerial.h>

// PID variables
#define PID_KP  1.0f // proportional gain
#define PID_KI  0.0f // integral gain
#define PID_KD  1.0f // integral gain
#define PID_INTEGRAL_MIN -5.0f // min integral term (Nm)
#define PID_INTEGRAL_MAX 5.0f // max integral term (Nm)
#define PID_TIME_STEP 0.05f // delay (longer than actuation response)
#define SERVO1_MIN -10.0f // minimum motor positon (degrees)
#define SERVO1_MAX  10.0f // maximum motor position
#define SERVO2_MIN -10.0f // minimum motor positon
#define SERVO2_MAX  10.0f // maximum motor position

#define MOMENT_ARM 0.3f //

// LKF variables
#define LKF_TIME_STEP

Servo servo_pitch;
Servo servo_yaw;

typedef enum StateMachine_t {
    Armed=0, // arm TVC?
    FastAscent=1, // enable TVC
    SlowAscent=2, // disable TVC
    FreeFall=3,
    Chute=4,
    Landed=5
} StateMachine_t;

typedef struct RocketState_t {

    float pos_x; // m
    float pos_y;
    float pos_z;
    float vel_x; // m/s
    float vel_y;
    float vel_z;
    float accel_x; // m/s^2
    float accel_y;
    float accel_z;

    float pitch;
    float yaw;
    float roll;
    float pitch_rate
    float roll_rate;
    float yaw_rate;
} RocketState_t;

void Read_State_Estimate(int pin);

Read_Battery();

Log_SD_Card();

Control_TVC();