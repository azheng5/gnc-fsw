#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <Servo.h>

#include <HardwareSerial.h>

#define PID_KP  2.0f // proportional gain
#define PID_KI  0.5f // integral gain
#define PID_KD  0.5f // integral gain
#define PID_INTEGRAL_MIN -5.0f // min integral term
#define PID_INTEGRAL_MAX  5.0f // max integral term
#define TIME_STEP 0.05f // delay (longer than actuation response)
// do i want to define servo limits tho...
#define SERVO1_MIN -0.4f // minimum motor positon (turns)
#define SERVO1_MAX  0.4f // maximum motor position (turns)
#define SERVO2_MIN -0.4f // minimum motor positon (turns)
#define SERVO2_MAX  0.4f // maximum motor position (turns)

Servo servo_pitch;
Servo servo_yaw;
//TODO: initialize GPS

typedef enum StateMachine_t {
    Armed=0, // arm TVC?
    FastAscent=1, // enable TVC
    SlowAscent=2, // disable TVC
    FreeFall=3,
    Chute=4,
    Landed=5
} StateMachine_t;

typedef struct RocketState_t {
    float pos_z; // m
    float vel_z; // m/s
    float accel_z; // m/s^2
    float roll; // rad
    float roll_rate;
    float roll_accel;
    float pitch;
    float pitch_rate;
    float pitch_accel;
    float yaw;
    float yaw_rate;
    float yaw_accel;
} RocketState_t;

void Read_State_Estimate(int pin);

Fire_Pyro();

Update_Time();

Update_LED();

Read_Battery();

Log_SD_Card();

Control_TVC();

Check_GPS_Lock();