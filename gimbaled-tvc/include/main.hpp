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
#define SERVO1_MIN -0.4f // minimum motor positon (turns)
#define SERVO1_MAX  0.4f // maximum motor position (turns)
#define SERVO2_MIN -0.4f // minimum motor positon (turns)
#define SERVO2_MAX  0.4f // maximum motor position (turns)

Servo servo_pitch;
Servo servo_yaw;
//TODO: initialize GPS
//TODO: initialize GPS

struct {
    bool Idle;
    bool Armed;
    bool FastAscent;
    bool SlowAscent;
    bool FreeFall;
    bool Chute;
    bool Landed;
} StateMachine;

Read_State_Estimate();

Fire_Pyro();

Update_Time();

Update_LED();

Read_Battery();

Log_SD_Card();

Control_TVC();

Check_GPS_Lock();