#include <stdio.h>
#include <stdlib.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <math.h>

#include <BasicLinearAlgebra.h>

#define ARMED 0
#define FASTASCENT 1
#define SLOWASCENT 2
#define FREEFALL 3
#define CHUTE 4
#define LANDED 5

// typedef enum StateMachine_t {
//     Armed=0,
//     FastAscent=1, // enable TVC
//     SlowAscent=2, // disable TVC
//     FreeFall=3,
//     Chute=4,
//     Landed=5
// } StateMachine;

// NOT a state vector
typedef struct RocketState_t {
    float pos_x; // m
    float pos_y;
    float pos_z;
    float vel_x; // m/s
    float vel_y;
    float vel_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float pitch; // rad
    float yaw;
    float roll;
    float pitch_rate; // rad/s
    float roll_rate;
    float yaw_rate;
} RocketState;

//Log_SD_Card();
// Log 12 rocket states, timestamp, 2 servo angle cmds, 3 raw gyro, 3 raw accelo, 1 raw altitude
