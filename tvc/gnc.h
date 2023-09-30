#include <stdio.h>
#include <stdlib.h>
#include "Eigen/Dense"

/****** PID CONTROLLER *******/

/* Global struct for the PID controller */
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

/* Used to update PID controller each iteration*/
float PIDController_Update(PIDController* pid, float setpoint, float measured_state);

// convert Euler angles to quaternions
void Euler2Quat(float phi, float theta, float psi, Eigen::Quaterniond q_eigen);

auto interpolateThrust(float time_elapsed);

auto momentToAngle(float moment_z, float moment_y, float d, float T);

auto PIDQuat(Eigen::Quaterniond cur_quat, PIDController pidmomentz, PIDController pidmomenty);

/****** ATTITUDE ******/

/* Global struct for rocket attitude, given in euler angles */
typedef struct {
    float time_step; // Needed for both pid and atd
    float phi;
    float theta;
    float psi;
    float prev_phi;
    float prev_theta;
    float prev_psi;
} Attitude;


/* Calculates the theta euler angle when given an angular rate measurement from the rate gyro*/
float Attitude_Update(Attitude *atd, float P, float Q, float R);