#include <stdio.h>
#include <stdlib.h>

/****** PID CONTROLLER *******/

/* Global struct for the PID controller */
typedef struct {
    /* Time of each iteration */
    float time_step;

    /* Controller gains */
	float Kp; // proportional term
	float Ki; // integral term

    /* Control limits */
	float lim_min; // min controller output limit
	float lim_max; // max controller output limit
	float integral_lim_min; // min integral output limit
	float integral_lim_max; // max integral output limit

    /* Integrator values */
	float integral;
	float prev_error; // for calculating integral term

    /* Controller outputs */
	float out;
} PIDController;

/* Used to update PID controller each iteration*/
float PIDController_Update(PIDController* pid, float setpoint, float measured_state);


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