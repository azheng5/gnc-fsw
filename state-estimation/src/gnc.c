#include "gnc.h"

float PIDController_Update(PIDController *pid, float setpoint, float measured_state) {

    /* Calculate error */
    float error = setpoint - measured_state;

    /* Calculate proportional term */
    float proportional = pid->Kp * error;

    /* Calculate integral term */
    pid->integral = pid->integral + 0.5f * pid->Ki * pid->time_step * (error + pid->prev_error);

	/* Apply integrator limits */
    if (pid->integral > pid->integral_lim_max) {
        pid->integral = pid->integral_lim_max;
    } else if (pid->integral < pid->integral_lim_max) {
        pid->integral = pid->integral_lim_min;
    }

    /* Compute output and apply control output limits */
    pid->out = proportional + pid->integral;
    if (pid->out > pid->lim_max) {
        pid->out = pid->lim_max;
    } else if (pid->out < pid->lim_min) {
        pid->out = pid->lim_min;
    }

	/* Store error */
    pid->prev_error = error;

	/* Return controller output */
    return pid->out;
}

float Attitude_Update(Attitude* atd, float P, float Q, float R) {

    /* Get time derivs of euler angles (NOT the same as body rates PQR) */
    float phi_dot = P + sin(atd->phi)*tan(atd->theta)*Q + cos(atd->phi)*tan(atd->theta)*R;
    float theta_dot = cos(atd->phi)*Q - sin(atd->phi)*R;
    float psi_dot = (1.0/cos(atd->theta))*sin(atd->phi)*Q + cos(atd->phi)*sec(atd->theta)*R;

    atd->phi = atd->time_step * (phi_dot) + atd->prev_phi;
    atd->theta = atd->time_step * (theta_dot) + atd->prev_theta;
    atd->psi = atd->time_step * (psi_dot) + atd->prev_psi;

    atd->prev_phi = atd->phi;
    atd->prev_theta = atd->theta;
    atd->prev_psi = atd->psi;

}