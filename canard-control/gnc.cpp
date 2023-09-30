#include "gnc.h"
#include <iostream>
#include "Eigen/Dense"

using namespace std;

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

    /* Calculate derivative term */
    pid->derivative = pid->Kd * (error - pid->prev_error) / pid->time_step;


    /* Compute output and apply control output limits */
    pid->out = proportional + pid->integral + pid->derivative;
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


// function that uses Eigen quaternion library to convert euler angles to quaternions
void Euler2Quat(float phi, float theta, float psi, Eigen::Quaterniond q_eigen) {

    q_eigen = Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ());

}   

// return thrust T according to simulated thrust curve
auto calcSimThrust(float time_elapsed) {

    float T = 0;

    // thrust curve calcs

    return T;

}


auto momentToAngle(float moment_z, float moment_y, float d, float T) {

    
    float theta = asin((moment_z * d) / T);
    float phi = asin((moment_y * d) / T);

    return std::make_pair(theta, phi);

}

// PID function that takes in kalman quaternion and outputs thrust moment in X and Y based on gains
auto PIDQuat(Eigen::Quaterniond cur_quat, PIDController pidmomentz, PIDController pidmomenty) {


    // set ideal quat to point straight up
    Eigen::Quaterniond ideal_quat = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());

    // calculate error between cur_quat and ideal_quat
    Eigen::Quaterniond error_quat;
    error_quat = cur_quat * ideal_quat.inverse();


    Eigen::Vector3d error_euler = error_quat.toRotationMatrix().eulerAngles(0, 1, 2);

    // pitch and yaw
    float pitch = error_euler[1];
    float yaw = error_euler[2];

    // update PID controllers with angle, return thrust moment
    float moment_z = PIDController_Update(&pidmomentz, 0, pitch);
    float moment_y = PIDController_Update(&pidmomenty, 0, yaw);

    return std::make_pair(moment_z, moment_y);

}


PIDController pidmomentz = PIDController();
PIDController pidmomenty = PIDController();
float d = 0.5;

int main() {

    // initialize quaternion that would be output from kalman filter
    Eigen::Quaterniond q_eigen;
    Euler2Quat(4, 0.2, 0.3, q_eigen);

    auto [moment_z, moment_y] = PIDQuat(q_eigen, pidmomentz, pidmomenty);

    float T = calcSimThrust(0.1); 

    auto [theta, phi] = momentToAngle(moment_z, moment_y, d, T);


    // update servos with pitch and yaw outputs

    return 0;
}