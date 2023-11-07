// #define PID_KP  1.0f // proportional gain
// #define PID_KI  0.0f // integral gain
// #define PID_KD  1.0f // integral gain
// #define PID_INTEGRAL_MIN -5.0f // min integral term (Nm)
// #define PID_INTEGRAL_MAX 5.0f // max integral term (Nm)
// #define PID_TIME_STEP 0.05f // delay (longer than actuation response)
#define PITCH_SERVO_MIN -10.0f // minimum motor positon (degrees)
#define PITCH_SERVO_MAX 10.0f // maximum motor position
#define YAW_SERVO_MIN -10.0f // minimum motor positon
#define YAW_SERVO_MAX 10.0f // maximum motor position

// #define MOMENT_ARM 0.3f // distance from cg to gimbal (meters)

// define servo objects
Servo servo_pitch;
Servo servo_yaw;

// will run 2 of these for pitch and yaw
typedef struct PIDController {
  /* Time of each iteration */
  // not global bc other algorithms may require unique time steps
  float time_step; // variable time step

  /* Controller gains */
	float Kp; // proportional term
	float Ki; // integral term
  float Kd; // derivative term

  /* Control limits */
	float lim_min; // min controller output limit (servo)
	float lim_max; // max controller output limit (servo)
	float integral_lim_min; // min integral output limit
	float integral_lim_max; // max integral output limit

  /* Proportional stuff */
  //

  /* Integrator stuff */
	float integral;
	float prev_error; // for calculating integral term

  /* Derivative stuff */
  float derivative;

  /* Controller output */
  float out;

} PIDController;

// Generalized PID controller
float PID_Controller_Update(PIDController *pid, float setpoint, float measured_state) {

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

// auto PIDQuat(BLA::Matrix<3,1> cur_angle, PIDController pid_pitch, PIDController pid_yaw) -> std::pair<float,float> {
//   // set ideal euler angle to point straight up
//   BLA::Matrix<3,1> ideal_angle = {0.0f,
//                                   1.0f,
//                                   0.0f};

//   // calculate error between cur_quat and ideal_quat
//   BLA::Matrix<3,1> error_angle;
//   error_angle = cur_angle - ideal_angle;

//   // pitch and yaw
//   float pitch = error_angle(1,0);
//   float yaw = error_angle(2,0);

//   // update PID controllers with angle, return thrust moment
//   float pitch_updated = PIDController_Update(&pid_pitch, 0.0f, pitch);
//   float yaw_updated = PIDController_Update(&pid_yaw, 0.0f, yaw);

//   return std::make_pair(pitch_updated, yaw_updated);

// }

// mega PID function
// takes in current state estimate (2 euler angles)
// void Control_TVC(pitch,yaw) {

//     // auto result = PIDQuat( eulerAngles, pid_pitch, pid_yaw);

//     pitch_controller->time_step = millis()/1000.0 - prev_time;
//     yaw_controller->time_step = millis()/1000.0 - prev_time;
//     prev_time = millis();

//     float pitch_cmd = PID_Controller_Update(&pitch_controller, 0.0f, pitch);
//     float yaw_cmd = PID_Controller_Update(&yaw_controller, 0.0f, yaw);

//     // send actuation cmd
//     servo_pitch.write(pitch_cmd);
//     servo_yaw.write(yaw_cmd);

// };

// BLA::Matrix<3,1> quatToEuler(BLA::Matrix<4,1> q) {

//     float roll = atan2(2*(q(0,0*q(1,0) + q(2,0)*q(3,0)), 1 - 2*(q(1,0)*q(1,0)+ q(2,0)*q(2,0)));
//     float pitch = asin(2*(q(0,0)*q(2,0) - q(3,0)*q(1,0)));
//     float yaw = atan2(2*(q(0,0)*q(3,0) + q(1,0)*q(2,0)), 1 - 2*(q(2,0)*q(2,0) + q(3,0)*q(3,0)));

//     BLA::Matrix<3,1> output = {roll,pitch,yaw};
//     return output;
// }