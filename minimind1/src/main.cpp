#include "../include/main.hpp"
//required for eigen library, change to actual directory
//#include "../../eigen-3.4.0"


void Read_State_Estimate(int pin) {
    /**
     * Reads from a hardware serial pin and waits until it has obtained a complete state estimate, or just times out.
    */
   //TODO maybe shouldnt make it void and have it return a success/failure status
};

Update_Time() {};

Read_Battery() {};

Log_SD_Card() {};

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

auto PIDQuat(Eigen::Vector3f cur_angle, PIDController pid_pitch, PIDController pid_yaw) {


    // set ideal euler angle to point straight up
    Eigen::Vector3f ideal_angle = {0.0, 1.0, 0.0};

    // calculate error between cur_quat and ideal_quat
    Eigen::Vector3f error_angle;
    error_angle = cur_angle - ideal_angle;

    // pitch and yaw
    float pitch = error_angle[1];
    float yaw = error_angle[2];

    // update PID controllers with angle, return thrust moment
    float pitch = PIDController_Update(&pid_pitch, 0, pitch);
    float yaw = PIDController_Update(&pid_yaw, 0, yaw);

    return std::make_pair(pitch, yaw);

}



Control_TVC(Eigen::Vector3f eulerAngles) {

    float [pitch, yaw] = PIDQuat( eulerAngles, pid_pitch, pid_yaw);

    // control servos here
    

};

void setup() {

    //TODO start global timer

    // Begin serial communication
    Serial.begin(115200);
    while(!Serial) {}

    // Detect sensors
    while (!mpu.begin && !bmp.begin) {
        if (!mpu.begin()) {
            Serial.println("Failed to find MPU6050 chip");
        }
        if (!bmp.begin()) {
            Serial.println("Failed to find BMP280 chip");
        }
    }
    Serial.println("MPU6050 Found!");
    Serial.println("BMP280 Found!");

    // Configure sensors
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    Serial.print("Accelerometer range set to: ");
    Serial.println(mpu.getAccelerometerRange());

    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.print("Gyro range set to: ");
    Serial.println(mpu.getGyroRange());

    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.print("Filter bandwidth set to: ");
    Serial.println(mpu.getFilterBandwidth());

    // default bmp settings
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    servo_pitch.attach(); //TODO: fill this out
    servo_yaw.attach(); //TODO: fill this out

    StateMachine_t state_machine = Armed; // initialize state machine
    RocketState_t state_vector = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // initialize rocket state

    int is_first_iteration = 1;

}


float quatToEuler(Eigen::Vector4f q) {

    float roll = atan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]*q[1] + q[2]*q[2]));
    float pitch = asin(2*(q[0]*q[2] - q[3]*q[1]));
    float yaw = atan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]*q[2] + q[3]*q[3]));

    Eigen::Vector3f output = {roll, pitch, yaw};
    return output;
}

void loop() {

    // Read sensor
    sensors_event_t raw_accelo, raw_gyro, raw_pressure;
    mpu.getEvent(&raw_accelo, &raw_gyro);
    bmp_pressure->getEvent(&raw_pressure);
    double altitude = 44330 * (1.0 - pow( (pressure_event.pressure/100) / seaLevelhPa, 0.1903)); // TODO figure out formula
    
    // Execute LKF
    if (is_first_iteration) {
        double seaLevelhpa = pressure_event.pressure; // set sea lvl pressure at gnd

        update_state_transition(raw_gyro.gyro.x,raw_gyro.gyro.y,raw_gyro.gyro.z);
        predict_state();
        predict_covariance();

        is_first_iteration = 0;
    } else {
        measurement = state_current;
        update_state();
        update_covariance();

        eulerAngles = quatToEuler(state_current); // convert estimated quats to euler angles to be used in PID

        update_state_transition(raw_gyro.gyro.x,raw_gyro.gyro.y,raw_gyro.gyro.z);
        predict_state();
        predict_covariance();
    }
    state_prev = state_future; // not really the "previous state", its the "previously predicted state"
    covariance_prev = covariance_future;

    // Enter state machine
    switch(StateMachine) {

        // ignore this comment vvv
        // if still in static estimation, calling Read_State_Estimate() will simply not do anything
        // ^ during static estimation SE teensy sends null values thru serial

        case (Armed):

            if (state_vector.accel_z > 10.0 && state_vector.pos_z > 1.0) {
                
                // get current time
                Read_State_Estimate()
                while (state_vector.accel_z > 10.0 && state_vector.pos_z > 1.5) {
                    delay(0.1);
                    time += 0.1;
                    Read_State_Estimate(&state_vector);
                }
                if (time > 1) {
                    StateMachine = FastAscent;
                }
            }
            break;

        case (FastAscent):
            delay(TIME_STEP); //at least longer than time response

            Control_TVC(&eulerAngles);
            Fast_Data_Log(&state_vector, /*motor posn*/);

            if (state_vector.accel_z < 2.0) {
                delay(0.1);
                Read_State_Estimate(&state_vector);
                if (state_vector.accel_z < 2.0) {
                    StateMachine = SlowAscent;
                }
            }
            break;

        case (SlowAscent):
            float previous_altitude = state_vector.pos_z;
            delay(1); //TODO obviously not keeping this, will have to implement a timer instead
            Read_State_Estimate(&state_vector);
            if (state_vector.pos_z < previous_altitude) {

                //TODO timestamp and log apogee event to sd card
                StateMachine = FreeFall;
            }
            break;

        case (FreeFall):
            if (state_vector.accel_z < value) {
                //TODO timestamp and log pyro fire event to sd card
                StateMachine = Chute;
            } else {

                if (state_vector.pos_z < 5.0) {

                    delay(0.1);
                    Read_State_Estimate(&state_vector);
                    if (state_vector.pos_z < 5.0) {
                        StateMachine = Landed;
                    }

                }
        
            }
            break;

        case (Chute):
            if (state_vector.pos_z < 5.0) {
                delay(0.1);
                Read_State_Estimate(&state_vector);
                if (state_vector.pos_z < 5.0) {
                    StateMachine = Landed;
                }
            }
            break;

        case (Landed):
            //TODO timestamp and log landing to sd card
            // TODO: close SD card
            break;

        default:
            break;

    }

}