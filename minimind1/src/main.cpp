#include "../include/main.hpp"
//required for eigen library, change to actual directory
//#include "../../eigen-3.4.0"

void Read_State_Estimate(int pin) {
    /**
     * Reads from a hardware serial pin and waits until it has obtained a complete state estimate, or just times out.
    */
   //TODO maybe shouldnt make it void and have it return a success/failure status
};
/**
matrixXd kalman_gain(matrixXd covariance_prev) {
    kalman_gain = (covariance_prev*observation.transpose())*((observation*covariance_prev*observation.transpose() + measurement_covariance).inverse());
    return kalman_gain;
};

void update_state(vectorXd state, vectorXd state_prev, matrixXd covariance_prev, matrixXd measurement) {
    K = kalman_gain(covariance_prev);
    state_crrent = state_prev + K*(measurement - observation*state_prev);
};

void update_covariance(matrixXd covariance_prev) {
    K = kalman_gain(covariance_prev);
    covariance = (identity - K*observation)*covariance_prev*(identity - K*observation).transpose() + K*measurement_covariance*K.transpose();
};

void predict_state(matrixXd state_current) {
    if (control_input && control_matrix) {
        state_future = state_transition*state_current + control_matrix*control_input;
    } else {
        state_future = state_transition*state_current;
    };
};

void predict_covariance(matrixXd covariance_current, matrixXd state_transition, matrixXd process_noise) {
    covariance_future = (state_transition*covariance_current)*state_transition.transpose() + process_noise;
};
**/

Update_Time() {};

Read_Battery() {};

Log_SD_Card() {};

Control_TVC() {};

void setup() {

    // start global timer

    servo_pitch.attach(); //TODO: fill this out
    servo_yaw.attach(); //TODO: fill this out

    StateMachine_t state_machine = Armed; // initialize state machine
    RocketState_t state_vector = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f}; // initialize rocket state

    //TODO start timer

}

void loop() {

    Read_State_Estimate(&state_vector);
    Read_Battery();

    //TODO in general do more condition checks for better redundancy
    switch(StateMachine) {

        // if still in static estimation, calling Read_State_Estimate() will simply not do anything
        // ^ during static estimation SE teensy sends null values thru serial
        case (Armed):

            if (state_vector.accel_z > 10.0f && state_vector.pos_z > 1.0f) {
                
                // get current time
                Read_State_Estimate()

                // get later time

                // difference

                if () {



                }

                while (state_vector.accel_z > 10.0f && state_vector.pos_z > 1.5f) {
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

            Control_TVC(&state_vector);
            Fast_Data_Log(&state_vector, /*motor posn*/);

            if (state_vector.accel_z < 2.0f) {
                delay(0.1);
                Read_State_Estimate(&state_vector);
                if (state_vector.accel_z < 2.0f) {
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

                if (state_vector.pos_z < 5.0f) {

                    delay(0.1);
                    Read_State_Estimate(&state_vector);
                    if (state_vector.pos_z < 5.0f) {
                        StateMachine = Landed;
                    }

                }
        
            }
            break;

        case (Chute):
            if (state_vector.pos_z < 5.0f) {
                delay(0.1);
                Read_State_Estimate(&state_vector);
                if (state_vector.pos_z < 5.0f) {
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