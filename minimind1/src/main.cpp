#include "../include/main.hpp"

void Read_State_Estimate(int pin) {
    /**
     * Reads from a hardware serial pin and waits until it has obtained a complete state estimate, or just times out.
    */
   //TODO maybe shouldnt make it void and have it return a success/failure status
};

Fire_Pyro() {
    /**
     * 
    */

   //TODO when firing pyro channels for the 2nd or 3rd time increase the voltage?
};

Update_Time() {};

Update_LED() {};

Read_Battery() {};

Log_SD_Card() {};

Control_TVC() {};

// Check_GPS_Lock() {};

void setup() {

    servo_pitch.attach(); //TODO: fill this out
    servo_yaw.attach(); //TODO: fill this out

    StateMachine_t state_machine = Armed; // initialize state machine
    RocketState_t state_vector = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f}; // initialize rocket state

    //TODO start timer

}

void loop() {

    Read_State_Estimate(&state_vector);
    Read_Battery();
    Update_LED();

    //TODO in general do more condition checks for better redundancy
    switch(StateMachine) {

        // if still in static estimation, calling Read_State_Estimate() will simply not do anything
        case (Armed):
            Read_Battery();

            if (state_vector.accel_z > 10.0f) {
                delay(0.1); //TODO figure out how much more useful it will be to use timer to enable the rest of the program to execute while the delay is occurring
                Read_State_Estimate(&state_vector);
                if (state_vector.pos_z > 10.0f) {
                    StateMachine = FastAscent;
                }
            }
            break;

        case (FastAscent):
            delay(TIME_STEP); //at least longer than time response
            Control_TVC();

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
            // currently fire pyro will just be called over and over again until it either succeeds or landing detected
            int pyro_status = Fire_Pyro(); // returns whether pyro fired successfully
            if (pyro_status) {
                //TODO timestamp and log pyro fire event to sd card
                StateMachine = Chute;
            } else {

                if (state_vector.pos_z < 3.0f) {

                    delay(0.1);
                    Read_State_Estimate(&state_vector);
                    if (state_vector.pos_z < 3.0f) {
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