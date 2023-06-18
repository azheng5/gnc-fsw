#include "../include/main.hpp"

Read_State_Estimate() {};

Fire_Pyro() {};

Update_Time() {};

Update_LED() {};

Read_Battery() {};

Log_SD_Card() {};

Control_TVC() {};

Check_GPS_Lock() {};

void setup() {

    servo_pitch.attach(); //TODO: fill this out
    servo_yaw.attach(); //TODO: fill this out


}

void loop() {


    //TODO: this switch statement is pseudocode look into proper implementation of state machines in c++
    switch(StateMachine) {

        case (StateMachine.Idle):
            Read_Battery();
            if () {
                StateMachine.Idle = false;
                StateMachine.Armed = true;
            }
            break;

        case (StateMachine.Armed):
            if () {
                StateMachine.Armed = false;
                StateMachine.FastAscent = true;
            }
            break;

        case (StateMachine.FastAscent):
            Read_State_Estimate();
            delay(TIME_STEP);
            Control_TVC();
            if () {
                StateMachine.FastAscent = false;
                StateMachine.SlowAscent = true;
            }
            break;

        case (StateMachine.SlowAscent):
            Read_State_Estimate();
            if () {
                StateMachine.SlowAscent = false;
                StateMachine.FreeFall = true;
            }
            break;

        case (StateMachine.FreeFall):
            Read_State_Estimate();
            Fire_Pyro();
            if () {
                StateMachine.FreeFall = false;
                StateMachine.Chute = true;
            }
            break;

        case (StateMachine.Chute):
            Read_State_Estimate();
            if () {
                StateMachine.Chute = false;
                StateMachine.Landed = true;
            }
            break;

        case (StateMachine.Landed):
            // TODO: close SD card
            break;

        default:
            break;

    }

}