#include "../include/main.hpp"
//required for eigen library, change to actual directory
//#include "../../eigen-3.4.0"

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

        //eulerangles = quat2angle(state_current); // convert estimated quats to euler angles to be used in PID

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

            Control_TVC();
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