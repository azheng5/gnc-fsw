#include "main.h"
#include "filter.h"
#include "pid.h"

// initialize sensors
Adafruit_MPU6050 mpu; // initialize imu
Adafruit_BMP280 bmp; // initialize bmp
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

int is_first_iteration = 1; // flag for whether loop() is in its first iteration or not

int state_machine = ARMED; // initialize state machine
RocketState rocket_state = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f}; // initialize rocket state
RocketState prev_rocket_state = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f}; // previous rocket state

float prev_time = 0.0; // used for calculating variable time step
float time_step = 0.0;

PIDController pitch_controller = {
  0.0f, //variable time_step
  1.0f, //Kp
  0.0f, //Ki
  1.0f, //Kd
  PITCH_SERVO_MIN*(M_PI/180.0f), //lim_min (deg)
  PITCH_SERVO_MAX*(M_PI/180.0f), //lim_max (deg)
  10.0f*(M_PI/180.0f), // integral_lim_min
  10.0f*(M_PI/180.0f), // integral_lim_max
  0.0f, // integral
  0.0f, // previous error
  0.0f, //derivative
  0.0f, // output
};

PIDController yaw_controller = {
  0.0f, //variable time_step
  1.0f, //Kp
  0.0f, //Ki
  1.0f, //Kd
  YAW_SERVO_MIN*(M_PI/180.0f), //lim_min (rad)
  YAW_SERVO_MAX*(M_PI/180.0f), //lim_max (rad)
  10.0f*(M_PI/180.0f), // integral_lim_min
  10.0f*(M_PI/180.0f), // integral_lim_max
  0.0f, // integral
  0.0f, // previous error
  0.0f, //derivative
  0.0f, // output
};

int servo_pitch_zero; // clicks
int servo_yaw_zero; // clicks

void setup() {

    // Begin serial communication

    Serial.begin(115200);
    // while(!Serial) {}
    Serial.println("Began serial communication");

    // Error checking
    if (BUFFER_SIZE%2 == 0) {
      Serial.println("Assert: buffer size is not an odd number");
      while(1) {};
    }

    // Detect sensors
    while (!mpu.begin() && !bmp.begin()) {
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

    servo_pitch.attach(5); //TODO: fill this out
    servo_yaw.attach(23); //TODO: fill this out

    servo_pitch_zero = 1500;
    servo_yaw_zero = 1500;


    // Servo range of motion test (only pitch and only yaw)
    // servo_pitch.writeMicroseconds(1500);
    // delay(100);
    // for (int i=1500; i>=1500+PITCH_SERVO_MIN*(1/0.18); i--) {
    //     Serial.print("i: "); Serial.println(i);
    //     servo_pitch.writeMicroseconds(i);
    //     Serial.print("pitch1: ");
    //     Serial.println(servo_pitch.read());
    //     delay(20);
    // }

    // servo_pitch.write(1500+PITCH_SERVO_MIN*(1/0.18));
    // for (int i=1500+PITCH_SERVO_MIN*(1/0.18); i<=1500+PITCH_SERVO_MAX*(1/0.18); i+=1) {
    //     Serial.print("i: "); Serial.println(i);
    //     servo_pitch.writeMicroseconds(i);
    //     Serial.print("pitch2: ");
    //     Serial.println(servo_pitch.read());
    //     delay(20);
    // }

    // servo_pitch.write(1500+PITCH_SERVO_MAX*(1/0.18));
    // for (int i=1500+PITCH_SERVO_MAX*(1/0.18); i>=1500; i-=1) {
    //     servo_pitch.writeMicroseconds(i);
    //     Serial.print("pitch3: ");
    //     Serial.println(servo_pitch.read());
    //     delay(20);
    // }


    // servo_pitch.write(1500);
    // delay(100);
    // for (int i=1500; i>=1500+YAW_SERVO_MIN*(1/0.18); i--) {
    //     servo_yaw.write(i);
    //     Serial.print("yaw1: ");
    //     Serial.println(servo_yaw.read());
    //     delay(20);
    // }
    // for (int i=servo_yaw_zero+YAW_SERVO_MIN*(1/0.18); i<=servo_yaw_zero+YAW_SERVO_MAX*(1/0.18); i++) {
    //     servo_yaw.write(i);
    //     Serial.print("yaw2: ");
    //     Serial.println(servo_yaw.read());
    //     delay(20);
    // }
    // for (int i=servo_yaw_zero+YAW_SERVO_MAX*(1/0.18); i>=1500; i--) {
    //     servo_yaw.write(i);
    //     Serial.print("yaw3: ");
    //     Serial.println(servo_yaw.read());
    //     delay(20);
    // }

    // servo_pitch.write(servo_pitch_zero);
    // Serial.print("pitch: ");
    // Serial.println(servo_pitch.read());
    // delay(1000);
    // servo_pitch.write(servo_pitch_zero+PITCH_SERVO_MIN);
    // Serial.print("pitch: ");
    // Serial.println(servo_pitch.read());
    // delay(1000);
    // servo_pitch.write(servo_pitch_zero);
    // Serial.print("pitch: ");
    // Serial.println(servo_pitch.read());
    // delay(1000);
    // servo_pitch.write(servo_pitch_zero+PITCH_SERVO_MAX);
    // Serial.print("pitch: ");
    // Serial.println(servo_pitch.read());
    // delay(1000);
    // servo_pitch.write(servo_pitch_zero);
    // Serial.print("pitch: ");
    // Serial.println(servo_pitch.read());
    
    // // servo_yaw.write(servo_yaw_zero);
    // Serial.print("yaw: ");
    // Serial.println(servo_yaw.read());
    // delay(1000);
    // servo_yaw.write(servo_yaw_zero+YAW_SERVO_MIN);
    // Serial.print("yaw: ");
    // Serial.println(servo_yaw.read());
    // delay(1000);
    // servo_yaw.write(servo_yaw_zero);
    // Serial.print("yaw: ");
    // Serial.println(servo_yaw.read());
    // delay(1000);
    // servo_yaw.write(servo_yaw_zero+YAW_SERVO_MAX);
    // Serial.print("yaw: ");
    // Serial.println(servo_yaw.read());
    // delay(1000);
    // servo_yaw.write(servo_yaw_zero);
    // Serial.print("yaw: ");
    // Serial.println(servo_yaw.read());



    for (int i=1500; i>=1500+YAW_SERVO_MIN*(1/0.18); i--) {
        servo_yaw.write(i);
        servo_pitch.write(i);
        Serial.print("yaw1: ");
        Serial.println(servo_yaw.read());
        delay(20);
    }
    for (int i=servo_yaw_zero+YAW_SERVO_MIN*(1/0.18); i<=servo_yaw_zero+YAW_SERVO_MAX*(1/0.18); i++) {
        servo_yaw.write(i);
        servo_pitch.write(i);
        Serial.print("yaw2: ");
        Serial.println(servo_yaw.read());
        delay(20);
    }
    for (int i=servo_yaw_zero+YAW_SERVO_MAX*(1/0.18); i>=1500; i--) {
        servo_yaw.write(i);
        servo_pitch.write(i);
        Serial.print("yaw3: ");
        Serial.println(servo_yaw.read());
        delay(20);
    }

    while(1) {}

}

// state transition detection timer stuff
static float lastTime = millis();
static float holdTime = 0.0;

int printnum = 0;

void loop() {

    /**** State estimation *****/
    // Read from sensor
    sensors_event_t raw_accelo, raw_gyro, raw_temp;
    mpu.getEvent(&raw_accelo, &raw_gyro, &raw_temp);
    float altitude = bmp.readAltitude(1013.25); // (meters), sealvlhpa = 1013.25
    // bmp_pressure->getEvent(&raw_pressure);
    // double altitude = 44330 * (1.0 - pow( (bmp_pressure->pressure/100) / 1013.25, 0.1903)); // (sealvlhpa = 1013.25)


    // Implement offset
    raw_gyro.gyro.x += GYRO_X_OFFSET;
    raw_gyro.gyro.y += GYRO_Y_OFFSET;
    raw_gyro.gyro.z += GYRO_Z_OFFSET;
    raw_accelo.acceleration.x += ACCELO_X_OFFSET;
    raw_accelo.acceleration.y += ACCELO_Y_OFFSET;
    raw_accelo.acceleration.z += ACCELO_Z_OFFSET;

    // push values to end of buffer
    pitch_rate_buffer.push(raw_gyro.gyro.x);
    roll_rate_buffer.push(raw_gyro.gyro.y);
    yaw_rate_buffer.push(-1*raw_gyro.gyro.z);
    accel_x_buffer.push(raw_accelo.acceleration.y+9.807);
    accel_y_buffer.push(raw_accelo.acceleration.x);
    accel_z_buffer.push(-1*raw_accelo.acceleration.z);
    pos_z_buffer.push(altitude);

    if (printnum%30 == 0) {


      // Serial.print("rawy:");
      // Serial.print(raw_gyro.gyro.y);
      // Serial.println(",");
      // Serial.print("rawx:");
      // Serial.print(raw_gyro.gyro.x);
      // Serial.println(",");
      // Serial.print("rawz:");
      // Serial.print(raw_gyro.gyro.z);
      // Serial.println("");
    }

    // system waits until all buffers are full before running real loop
    if (!pitch_rate_buffer.isFull() &&
        !roll_rate_buffer.isFull() && 
        !yaw_rate_buffer.isFull() &&
        !accel_x_buffer.isFull() &&
        !accel_y_buffer.isFull() &&
        !accel_z_buffer.isFull() &&
        !pos_z_buffer.isFull()) {
      return;
    }

    // calculate moving avg (avg of buffer array)
    float pitch_rate_sum = 0.0f;
    float roll_rate_sum = 0.0f;
    float yaw_rate_sum = 0.0f;
    float accel_x_sum = 0.0f;
    float accel_y_sum = 0.0f;
    float accel_z_sum = 0.0f;
    float pos_z_sum = 0.0f;
    for (int i=0; i<BUFFER_SIZE; i++) {
        pitch_rate_sum += pitch_rate_buffer[i];
        roll_rate_sum += roll_rate_buffer[i];
        yaw_rate_sum += yaw_rate_buffer[i];
        accel_x_sum += accel_x_buffer[i];
        accel_y_sum += accel_y_buffer[i];
        accel_z_sum += accel_z_buffer[i];
        pos_z_sum += pos_z_buffer[i];
    }
    rocket_state.pitch_rate = pitch_rate_sum / (float)BUFFER_SIZE;
    rocket_state.roll_rate = roll_rate_sum / (float)BUFFER_SIZE; //
    rocket_state.yaw_rate = yaw_rate_sum / (float)BUFFER_SIZE;
    rocket_state.accel_x = accel_x_sum / (float)BUFFER_SIZE;
    rocket_state.accel_y = ( (accel_y_sum) / (float)BUFFER_SIZE);
    rocket_state.accel_z = accel_z_sum / (float)BUFFER_SIZE;
    rocket_state.pos_z = pos_z_sum / (float)BUFFER_SIZE;

    // Integrate
    time_step = (millis() - prev_time)/1000.0f;
    rocket_state.pitch = prev_rocket_state.pitch + 0.5f * time_step * rocket_state.pitch_rate;
    rocket_state.yaw = prev_rocket_state.yaw + 0.5f * time_step * rocket_state.yaw_rate;
    rocket_state.roll = prev_rocket_state.roll + 0.5f * time_step * rocket_state.roll_rate;
    rocket_state.vel_x = prev_rocket_state.vel_x + 0.5f * time_step * rocket_state.accel_x;
    rocket_state.vel_y = prev_rocket_state.vel_y + 0.5f * time_step * rocket_state.accel_y;
    rocket_state.vel_z = prev_rocket_state.vel_z + 0.5f * time_step * rocket_state.accel_z;
    rocket_state.pos_x = prev_rocket_state.pos_x + 0.5f * time_step * rocket_state.vel_x;
    rocket_state.pos_y = prev_rocket_state.pos_y + 0.5f * time_step * rocket_state.vel_y;

    if (printnum % 30 == 0) {
      // Serial.print("estimated_gyro_x:");
      // Serial.print(rocket_state.pitch_rate);
      // Serial.print(",");

      Serial.print("rolly:");
      Serial.print(rocket_state.roll);
      Serial.println(",");
      Serial.print("pitchx:");
      Serial.print(rocket_state.pitch);
      Serial.println(",");
      Serial.print("yawz:");
      Serial.print(rocket_state.yaw);
      Serial.println("");

      // Serial.print("error:");
      // Serial.print(rocket_state.pitch_rate - raw_gyro.gyro.x);
      // Serial.println("");
    }
    printnum++;

    prev_rocket_state = rocket_state;

    // Serial.print("Estimated pitch: ");
    // Serial.print(rocket_state.pitch);
    // Serial.println(rocket_state.pitch);

    // Serial.print("Estimated_gyro_x:");
    // Serial.print(rocket_state.pitch);
    // Serial.println("");


    // Execute LKF
    // if (is_first_iteration) {
        
    //     double seaLevelhpa = bmp_pressure.pressure; // set sea lvl pressure at gnd

    //     update_state_transition(raw_gyro.gyro.x,raw_gyro.gyro.y,raw_gyro.gyro.z);
    //     predict_state();
    //     predict_covariance();

    //     is_first_iteration = 0;
    // } else {
    //     measurement = state_current;
    //     update_state();
    //     update_covariance();

    //     eulerAngles = quatToEuler(state_current); // convert estimated quats to euler angles to be used in PID

    //     update_state_transition(raw_gyro.gyro.x,raw_gyro.gyro.y,raw_gyro.gyro.z);
    //     predict_state();
    //     predict_covariance();
    // }
    // state_prev = state_future; // not really the "previous state", its the "previously predicted state"
    // covariance_prev = covariance_future;

    // Enter state machine
    switch(state_machine) {

        // ignore this comment vvv
        // if still in static estimation, calling Read_State_Estimate() will simply not do anything
        // ^ during static estimation SE teensy sends null values thru serial

        case (ARMED): {

            if (rocket_state.accel_z > 10.0 && rocket_state.pos_z > 1.0 ) {
                
              holdTime = holdTime + millis()-lastTime;
              lastTime = millis();
              if (holdTime > 1500) { // wait 1.5 seconds before enabling tvc
                  state_machine = FASTASCENT;
                  holdTime = 0;
              }
            }else{
              holdTime = 0;
              lastTime = millis();
            }
            break;

        }

        case (FASTASCENT): {

            // delay(TIME_STEP); //at least longer than time response

            /********* ControlTVC() ***************/
            // Control_TVC(raw_gyro.gyro.x,raw_gyro.gyro.y); // TODO need to reorient
            pitch_controller.time_step = millis()/1000.0 - prev_time;
            yaw_controller.time_step = millis()/1000.0 - prev_time;
            // prev_time = millis();

            float pitch_cmd = PID_Controller_Update(&pitch_controller, 0.0f, rocket_state.pitch);
            float yaw_cmd = PID_Controller_Update(&yaw_controller, 0.0f, rocket_state.yaw);

            // send actuation cmd
            servo_pitch.write(servo_pitch_zero + pitch_cmd*(180.0f/M_PI));
            servo_yaw.write(servo_yaw_zero + yaw_cmd*(180.0f/M_PI));

            // Fast_Data_Log(&rocket_state, /*motor posn*/);

            if (rocket_state.accel_z < 0.0f) {
              holdTime = holdTime + millis()-lastTime;
              lastTime = millis();
              if (holdTime > 1000) {
                  state_machine = SLOWASCENT;
                  holdTime = 0;
              }
            }else{
              holdTime = 0;
              lastTime = millis();
            }

            break;

        }

        case (SLOWASCENT): {

            if (rocket_state.pos_z < prev_rocket_state.pos_z) {

                //TODO timestamp and log apogee event to sd card
              holdTime = holdTime + millis()-lastTime;
              lastTime = millis();
              if (holdTime > 1000) {
                  state_machine = FREEFALL;
                  holdTime = 0;
              }

            }else{
              holdTime = 0;
              lastTime = millis();
            }
            break;

        }

        case (FREEFALL): {
            if (rocket_state.accel_z < -9.8 && rocket_state.accel_z > 0.0) {
              holdTime = holdTime + millis()-lastTime;
              lastTime = millis();
              if (holdTime > 1000) {
                  state_machine = CHUTE;
                  holdTime = 0;
              }
            } else if (rocket_state.pos_z < 5.0) { 
              
                  holdTime = holdTime + millis()-lastTime;
                  lastTime = millis();
                  if (holdTime > 1000) {
                      state_machine = LANDED;
                      holdTime = 0;
                  }
              
            } else {
              lastTime = millis();
              holdTime = 0;
            }
            break;

        }

        case (CHUTE): {
            if (rocket_state.pos_z < 5.0) { 
              holdTime = holdTime + millis()-lastTime;
              lastTime = millis();
              if (holdTime > 1000) {
                  state_machine = LANDED;
                  holdTime = 0;
              }
            }else{
              lastTime = millis();
              holdTime = 0;
            }
            break;
        }

        case (LANDED): {
            //TODO timestamp and log landing to sd card
            // TODO: close SD card
            break;

        }

        default: {
          Serial.println("ASSERT: Invalid state?????");
          while(1) {};
          break;
        }

    }

    prev_time = millis(); // set previous time to current time, then go to next iteration

}