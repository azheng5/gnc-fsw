#include <CircularBuffer.h>

#define BUFFER_SIZE 1 // must be odd if using symmetrical buffer

#define GYRO_X_OFFSET 0.04f // offset in the accelerometer frame
#define GYRO_Y_OFFSET 0.01f // offset in the accelerometer frame
#define GYRO_Z_OFFSET 0.0f // offset in the accelerometer frame
#define ACCELO_X_OFFSET 0.0f // offset in the accelerometer frame
#define ACCELO_Y_OFFSET 0.0f// offset in the accelerometer frame
#define ACCELO_Z_OFFSET 0.0f// offset in the accelerometer frame

// buffers for sensor measurements for moving avg filter
CircularBuffer<float, BUFFER_SIZE> pitch_rate_buffer;
CircularBuffer<float, BUFFER_SIZE> roll_rate_buffer;
CircularBuffer<float, BUFFER_SIZE> yaw_rate_buffer;
CircularBuffer<float, BUFFER_SIZE> accel_x_buffer;
CircularBuffer<float, BUFFER_SIZE> accel_y_buffer;
CircularBuffer<float, BUFFER_SIZE> accel_z_buffer;
CircularBuffer<float, BUFFER_SIZE> pos_z_buffer;