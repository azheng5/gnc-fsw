#define LKF_TIME_STEP 0.01

Eigen::Vector4f state; // state vector



float Measure();

UpdateState(float* state_estimate, float* covariance, float* measurement, float* observation, float* measurement_covariance);

float UpdateCovariance(float* covariance, float* observation, float* measurement_covariance);

float PredictState(float* state_estimate,float* control_input, float* process_noise);

float PredictCovariance(float* covariance);

float EulerToQuaternion(float* state_estimate);