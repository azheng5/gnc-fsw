/**** PARAMETERS *****/
#define TIME_STEP 0.01f

/* Global struct for rocket attitude, given in euler angles */
typedef struct {
    
} StateMachine;

/****** FUNCTIONS ******/

float Measure();

float UpdateState(float* state_estimate, float* covariance, float* measurement, float* observation, float* measurement_covariance);

float UpdateCovariance(float* covariance, float* observation, float* measurement_covariance);

float PredictState(float* state_estimate,float* control_input, float* process_noise);

float PredictCovariance(float* covariance);

float EulerToQuaternion(float* state_estimate);