#include "../include/state-est-sim.hpp" // replace with angular brackets
#include MatrixMath.h

float Measure() {

}

float UpdateState(float* state_estimate, float* covariance, float* measurement, float* observation, float* measurement_covariance) {

}

float UpdateCovariance(float* covariance, float* observation, float* measurement_covariance) {

}

float PredictState(float* state_estimate,float* control_input, float* process_noise) {

}

float PredictCovariance(float* covariance) {

}

float EulerToQuaternion(float* state_estimate) {

}

float state_estimate[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

int main(int argc, char *argv[]) {

    
    
    // initialize
    PredictState();
    PredictCovariance();
    delay(TIME_STEP*1000.0);
    while(1) {
        
        Measure();

        UpdateState();

        UpdateCovariance();

        PredictState()

        PredictCovariance();

        delay(TIME_STEP*1000.0);
    }


}