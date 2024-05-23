#include "PID.h"

PID::PID(float KP, float KI, float KD) {
    Kp = KP;
    Ki = KI;
    Kd = Kd;
    error_last = 0;
    integral_error = 0;
}

float PID::compute(float theta,float dt) {
    float error = theta; //error will just be theta
    integral_error += error*dt; // sum all errors*dt
    float P = -1*Kp*error;
    float I = -1*Ki*integral_error;
    float D = -1*Kd*( error - error_last )/dt;
    float output = P+I+D;
    error_last = error;

    return output;
}

