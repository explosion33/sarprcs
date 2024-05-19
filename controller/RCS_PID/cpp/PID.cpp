#include "PID.h"

PID::PID(float KP, float KI, float KD) {
    max_thrust = 999999999;
    min_thrust = 999999999; // can I do this?
    error_last = 0;
    integral_error = 0;
    // max_thrust = None;
    // min_thrust = None;
    Kp = KP;
    Ki = KI;
    Kd = Kd;
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

void PID::setLims(float min, float max) {
    // Sets minimum & maximum thrust in newtons
    max_thrust = max;
    min_thrust = min;

}