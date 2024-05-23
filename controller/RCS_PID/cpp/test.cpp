#include "PID.h"
#include "Actuator.h"

const float solenoid_thrust = 20; // newtons
const float min_act_time = 0.2; // seconds

PID x_controller(40,1,20);
Actuator x_act(solenoid_thrust, min_act_time);

const float dt = 0.01;

int main(){

    float state_vector[3] = {1, -2, -7};
    float x_cmd = x_controller.compute(state_vector[0], 0.01);
    x_act.state = x_act.setState(t, x_cmd);
}