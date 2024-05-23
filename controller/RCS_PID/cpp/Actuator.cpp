
#include "Actuator.h"
#include <cmath>

Actuator::Actuator(float solenoid_thrust, float min_act_time){
    thrust = solenoid_thrust;
    min_act_time = min_act_time;
    state = 0;
    time_switch = 0;
    ontime = 0;
}

float Actuator::setState(float current_time, float cmd){
    ontime = current_time - time_switch; // track ontime

    int cmd_state = 0;
    if (cmd != 0) {
        cmd_state = cmd / abs(cmd);// either 1 or -1
    }

    if (state != cmd_state && ontime >= min_act_time){
        ontime = 0; // reset ontime if a different command is sent
        time_switch = current_time; // mark when solenoid state was switched
    
        if (abs(cmd) > min_act_time){
            state = cmd_state;
            time_switch = current_time; // mark when solenoid state was switched
        } 
        else{ // command is too small
            state = 0;
            time_switch = current_time;
        }    
    }
    return state;
            
}