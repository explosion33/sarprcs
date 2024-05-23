// use function calls to get rocket position n stuff
#include "PID.h"
#include "Actuator.h"
#include <chrono>
#include <iostream>
#include <thread>

using namespace std;

const float solenoid_thrust = 20; // newtons
const float min_act_time = 0.2; // seconds
const int dt = 100; // miliseconds

PID x_controller(40,1,20);
PID y_controller(40,1,20);
PID z_controller(50,0.01,0.1);

Actuator x_act(solenoid_thrust, min_act_time);
Actuator y_act(solenoid_thrust, min_act_time);
Actuator z_act(solenoid_thrust, min_act_time);

struct vec3 {
    double x, y, z;
};

vec3 getAttitude() { // FILLER FUNCTION
    
    vec3 state_vec = {-0.2, 0.5, 1.0}; // theta_x, theta_y, omega_z FROM SENSORS

    return state_vec; //address of state_vector returned
}

vec3 getCommands(vec3 state_vec, float dt) {
        float x_cmd = x_controller.compute(state_vec.x, dt);
        float y_cmd = y_controller.compute(state_vec.y, dt);
        float z_cmd = z_controller.compute(state_vec.z, dt);
        vec3 commands = {x_cmd, y_cmd, z_cmd};
        return commands; //address of commands
    }

int setStates(double t, vec3 cmds) { // Use a method to send signal to solenoids
        x_act.state = x_act.setState(t, cmds.x);
        y_act.state = y_act.setState(t, cmds.y);
        z_act.state = z_act.setState(t, cmds.z);
        return 0;
}

double t = 0;

int main() {
    while (true){
        vec3 state_vector = getAttitude();
        vec3 cmd = getCommands(state_vector, dt);
        setStates(t, cmd);
        cout << t << "\tstates: " << state_vector.x <<", "<< state_vector.y<<", "<< state_vector.z;
        cout << "\tcommands: " << cmd.x<<", "<< cmd.y<<", "<< cmd.z;
        cout << "\tactuator: " << x_act.state << "\n";
        this_thread::sleep_for(chrono::milliseconds(dt));
        t += double(dt)/1000;
    }
    return 0;

    // double x_ang = -0.0349;
    // float x_cmd = x_controller.compute(x_ang, dt);
    // x_act.setState(t, x_cmd);
    // cout << t << "\tx-angle: " << x_ang;
    // cout << "\tcommand: " << x_cmd;
    // cout << "\tactuator: " << x_act.state << "\n";
    // return 0;

}