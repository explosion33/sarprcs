// use function calls to get rocket position n stuff
#include "PID.h"
#include "Actuator.h"
#include <chrono>

using namespace std;

const float solenoid_thrust = 20; // newtons
const float min_act_time = 0.2; // seconds
const double dt = 0.01;


PID x_controller(40,1,20);
PID y_controller(40,1,20);
PID z_controller(50,0.01,0.1);

Actuator x_act(solenoid_thrust, min_act_time);
Actuator y_act(solenoid_thrust, min_act_time);
Actuator z_act(solenoid_thrust, min_act_time);

struct vec3 {
    double x, y, z;
};

auto getTime(){
    auto t = chrono::high_resolution_clock::now();
    return t;
}

vec3 getAttitude() { // FILLER FUNCTION
    
    vec3 state_vec = {0, 0, 0}; // theta_x, theta_y, omega_z FROM SENSORS

    return state_vec; //address of state_vector returned
}

vec3 getCommands(vec3 state_vec, float dt) {
        float x_cmd = x_controller.compute(state_vec.x, dt);
        float y_cmd = y_controller.compute(state_vec.y, dt);
        float z_cmd = z_controller.compute(state_vec.z, dt);
        vec3 commands = {x_cmd, y_cmd, z_cmd};
        return commands; //address of commands
    }

int setStates(vec3 cmds) { // Use a method to send signal to solenoids
        x_act.state = x_act.setState(t, cmds.x);
        y_act.state = y_act.setState(t, cmds.y);
        z_act.state = z_act.setState(t, cmds.z);
        return 0;
}

auto t_last = std::chrono::high_resolution_clock::now();

int main() {

    while (true){
        vec3 state_vector = getAttitude();
        vec3 cmd = getCommands(state_vector, dt);
        setStates(cmd);
        t_last = std::chrono::high_resolution_clock::now();
    }

    return 0;
}