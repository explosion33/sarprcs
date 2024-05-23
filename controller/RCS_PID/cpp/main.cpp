// use function calls to get rocket position n stuff
#include "PID.h"
#include "Actuator.h"
#include <chrono>

using namespace std;

int main() {

    const float solenoid_thrust = 20; // newtons
    const float min_act_time = 0.2; // seconds
    auto t = std::chrono::system_clock::now();


    PID x_controller(40,1,20);
    PID y_controller(40,1,20);
    PID z_controller(50,0.01,0.1);

    Actuator x_act(solenoid_thrust, min_act_time);
    Actuator y_act(solenoid_thrust, min_act_time);
    Actuator z_act(solenoid_thrust, min_act_time);

    float* getAttitutde() { // FILLER FUNCTION
        
        state_vec = {0, 0, 0}; // theta_x, theta_y, omega_z FROM SENSORS

        return state_vec; //address of state_vector returned
    }

    float* getCommands(float state_vec, dt) {
         float x_cmd = x_controller.compute(state_vector[0], dt);
         float y_cmd = y_controller.compute(state_vector[1], dt);
         float z_cmd = z_controller.compute(state_vector[2], dt);
         static float commands[3] = {x_cmd, y_cmd, z_cmd};
         return commands; //address of commands
    }

    int setStates(cmds) { // Use a method to send signal to solenoids
        x_act.state = x_act.setState(t, cmds[0]);
        y_act.state = y_act.setState(t, cmds[1]);
        z_act.state = z_act.setState(t, cmds[2]);
        return 0;
    }; // WHY DO I NEED A SEMICOLON HERE FOR t_last TO INSTANTIATE

    auto t_last = std::chrono::system_clock::now();
    while (true){
        auto t = std::chrono::system_clock::now();
        auto dt = t-t_last;
        float state_vector[3] = getAttitude();
        float cmd[3] = getCommands(state_vector, dt);
        setStates(cmd);
        t_last = std::chrono::system_clock::now();
    }

    return 0;
}