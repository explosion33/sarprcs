
#ifndef _Actuator_H_
#define _Actuator_H_

class Actuator{
    public:
        Actuator(float solenoid_thrust, float min_act_time);
        float setState(float current_time, float cmd);
        float state;

    private:
        float thrust;
        float min_act_time;
        float time_switch;
        float ontime;
};

#endif //_Actuator_H_