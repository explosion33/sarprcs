
#ifndef _PID_H_
#define _PID_H_

class PID {
    public:
        PID(float KP, float KI, float KD); //constructor
        float compute(float theta,float dt);

    private:
        float Kp;
        float Ki;
        float Kd;
        float setpoint;
        float error_last;
        float integral_error;
        float max_thrust;
        float min_thrust;
};


#endif //_PID_H_