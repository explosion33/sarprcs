
class PID {
    public:
        float Kp;
        float Ki;
        float Kd;
        float setpoint;
        float error_last;
        float integral_error;
        float max_thrust;
        float min_thrust;

        // do these go here???
            float error;
            float P;
            float I;
            float D;
            float output;


        PID(float KP, float KI, float KD) {
            error_last = 0;
            integral_error = 0;
            // max_thrust = None;
            // min_thrust = None;
            Kp = KP;
            Ki = KI;
            Kd = Kd;
        }

        void compute(float theta,float dt) {
            error = theta; //error will just be theta
            integral_error += error*dt; // sum all errors*dt
            P = -1*Kp*error;
            I = -1*Ki*integral_error;
            D = -1*Kd*( error - error_last )/dt;
            output = P+I+D;
            error_last = error;
        }

        void setLims(float min, float max) {
        // Sets minimum & maximum thrust in newtons
        max_thrust = max;
        min_thrust = min;
        }

};