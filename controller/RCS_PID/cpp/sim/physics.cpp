// #include matrixMult;
#include <cmath>

class RocketPhysics{
    public:
        float state_vector;
        float mmoiX;
        float mmoiZ;
        float mmoiY;
        float rz;
        float rx;
        RocketPhysics(float inital_state_vector, float MMOI_X, float MMOI_Z, float r_z, float r_x){
            state_vector = inital_state_vector;
            mmoiX = MMOI_X;
            mmoiZ = MMOI_Z;
            mmoiY = MMOI_X; //symmetric
            rz = r_z;
            rx = r_x;
        }

        void forces(float u, float dt){
            // STATE SPACE EQUATION
            // x = Ax+Bu (dt is inside A and B matrices)
            // state_vector = matrixMult(A,x) + matrixMult(B,u)
            double A[5][5] = {  {1 ,0, dt, 0, 0},
                                {0, 1, 0, dt, 0},
                                {0, 0, 1, 0, 0},
                                {0, 0, 0, 1, 0},
                                {0, 0, 0, 0, 1},
                            };
            double B[5][5] = {  {0, 0.5*(rz/mmoiX)*(pow(dt,2), 0.5*(rz/mmoiX)*pow(dt,2))},
                                {0.5*(rz/mmoiY)*pow(dt,2), 0, 0},
                                {0, (rz/mmoiX)*dt, (rz/mmoiX)*dt},
                                {(rz/mmoiY)*dt, 0, 0},
                                {0, -(rx/mmoiZ)*dt, (rz/mmoiZ)*dt},
                            };

        }
};