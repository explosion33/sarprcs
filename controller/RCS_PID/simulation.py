import PID as pid

controller = pid.PID(1,0.01,0.01)
controller.setLims(1,10)

theta0 = -0.5
dt = 0.01
sim_time = 0
time_limit = 10

# TODO use lqr notes & adapt state space model for PID tuning

def state_space_model(A, state_t_minus_1, B, control_input_t_minus_1):
    """
    Calculates the state at time t given the state at time t-1 and
    the control inputs applied at time t-1
     
    :param: A   The A state transition matrix
        3x3 NumPy Array
    :param: state_t_minus_1     The state at time t-1  
        3x1 NumPy Array given the state is [x,y,yaw angle] ---> 
        [meters, meters, radians]
    :param: B   The B state transition matrix
        3x2 NumPy Array
    :param: control_input_t_minus_1     Optimal control inputs at time t-1  
        2x1 NumPy Array given the control input vector is 
        [linear velocity of the car, angular velocity of the car]
        [meters per second, radians per second]
         
    :return: State estimate at time t
        3x1 NumPy Array given the state is [x,y,yaw angle] --->
        [meters, meters, radians]
    """

    state_estimate_t = (A @ state_t_minus_1) + (B @ control_input_t_minus_1) 
    # x_dot = Ax + Bu
    return state_estimate_t

A = np.array([  [1 ,0, dt, 0, 0],
                [0, 1, 0, dt, 0],
                [0, 0, 1, 0, 0],
                [0, 0, 0, 1, 0],
                [0, 0, 0, 0, 1],
             ])

B = np.array([  [0, 0.5*(r_z/I_x)*(dt**2), 0.5*(r_z/I_x)*(dt**2)],
                [0.5*(r_z/I_y)*(dt**2), 0, 0],
                [0, (r_z/I_x)*dt, (r_z/I_x)*dt],
                [(r_z/I_y)*dt, 0, 0],
                [0, -(r_x/I_z)*dt, (r_z/I_z)*dt],
             ])
    
while sim_time < time_limit:

    controller.compute(theta,dt)
    sim_time += dt