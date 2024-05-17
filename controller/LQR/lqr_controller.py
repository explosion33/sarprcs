import numpy as np
import matplotlib.pyplot as plt
 
######################## DEFINE CONSTANTS #####################################
# Supress scientific notation when printing NumPy arrays
np.set_printoptions(precision=3,suppress=True)
 
# Optional Variables
#max_linear_velocity = 3.0 # meters per second
#max_angular_velocity = 1.5708 # radians per second
 
# physical parameters

r_z = 5 # [m]
r_x = .1 # [m] distance from y1 and y2 thrusters to center
I_x = 3200 # [m^4]
I_y = I_x
I_z = 14 # [m^4]

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
     
def lqr(actual_state_x, desired_state_xf, Q, R, A, B, dt):
    """
    Discrete-time linear quadratic regulator for a nonlinear system.
 
    Compute the optimal control inputs given a nonlinear system, cost matrices, 
    current state, and a final state.
    
    Compute the control variables that minimize the cumulative cost.
 
    Solve for P using the dynamic programming method.
 
    :param actual_state_x: The current state of the system 
        3x1 NumPy Array given the state is [x,y,yaw angle] --->
        [meters, meters, radians]
    :param desired_state_xf: The desired state of the system
        3x1 NumPy Array given the state is [x,y,yaw angle] --->
        [meters, meters, radians]   
    :param Q: The state cost matrix
        3x3 NumPy Array
    :param R: The input cost matrix
        2x2 NumPy Array
    :param dt: The size of the timestep in seconds -> float
 
    :return: u_star: Optimal action u for the current state 
        2x1 NumPy Array given the control input vector is
        [linear velocity of the car, angular velocity of the car]
        [meters per second, radians per second]
    """
    # We want the system to stabilize at desired_state_xf.
    x_error = actual_state_x - desired_state_xf
 
    # Solutions to discrete LQR problems are obtained using the dynamic 
    # programming method.
    # The optimal solution is obtained recursively, starting at the last 
    # timestep and working backwards.
    # You can play with this number
    N = 50
 
    # Create a list of N + 1 elements
    P = [None] * (N + 1)
    
    Qf = Q
 
    # LQR via Dynamic Programming
    P[N] = Qf
 
    # For i = N, ..., 1
    for i in range(N, 0, -1):
 
        # Discrete-time Algebraic Riccati equation to calculate the optimal 
        # state cost matrix
        P[i-1] = Q + A.T @ P[i] @ A - (A.T @ P[i] @ B) @ np.linalg.pinv(
            R + B.T @ P[i] @ B) @ (B.T @ P[i] @ A)      
 
    # Create a list of N elements
    K = [None] * N
    u = [None] * N
 
    # For i = 0, ..., N - 1
    for i in range(N):
 
        # Calculate the optimal feedback gain K
        K[i] = -np.linalg.pinv(R + B.T @ P[i+1] @ B) @ B.T @ P[i+1] @ A
 
        u[i] = K[i] @ x_error
 
    # Optimal control input is u_star
    u_star = u[N-1]
 
    return u_star
 
def main():
     
    # Let the time interval be 0.01 seconds
    dt = 0.01
     
    # Actual state
    # rocket starts at tX = 3 rad, tY = 1 rad,
    # tXdot = 1 rad/s, tYdot = 1 rad/s, tZdot = 1 rad/s
    actual_state_x = np.array([np.pi/2,-np.pi/2,3,-2,15]) 
 
    # Desired state [tX,tY,tXdot,tYdot,tZdot]
    # [rad, rad, rad/s, rad/s, rad/s]
    desired_state_xf = np.array([0,0,0,0,0])  
     
    # A matrix
    # 5x5 matrix -> number of states x number of states matrix
    # Expresses how the state of the system [tX, tY, tXdot, tYdot, tZdot] changes 
    # from t-1 to t when no control command is executed.  
    A = np.array([  [1 ,0, dt, 0, 0],
                    [0, 1, 0, dt, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 1],
                 ])

    # B matrix
    # 5x3 -> number of states x number of actuators.
    # Expresses how the state of the system changes w/ dt 
    # due to a given actuator input.
    B = np.array([  [0, 0.5*(r_z/I_x)*(dt**2), 0.5*(r_z/I_x)*(dt**2)],
                    [0.5*(r_z/I_y)*(dt**2), 0, 0],
                    [0, (r_z/I_x)*dt, (r_z/I_x)*dt],
                    [(r_z/I_y)*dt, 0, 0],
                    [0, -(r_x/I_z)*dt, (r_z/I_z)*dt],
                 ])
 
    # R matrix
    # The control input cost matrix
    # Experiment with different R matrices
    # This matrix penalizes actuator effort (i.e. rotation of the 
    # motors on the wheels that drive the linear velocity and angular velocity).
    # The R matrix has the same number of rows as the number of control
    # inputs and same number of columns as the number of 
    # control inputs.
    # This matrix has positive values along the diagonal and 0s elsewhere.
    # We can target control inputs where we want low actuator effort 
    # by making the corresponding value of R large. 
    R = np.array([[0.00000001]])
 
    # Q matrix
    # The state cost matrix.
    # Experiment with different Q matrices.
    # Q helps us weigh the relative importance of each state in the 
    # state vector (X angle, Y angle, X angular velocity, Y angular velocity, Z angular velocity). 
    # Q is a square matrix that has the same number of rows as 
    # there are states.
    # Q penalizes bad performance.
    # Q has positive values along the diagonal and zeros elsewhere.
    # Q enables us to target states where we want low error by making the 
    # corresponding value of Q large.
    Q = np.array([
                    [1000, 0, 0, 0, 0],
                    [0, 1000, 0, 0, 0],
                    [0, 0, 10, 0, 0],
                    [0, 0, 0, 10, 0],
                    [0, 0, 0, 0, 10000]
                ])
                   
    # Launch the rocket, and have it adjust to the desired orientation

    time = []
    tX = []
    tY = []
    tXdot = []
    tYdot = []
    tZdot = []
    for i in range(15000):
        print(f'iteration = {i/100} seconds')
        print(f'Current State = {actual_state_x}')
        print(f'Desired State = {desired_state_xf}')

        time.append(i/100)
        tX.append(actual_state_x[0])
        tY.append(actual_state_x[1])
        tXdot.append(actual_state_x[2])
        tYdot.append(actual_state_x[3])
        tZdot.append(actual_state_x[4])

        state_error = actual_state_x - desired_state_xf
        state_error_magnitude = np.linalg.norm(state_error)     
        print(f'State Error Magnitude = {state_error_magnitude}')
         
        # LQR returns the optimal control input
        optimal_control_input = lqr(actual_state_x, 
                                    desired_state_xf, 
                                    Q, R, A, B, dt) 
         
        print(f'Control Input = {optimal_control_input}')
                                     
         
        # apply the optimal control to the rocket
        # so we can get a new actual (estimated) state.
        actual_state_x = state_space_model(A, actual_state_x, B, 
                                        optimal_control_input * 1.4)  
        # this will
        # Stop as soon as we reach the goal
        # Feel free to change this threshold value.
        if state_error_magnitude < 0.5:
            print("\nGoal Has Been Reached Successfully!")
            break

        # Stop if it has been greater than 20 seconds and the
        # goal state has not been reached
        if i/100 > 100:
            print("\nmax time limite has been reached")
            break
    
        print()
 
    fig, ax = plt.subplots(5, 1, sharex=True, figsize=(8,8))
    
    ax[0].set_title("States over Time", fontsize=15)

    ax[0].plot(time, tX, '-', color='r')
    ax[1].plot(time, tY, '-', color='royalblue')
    ax[2].plot(time, tXdot, '-', color='orange')
    ax[3].plot(time, tYdot, '-', color='springgreen')
    ax[4].plot(time, tZdot, '-', color='magenta')

    ax[0].set_ylabel(r"$\theta_x$", fontsize=15)
    ax[1].set_ylabel(r"$\theta_y$", fontsize=15)
    ax[2].set_ylabel(r"$\omega_x$", fontsize=15)
    ax[3].set_ylabel(r"$\omega_y$", fontsize=15)
    ax[4].set_ylabel(r"$\omega_z$", fontsize=15)

    ax[4].set_xlabel("Time [s]", fontsize=15)

    plt.tight_layout()
    plt.show()

# Entry point for the program
main()