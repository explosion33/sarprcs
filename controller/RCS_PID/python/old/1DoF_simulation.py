from RCS_PID.python.PID import PID 
from RCS_PID.python.physics import oneDofPhysics
import numpy as np
import matplotlib.pyplot as plt

# --INITIALIZE STATE--
theta0 = -0.5 # rad
omega0 = -0.5 # rad/s
initial_state = [theta0, omega0]
# --------------------

# --SIM PARAMS-- (units in N, m, kg, s, etc.) 
dt = 0.01 
mmoi = 1000
rz = 5 # vertical distance from COM to thruster 
sim_time = 0
time_limit = 100
# --------------

# --CONTROLLER PARAMS--
Kp = 500
Ki = 1
Kd = 50
min_thrust = 5
max_thrust = 100
# ---------------------

controller = PID(Kp, Ki, Kd)
controller.setLims(min_thrust, max_thrust) # min & max thrust in newtons
rocket = oneDofPhysics(initial_state, mmoi, rz)

N = int(time_limit/dt)
thetas = [None]*N
omegas = [None]*N
commands = [None]*N
i=0
while sim_time < time_limit:
    thetas[i], omegas[i] = rocket.state_vector
    command = controller.compute(rocket.state_vector[0], dt)
    commands[i] = command
    # theta_deg = rocket.state_vector[0]*360/(2*np.pi)
    # omega_deg_s = rocket.state_vector[1]*360/(2*np.pi)
    # print("time: ", round(sim_time,3),"s"
    #       "\ttheta: ", round(theta_deg,3),
    #       "\tomega: ", round(omega_deg_s,3),
    #       "\tcommand: ", round(command,3))
    rocket.nextState(command, dt)
    sim_time += dt
    i+=1

# --PLOTTING--
theta_deg = [t*360/(2*np.pi) for t in thetas]
omega_deg_s = [o*360/(2*np.pi) for o in omegas]
time = np.linspace(0, time_limit, N)
fig, ax = plt.subplots(3)
ax[0].plot(time, theta_deg, 'b-')
ax[1].plot(time, omega_deg_s,'m-')
ax[2].plot(time, commands, 'r-')
ax[0].set_ylabel(r"$\theta_x$ [deg]", fontsize=15)
ax[1].set_ylabel(r"$\omega_x$ [deg/s]", fontsize=15)

fig.savefig("RCS_PID/figs/1DoF_States.png")
fig.show()