from PID import PID 
from physics import threeDofPhysics
import numpy as np
import matplotlib.pyplot as plt

# --INITIALIZE STATE--
initial_state = {'thX': -0.5,
                 'wX': -0.3,
                 'thY': 0.2,
                 'wY': -1,
                 'wZ': 5,}
# --------------------

# --SIM PARAMS-- (units in N, m, kg, s, etc.) 
dt = 0.01 
mmoiX = 2000
mmoiZ = 50
rz = 5 # vertical distance from COM to thruster
rx = 0.1 # distanace from centerline to offset thrust vector
sim_time = 0
time_limit = 50
# --------------

# --CONTROLLER PARAMS--
Kp = 500
Ki = 1
Kd = 100
min_thrust = 5
max_thrust = 50
# ---------------------

coeff_rad_to_degrees = 360/(2*np.pi)

x_controller = PID(Kp, Ki, Kd)
y_controller = PID(Kp, Ki, Kd)
z_controller = PID(100, 0, 0) # z_controller gets handed angular velocty, not angle

x_controller.setLims(min_thrust, max_thrust) # min & max thrust in newtons
y_controller.setLims(min_thrust, max_thrust)
z_controller.setLims(min_thrust, max_thrust)

rocket = threeDofPhysics(initial_state, mmoiX, mmoiZ, rz, rx)

N = int(time_limit/dt)+1
Xthetas = [None]*N
Ythetas = [None]*N
Xomegas = [None]*N
Yomegas = [None]*N
Zomegas = [None]*N
Xcommands = [None]*N 
Ycommands = [None]*N
Zcommands = [None]*N 
i=0
while sim_time < time_limit:
    Xthetas[i] = rocket.state_vector['thX'] *coeff_rad_to_degrees
    Ythetas[i] = rocket.state_vector['thY'] *coeff_rad_to_degrees
    Xomegas[i] = rocket.state_vector['wX'] *coeff_rad_to_degrees
    Yomegas[i] = rocket.state_vector['wY'] *coeff_rad_to_degrees 
    Zomegas[i] = rocket.state_vector['wZ']  *coeff_rad_to_degrees
    x_command = x_controller.compute(rocket.state_vector['thX'], dt)
    y_command = y_controller.compute(rocket.state_vector['thY'], dt)
    z_command = z_controller.compute(rocket.state_vector['wZ'], dt)
    Xcommands[i], Ycommands[i], Zcommands[i] = x_command, y_command, z_command
    force_vector = {'x_thrust': x_command, 'y_thrust': y_command, 'z_thrust': z_command}
    rocket.forces(force_vector, dt)
    sim_time += dt
    i+=1

# --PLOTTING--
time = np.linspace(0, time_limit, N)
fig, ax = plt.subplots(6, figsize=[8,12])

ax[0].plot(time, Xthetas, color='r')
ax[1].plot(time, Ythetas, color='royalblue')
ax[2].plot(time, Xomegas, color='orange')
ax[3].plot(time, Yomegas, color='springgreen')
ax[4].plot(time, Zomegas, color='m')
ax[5].plot(time, Xcommands, color='r', label='x')
ax[5].plot(time, Ycommands, color='royalblue',label='y')
ax[5].plot(time, Zcommands, color='m',label='z')

ax[0].set_ylabel(r"$\theta_x$", fontsize=15)
ax[1].set_ylabel(r"$\theta_y$", fontsize=15)
ax[2].set_ylabel(r"$\omega_x$", fontsize=15)
ax[3].set_ylabel(r"$\omega_y$", fontsize=15)
ax[4].set_ylabel(r"$\omega_z$", fontsize=15)
ax[5].set_ylabel(r"commands", fontsize=15)
ax[5].legend()
# fig.savefig("RCS_PID/figs/3DoF_States")
plt.tight_layout()
plt.show()
# -----------