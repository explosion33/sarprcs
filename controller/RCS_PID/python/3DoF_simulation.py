from PID import PID 
from physics import threeDofPhysics
import numpy as np
import matplotlib.pyplot as plt
import time

coeff_rad_to_deg = 360/(2*np.pi)
coeff_deg_to_rad = (2*np.pi)/360


# --INITIALIZE STATE--
initial_state = {'thX': -5 *coeff_deg_to_rad,
                 'wX': -2 *coeff_deg_to_rad,
                 'thY': 7 *coeff_deg_to_rad,
                 'wY': -1 *coeff_deg_to_rad,
                 'wZ': 50 *coeff_deg_to_rad,}
# --------------------

# --SIM PARAMS-- (units in N, m, kg, s, etc.) 
dt = 0.01 
mmoiX = 2000
mmoiZ = 50
rz = 5 # vertical distance from COM to thruster
rx = 0.1 # distanace from centerline to offset thrust vector
sim_time = 0
time_limit = 30
solenoid_thrust = 20
# --------------

# --CONTROLLER PARAMS--
Kp = 100
Ki = 1
Kd = 30
min_ontime = 0.5
# ---------------------

x_controller = PID(Kp, Ki, Kd)
y_controller = PID(Kp, Ki, Kd)
z_controller = PID(1000, 1, 1) # z_controller gets handed angular velocty, not angle

x_controller.setMinOntime(min_ontime)
y_controller.setMinOntime(min_ontime)
z_controller.setMinOntime(min_ontime)

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
x_on = False
y_on = False
z_on = False
x_time_on = 0
y_time_on = 0
z_time_on = 0
force_vector = {'x_thrust': solenoid_thrust}
x_act_track = [None]*N
y_act_track = [None]*N
z_act_track = [None]*N
debug = []
while sim_time < time_limit:
    Xthetas[i] = rocket.state_vector['thX'] *coeff_rad_to_deg
    Ythetas[i] = rocket.state_vector['thY'] *coeff_rad_to_deg
    Xomegas[i] = rocket.state_vector['wX'] *coeff_rad_to_deg
    Yomegas[i] = rocket.state_vector['wY'] *coeff_rad_to_deg 
    Zomegas[i] = rocket.state_vector['wZ']  *coeff_rad_to_deg
    x_command = x_controller.compute(rocket.state_vector['thX'], dt)
    y_command = y_controller.compute(rocket.state_vector['thY'], dt)
    z_command = z_controller.compute(rocket.state_vector['wZ'], dt)
    Xcommands[i], Ycommands[i], Zcommands[i] = x_command, y_command, z_command
    ontime_vector = {'x': x_command, 'y': y_command, 'z': z_command}
    # CONTROLLER OUTPUTS SOLENOID ON-TIME
    
    if x_on:
        ontime_x = sim_time - x_time_on
    else:
        ontime_x = 0

    if y_on:
        ontime_y = sim_time - y_time_on
    else:
        ontime_y = 0

    if z_on:   
        ontime_z = sim_time - z_time_on
    else:
        ontime_z = 0

    if not x_on and x_command != 0:
        x_on = True
        x_time_on = sim_time # mark when solenoid was turned ON
    if x_on and ontime_x > abs(x_command) and ontime_x >= min_ontime:
        # if it has been longer than commanded time, turn back off
        x_on = False

    if not y_on and y_command != 0:
        y_on = True
        y_time_on = sim_time
    if y_on and ontime_y > abs(y_command) and ontime_y >= min_ontime:
        y_on = False

    if not z_on and z_command != 0:
        z_on = True
        z_time_on = sim_time
    if z_on and ontime_z > abs(z_command) and ontime_z >= min_ontime:
        z_on = False
    
    # DEBUG
    debug.append((x_on, round(sim_time,3) , x_time_on, round(ontime_x,3)))

    def getSign(cmd):
        if cmd == 0:
            return 0
        return cmd/abs(cmd)

    force_vector = {'x_thrust': getSign(x_command)*solenoid_thrust*x_on, # thrust will be zero if x_on = False
                    'y_thrust': getSign(y_command)*2*solenoid_thrust*y_on,
                    'z_thrust': getSign(z_command)*2*solenoid_thrust*z_on}
    
    x_act_track[i] = getSign(x_command)*x_on
    y_act_track[i] = getSign(y_command)*y_on
    z_act_track[i] = getSign(z_command)*z_on

    rocket.forces(force_vector, dt)
    sim_time += dt
    i+=1

for s in debug[0:1000]:
    print(s)

# -- PLOTTING ------------------------
time = np.linspace(0, time_limit, N)
fig, ax = plt.subplots(3, 3, figsize=[12,8])

# theta
ax[0][0].plot(time, Xthetas, color='#4C4281')
ax[0][1].plot(time, Ythetas, color='#055A39')
ax[0][0].set_ylabel(r"$\theta_x$", fontsize=15)
ax[0][1].set_ylabel(r"$\theta_y$", fontsize=15)

# omega
ax[1][0].plot(time, Xomegas, color='#4C4281')
ax[1][1].plot(time, Yomegas, color='#055A39')
ax[1][2].plot(time, Zomegas, color='#F9216A')
ax[1][0].set_ylabel(r"$\omega_x$", fontsize=15)
ax[1][1].set_ylabel(r"$\omega_y$", fontsize=15)
ax[1][2].set_ylabel(r"$\omega_z$", fontsize=15)

# actuation
ax[2][0].plot(time, x_act_track, color='#4C4281', label='x')
ax[2][1].plot(time, y_act_track, color='#055A39',label='y')
ax[2][2].plot(time, z_act_track, color='#F9216A',label='z')
ax[2][0].set_ylabel("x_cmd", fontsize=15)
ax[2][1].set_ylabel("y_cmd", fontsize=15)
ax[2][2].set_ylabel("z_cmd", fontsize=15)

for i in range(len(ax)):
    for j in range(len(ax[0])):
        ax[i][j].grid()

# fig.savefig("RCS_PID/figs/3DoF_States")
plt.tight_layout()
plt.show()
# -----------------------------------