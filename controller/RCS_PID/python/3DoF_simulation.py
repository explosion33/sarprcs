from PID import PID 
from physics import threeDofPhysics
from Actuator import Actuator
import numpy as np
import matplotlib.pyplot as plt
import time

coeff_rad_to_deg = 360/(2*np.pi)
coeff_deg_to_rad = (2*np.pi)/360

# --INITIALIZE STATE--
initial_state = {'thX': 2 *coeff_deg_to_rad,
                 'wX': 2 *coeff_deg_to_rad,
                 'thY': -5 *coeff_deg_to_rad,
                 'wY': -2 *coeff_deg_to_rad,
                 'wZ': -50 *coeff_deg_to_rad,}
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
Kp = 40
Ki = 1
Kd = 20
min_act_time = 0.2
# ---------------------

x_controller = PID(Kp, Ki, Kd)
y_controller = PID(Kp, Ki, Kd)
z_controller = PID(50, 0, 0.1) # z_controller gets handed angular velocty, not angle

x_act = Actuator(solenoid_thrust, min_act_time)
y_act = Actuator(solenoid_thrust, min_act_time)
z_act = Actuator(solenoid_thrust, min_act_time)

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
x_act_track = [None]*N
y_act_track = [None]*N
z_act_track = [None]*N
total_impulse = 0
debug = []
while sim_time <= time_limit:
    Xthetas[i] = rocket.state_vector['thX'] *coeff_rad_to_deg # for plotting
    Ythetas[i] = rocket.state_vector['thY'] *coeff_rad_to_deg
    Xomegas[i] = rocket.state_vector['wX'] *coeff_rad_to_deg
    Yomegas[i] = rocket.state_vector['wY'] *coeff_rad_to_deg 
    Zomegas[i] = rocket.state_vector['wZ']  *coeff_rad_to_deg
    
    x_cmd = x_controller.compute(rocket.state_vector['thX'], dt)
    y_cmd = y_controller.compute(rocket.state_vector['thY'], dt)
    z_cmd = z_controller.compute(rocket.state_vector['wZ'], dt)
    
    x_act.state, x_act.time_on, x_act.ontime = x_act.getState(sim_time, x_cmd)
    y_act.state, y_act.time_on, y_act.ontime = y_act.getState(sim_time, y_cmd)
    z_act.state, z_act.time_on, z_act.ontime = z_act.getState(sim_time, z_cmd)
      
    # FOR DEBUGGING
    debug.append((round(sim_time,3), round(Xthetas[i],3),  x_act.state, round(x_cmd,3), round(x_act.time_on,3), round(x_act.ontime,3)))

    force_vector =  {   'x_thrust': x_act.state * solenoid_thrust,
                        'y_thrust': y_act.state * 2 * solenoid_thrust, # 2 y-solenoids on each side
                        'z_thrust': z_act.state * 2 * solenoid_thrust # 2 z-solenoids (force couple)
                    }
    
    total_impulse += sum(abs(force) for force in force_vector.values())*dt 
    # double counts when the same solenoid is being used for two different axes
    
    x_act_track[i] = x_act.state # for plotting
    y_act_track[i] = y_act.state
    z_act_track[i] = z_act.state

    rocket.forces(force_vector, dt)
    
    sim_time += dt
    i+=1

# for s in debug[0:1000]:
#     print(s[0], "\tangle: ", s[1], "\tact.: ", s[2], "\tcmd: ", s[3] , "\ttime on: ", s[4], "\tontime: ", s[5])

print("Total Impulse: ", round(total_impulse,3), " N-s")

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
ax[2][0].set_ylabel("x-actuator state", fontsize=15)
ax[2][1].set_ylabel("y-actuator state", fontsize=15)
ax[2][2].set_ylabel("z-actuator state", fontsize=15)

for i in range(len(ax)):
    for j in range(len(ax[0])):
        ax[i][j].grid()
plt.tight_layout(h_pad=1, w_pad=2)
fig.savefig("controller/RCS_PID/python/figs/3DoF_States")
plt.show()
# -----------------------------------