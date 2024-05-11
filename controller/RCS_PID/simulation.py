from PID import PID 
from physics import oneDofPhysics

# --INITIALIZE STATE--
theta0 = -1 # rad
omega0 = -3 # rad/s
initial_state = [theta0, omega0]
# --------------------

# --SIM PARAMS-- (units in N, m, kg, s, etc.) 
dt = 0.01 
mmoi = 100
rz = 5 # vertical distance from COM to thruster 
sim_time = 0
time_limit = 10
# --------------

# --CONTROLLER PARAMS--
Kp = 100
Ki = 0.1
Kd = 0.1
min_thrust = 5
max_thrust = 50
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
    print("theta: ", rocket.state_vector[0],"\tomega: ", rocket.state_vector[1], "\tcommand: ",command)
    rocket.nextState(command, dt)
    sim_time += dt
    i+=1

# for i,j in zip(thetas, commands):
#     print("theta: ", i, "\tcommand: ",j)


# # --PLOTTING--
# time = range(0, time_limit, dt)
# fig, ax = plt.subplots(2)
# ax[0].plot(time, thetas)
# ax[1].plot(time, omegas)