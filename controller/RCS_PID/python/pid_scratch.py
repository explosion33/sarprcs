import math

# Units are SI units... m/s/radians/kg
# GLOABAL PARAMS
TIME_STEP = 0.01
SETPOINT = 0 # desired state
MAX_THRUST = 10 # <--test
MIN_THRUST = 1 # <--test
SIM_TIME = 1000 # seconds
INITIAL_X = -15 * 2*math.pi/360 # degrees to rad
THRUST_COEFF = 10 # <---TUNE (converts PID output to thrust command)
# -- PID GAINS --
Kp = 1
Ki = 1
Kd = 1
# ---------------

def getAngle():
	# from BNO055
	return 5

def getError():
	return getAngle() - SETPOINT

def get_controller_input():
	P = error*Kp
	I += sum(errors)
	D += (last_error - error) * (getTime() - last_time) * Kd
	return(P+I+D)

errors = []
time = [] # for plotting
time = 0 

while True:

	error = getError()
	
	errors.append(error)
	if len(errors) > 20:
		errors.remove(0)

	thrust_x = get_controller_input()
	
    # Saturation
	if thrust_x > MAX_THRUST: 
		thrust_x = MAX_THRUST
	if thrust_x < MIN_THRUST:
		thrust_x = MIN_THRUST

    # Direction (which x-thruster)
	if thrust_x < 0:
		x1_thrust_percent = THRUST_COEFF*abs(thrust_x) # 
	else:
		x2_thrust_percent = THRUST_COEFF*abs(thrust_x)
		
	last_time = getTime()
	last_error = error

    # ~~~ PLOTTING ~~~
	

	



    