
class PID():
    def __init__(self,KP,KI,KD,target=0) -> None:
        self.Kp = KP
        self.Ki = KI
        self.Kd = KD
        self.setpoint = target
        self.error_last = 0
        self.integral_error = 0
        self.max_thrust = None
        self.min_thrust = None
    
    def compute(self,theta:float,dt:float) -> float: #idk if type specification is useful
        '''
        Args: 
        theta - angle (rad) of rocket: float
        dt - time step (sec) since last computation: float
        Output: 
        PID control command (newtons): float
        '''
        error = theta - self.setpoint # error will just be theta
        self.integral_error += error*dt # sum all errors*dt
        P = -1*self.Kp*error
        I = -1*self.Ki*self.integral_error
        D = -1*self.Kd*( error - self.error_last )/dt
        output = P+I+D
        self.error_last = error
        if self.max_thrust is not None: # check for saturation
            if output > 0: # positive direction
                if output > self.max_thrust:
                    output = self.max_thrust
                elif output < self.min_thrust:
                    output = self.min_thrust
            else: # negative direction
                if abs(output) > self.max_thrust:
                    output = -1*self.max_thrust
                elif abs(output) < self.min_thrust:
                    output = -1*self.min_thrustf
        return output
    
    def setLims(self,min,max):
        ''' Sets minimum & maximum thrust in newtons'''
        self.max_thrust = max
        self.min_thrust = min    
                
        