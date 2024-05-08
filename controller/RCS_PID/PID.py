
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
    
    def compute(self,theta,dt):
        error = theta - self.setpoint # error will just be theta
        self.integral_error += error*dt # add error to integral
        P = -1*self.Kp*error
        I = -1*self.Ki*self.integral_error
        D = -1*self.Kd*( error - self.error_last )/dt
        output = P+I+D
        self.error_last = error

        # check saturation limit
        if self.max_thrust is not None:
            if output > self.max_thrust:
                output = self.max_thrust
            elif output < self.min_thrust:
                output = self.min_thrust
        return output
    
    def setLims(self,min,max):
        self.max_thrust = max
        self.min_thrust = min    
                
        