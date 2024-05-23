
class PID():
    def __init__(self,KP,KI,KD,target=0) -> None:
        self.Kp = KP
        self.Ki = KI
        self.Kd = KD
        self.setpoint = target
        self.error_last = 0
        self.integral_error = 0
    
    def compute(self,theta:float,dt:float) -> float: #idk if type specification is useful
        '''
        Args: 
        theta - angle (rad) of rocket: float
        dt - time step (sec) since last computation: float
        
        Returns:
        output: commanded actuation time (how long the solenoid should be open).
                Sign denotes which solenoid to fire on the actuator's axis
        '''
        error = theta - self.setpoint # error will just be theta
        self.integral_error += error*dt # sum all errors*dt
        P = -1*self.Kp*error
        I = -1*self.Ki*self.integral_error
        D = -1*self.Kd*( error - self.error_last )/dt
        output = P+I+D
        self.error_last = error
    
        return output  
                
        