
class PID():
    def __init__(self,KP,KI,KD,target=0) -> None:
        self.Kp = KP
        self.Ki = KI
        self.Kd = KD
        self.setpoint = target
        self.error_last = 0
        self.integral_error = 0
        # self.min_act_time = None
    
    def compute(self,theta:float,dt:float) -> float: #idk if type specification is useful
        '''
        Args: 
        theta - angle (rad) of rocket: float
        dt - time step (sec) since last computation: float
        
        Returns:
        cmd: 
        output: commanded actuation time (how long the solenoid should be open)
        '''
        error = theta - self.setpoint # error will just be theta
        self.integral_error += error*dt # sum all errors*dt
        P = -1*self.Kp*error
        I = -1*self.Ki*self.integral_error
        D = -1*self.Kd*( error - self.error_last )/dt
        output = P+I+D
        self.error_last = error
        # if self.min_act_time is not None:
        #     if abs(output) < self.min_act_time:
        #         output = 0
        
        if output > 0:
            cmd = 1
        elif output < 0:
            cmd = -1
        else:
            cmd = 0
        
        return cmd, abs(output)
    
    def setMinAct(self, min): # Move saturation limits outside PID class?
        ''' Sets minimum actuation for the solenoid in seconds'''
        self.min_act_time = min    
                
        