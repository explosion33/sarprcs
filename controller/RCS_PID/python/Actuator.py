
class Actuator():
    def __init__(self, min_act_time) -> None:
        '''
        state: current state of the actuator...
            -1: negative solenoid on, positive solenoid off
             0: both solenoids off
             1: positive solenoid on, negative solenoid off
        
        time_on: when the actuator was first commanded to be in the current state
        
        ontime: how long the actuator has been in the current state
        
        min_act_time: minimum time the actuator has to wait between switching states
        '''
        self.min_act_time = min_act_time
        self.state = 0
        self.time_on = 0
        self.ontime = 0
        
    def getState(self, time, cmd):
        '''
        Manages the state of the solenoids controlling motion on an axis
        
        time: current time
        
        cmd (array): [commanded state, commanded time]
            commanded state is either...
            -1: negative solenoid on, positive solenoid off
             0: both solenoids off
             1: positive solenoid on, negative solenoid off
            commanded time is how long the commanded state should be held in seconds
            
        '''
        
        self.ontime = time - self.time_on # track ontime
        
        if self.state != cmd[0] and self.ontime >= self.min_act_time:
            self.ontime = 0 # reset ontime if a different command is sent
            self.state = cmd[0]
            self.time_on = time # mark when solenoid was turned ON (positive or negative)

        if self.ontime > cmd[1] and self.ontime >= self.min_act_time:
            # if it has been longer than the commanded time, turn back off
            self.time_on = time # time_on is actually time_switch
            self.ontime = 0
            self.state = 0
        
        return self.state, self.time_on, self.ontime