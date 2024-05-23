
def getSign(cmd): # REPLACE
        if cmd == 0:
            return 0
        return cmd/abs(cmd)


def getActState(sim_time, command, on, time_on, ontime, min_ontime):
    '''Manages the state of the solenoid'''
    if on:
        ontime = sim_time - time_on # track ontime
    else:
        ontime = 0
        
    if not on and command != 0:
        on = True
        time_on = sim_time # mark when solenoid was turned ON

    if on and ontime > abs(command) and ontime >= min_ontime:
        # if it has been longer than commanded time, turn back off
        on = False
    
    return [on, time_on, ontime]

def getActState(sim_time, command, act_state, time_on, ontime, min_act_time):
    '''
    Manages the state of the solenoids controlling motion on an axis
    
    command: array of [commanded state, commanded time]
    act_state: current state of the actuator: -1, 0, or 1
    time_on: when the actuator was first commanded to be in the current state
    ontime: how long the actuator has been in the current state
    min_act_time: minimum time the actuator has to wait between switching states
    '''
    if act_state == command[0]:
        ontime = sim_time - time_on # track ontime
    else:
        ontime = 0 # reset ontime if a different command is sent
        act_state = command[0]
        time_on = sim_time # mark when solenoid was turned ON (positive or negative)

    if ontime > abs(command[1]) and ontime >= min_act_time:
        # if it has been longer than commanded time, turn back off
        act_state = 0
    
    return [act_state, time_on, ontime]

