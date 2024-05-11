
class oneDofPhysics():
    def __init__(self, initial_state_vector, mmoi, r_z):
        '''
        state vector: [theta, omega]
        '''
        self.state_vector = initial_state_vector
        self.mmoi = mmoi
        self.rz = r_z
    
    def nextState(self, x_thrust, dt):
        next_theta = self.state_vector[0] + self.state_vector[1]*dt + x_thrust*(0.5*self.rz/self.mmoi)*dt**2
        next_omega = self.state_vector[1] + x_thrust*(self.rz/self.mmoi)*dt
        self.state_vector = [next_theta, next_omega]
