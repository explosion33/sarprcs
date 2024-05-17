
class oneDofPhysics():
    def __init__(self, initial_state_vector, mmoi, r_z):
        self.state_vector = initial_state_vector
        self.mmoi = mmoi
        self.rz = r_z

    def nextState(self, x_thrust, dt):
        next_theta = self.state_vector[0] + self.state_vector[1]*dt + x_thrust*(0.5*self.rz/self.mmoi)*dt**2
        next_omega = self.state_vector[1] + x_thrust*(self.rz/self.mmoi)*dt
        self.state_vector = [next_theta, next_omega]
        
class threeDofPhysics():
    def __init__(self, initial_state_vector, mmoiX, mmoiZ, r_z, r_x):
        self.state_vector = initial_state_vector
        self.mmoiX = mmoiX
        self.mmoiZ = mmoiZ
        self.rz = r_z
        self.rx = r_x
        
    def forces(self, forces, dt):
        self.state_vector['thX'] = self.state_vector['thX'] \
            + self.state_vector['wX']*dt + forces['x_thrust']*(0.5*self.rz/self.mmoiX)*dt**2
        self.state_vector['wX'] = self.state_vector['wX'] \
            + forces['x_thrust']*(self.rz/self.mmoiX)*dt
        self.state_vector['thY'] = self.state_vector['thY'] \
            + self.state_vector['wY']*dt + forces['y_thrust']*(0.5*self.rz/self.mmoiX)*dt**2
        self.state_vector['wY'] = self.state_vector['wY'] \
            + forces['y_thrust']*(self.rz/self.mmoiX)*dt
        self.state_vector['wZ'] = self.state_vector['wZ'] \
            + 2*forces['z_thrust']*(self.rx/self.mmoiZ)*dt # force couple --> no resultant force, only moment
        
        