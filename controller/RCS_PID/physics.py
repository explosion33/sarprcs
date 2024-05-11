
class oneDofPhysics():
    def __init__(self, initial_state_vector, mass, mmoi, r_z):
        self.state_vector = initial_state_vector
        self.mass = mass
        self.mmoi = mmoi
        self.rz = r_z
    
    def force(self, x_thrust, dt):
        self.state_vector

