import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.robot
from math import *

alpha_v     = 0.1
beta_v      = 0.7
gamma_psi   = 0.0666667
delta_theta = 0.01
dzeta_phi   = 1.0
eta_phi     = 0.1

# alpha_v     = 0.285714
# beta_v      = 14
# gamma_psi   = 1
# delta_theta = 1
# dzeta_phi   = 1
# eta_phi     = 0.1

class Auv(morse.core.robot.Robot):
    _name = 'Scout robot'

    # speed
    _v = 0.0

    #commands
    vX = 0.0 # propulseur arriere (v) entre -1 et 1        # linear.x
    gH = 0.0 # gouverne horizontale (self.pitch) idem      # angular.y
    gV = 0.0# gouverne verticale (self.yaw) idem           # angular.z

    yaw = 0.0
    pitch = 0.0
    roll = 0.0

    x = 0.0
    y = 0.0
    z = 0.0

    last_time = 0.0

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        morse.core.robot.Robot.__init__(self, obj, parent)

        # Convert Coordinate to NED
        self.x = self.position_3d.x
        self.y = -self.position_3d.y
        self.z = -self.position_3d.z
        
        self.yaw = -self.position_3d.yaw
        self.roll = self.position_3d.roll
        self.pitch = -self.position_3d.pitch

        self.last_time = self.gettime()

    def default_action(self):
        current_time = self.gettime()
        dt = (current_time - self.last_time)
        self.last_time = current_time
        v = self._v
        pX = self.vX
        gH = self.gH
        gV = self.gV

        # Physical constraints
        if(abs(gV) > pi/3.0):
            gV = copysign(pi/3.0, gV)
        if(abs(gH) > pi/3.0):
            gH = copysign(pi/3.0, gH)
        if(pX > 4.0):
            pX = 4.0
        if(pX < 0.0):
            pX = 0.0

        dX = v*cos(self.yaw)*cos(self.pitch)
        dY = v*sin(self.yaw)*cos(self.pitch)
        dZ = -v*sin(self.pitch) # add flotability ?
        
        dV = beta_v*pX - alpha_v*abs(v)*v      
        
        dRoll = 0.0                          # roll : ToDo improve the model
        dPitch = copysign(sin(gH)*sin(gH)*v*0.1, gH)      # pitch
        dYaw = copysign(sin(gV)*sin(gV)*v*0.7, gV)        # yaw

        #################### Euler Integration ####################
        # + fonction dent de scie
        self.yaw = atan(tan((self.yaw + dYaw * dt)/2.0))*2.0         
        self.pitch = atan(tan((self.pitch + dPitch * dt)/2.0))*2.0
        self.roll = atan(tan((self.roll + dRoll * dt)/2.0))*2.0

        self.x = self.x + dX * dt
        self.y = self.y + dY * dt
        self.z = self.z + dZ * dt

        self._v = v + dV * dt

        #################### Apply Movement ####################
        # Convert NED to Coordinate
        self.bge_object.worldPosition = [self.x, -self.y, -self.z]
        self.bge_object.localOrientation = [self.roll, -self.pitch, -self.yaw] # x, y, z = roll, pitch, yaw

        pass







