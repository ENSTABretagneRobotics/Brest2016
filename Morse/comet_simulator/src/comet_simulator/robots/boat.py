import logging
import morse.core.robot
from math import *
from morse.core import blenderapi
logger = logging.getLogger("morse." + __name__)


class Boat(morse.core.robot.Robot):
    _name = 'Boat robot'

    # Speed
    _v = 0

    # u1 (acceleration)
    vX = 0
    # u2 (rudder)
    wZ = 0

    _first_run = True

    x = 0.0
    y = 0.0
    z = 0.0
    yaw = 0.0
    roll = 0.0
    pitch = 0.0

    dyaw = 0.0

    last_time = 0.0  # time in s

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        morse.core.robot.Robot.__init__(self, obj, parent)
        # Convert Coordinate to NED
        # self.x = self.position_3d.x
        # self.y = -self.position_3d.y
        # self.z = -self.position_3d.z

        # self.yaw = -self.position_3d.yaw
        # self.roll = self.position_3d.roll
        # self.pitch = -self.position_3d.pitch

        self.x = self.position_3d.x
        self.y = self.position_3d.y
        self.z = self.position_3d.z

        self.yaw = self.position_3d.yaw
        self.roll = self.position_3d.roll
        self.pitch = self.position_3d.pitch

        self.last_time = self.gettime()

    def default_action(self):

        ###############################  DERIVATE ########################
        current_time = self.gettime()
        dt = (current_time - self.last_time)
        self.last_time = current_time

        v = self.vX

        dX = v * cos(self.yaw)
        dY = v * sin(self.yaw)
        dZ = 0.0
        self.dyaw = v * self.wZ * dt
        self._v = v

        ###############################  EULER    ########################

        self.yaw = atan(tan((self.yaw + self.dyaw * dt) / 2.0)) * 2.0
        self.x = self.x + dX * dt
        self.y = self.y + dY * dt

        #################### Apply Movement ####################
        # Convert NED to Coordinate
        # self.bge_object.worldPosition = [self.x, -self.y, -self.z]
        # self.bge_object.localOrientation = [
        #     self.roll, -self.pitch, -self.yaw]  # x, y, z = roll, pitch, yaw
        self.bge_object.worldPosition = [self.x, self.y, self.z]
        self.bge_object.localOrientation = [
            self.roll, self.pitch, self.yaw]  # x, y, z = roll, pitch, yaw

        # Attach the camera to the movment of the boat
        # if self._first_run == True:
        #     blenderapi.scene().objects['CameraFP'].setParent(
        #         self.bge_object, 0, 0)
        #     self._first_run = False

        pass
