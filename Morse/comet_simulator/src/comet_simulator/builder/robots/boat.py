from morse.builder import *
from comet_simulator.builder.actuators import *
from comet_simulator.builder.sensors import *

class Boat(GroundRobot):
    """
    A template robot model for boat, with a motion controller and a pose sensor.
    """
    def __init__(self, name = None, debug = True):

        # boat.blend is located in the data/robots directory
        GroundRobot.__init__(self, 'comet_simulator/robots/boat.blend', name)
        self.properties(classpath = "comet_simulator.robots.boat.Boat")

        ###################################
        # Actuators
        ###################################

        self.boatActuator = Boatactuator()
        self.boatActuator.frequency(10.0)
        self.boatActuator.add_stream('ros','comet_simulator.actuators.boatActuatorROS.BoatActuatorROS', topic='/'+ self.name +'/actuator')

        self.append(self.boatActuator)

        ###################################
        # Sensors
        ###################################

        self.boatGPS = Boatgps()
        self.boatGPS.frequency(7.0)
        self.boatGPS.add_stream('ros','comet_simulator.sensors.boatGPSROS.BoatGPSROS_velocity', topic='/'+ self.name +'/gps/velocity')
        self.boatGPS.add_stream('ros','comet_simulator.sensors.boatGPSROS.BoatGPSROS_pose', topic='/'+ self.name +'/gps/pose')

        self.boatCompas = Boatcompas()
        self.boatCompas.frequency(7.0)
        self.boatCompas.add_stream('ros', 'comet_simulator.sensors.boatCompasROS.BoatCompasROS', topic='/'+ self.name +'/compas')
        
        self.append(self.boatGPS)
        self.append(self.boatCompas)

