from morse.builder import *
from comet_simulator.builder.actuators import *
from comet_simulator.builder.sensors import *

class Auv(GroundRobot):
    """
    A template robot model for auv, with a motion controller and a pose sensor.
    """
    def __init__(self, name = None, debug = True):

        # auv.blend is located in the data/robots directory
        GroundRobot.__init__(self, 'comet_simulator/robots/auv.blend', name)
        self.properties(classpath = "comet_simulator.robots.auv.Auv")

        ###################################
        # Actuators
        ###################################

        self.Actuator = Auvactuator()
        self.Actuator.frequency(25.0)
        self.Actuator.add_stream('ros','comet_simulator.actuators.auvActuatorROS.AuvActuatorROS', topic='/'+ self.name +'/actuator')

        self.append(self.Actuator)

        ###################################
        # Sensors
        ###################################

        self.IMU = Imu()
        self.IMU.frequency(25.0)
        self.IMU.add_stream('ros','comet_simulator.sensors.imuROS.ImuROS', topic='/'+ self.name + '/imu')

        self.DVL = Dvl()
        self.DVL.frequency(10.0)
        self.DVL.add_stream('ros','comet_simulator.sensors.dvlROS.DvlROS', topic='/'+ self.name +'/dvl')

        self.Pressure = Pressure()
        self.Pressure.frequency(10.0)
        self.Pressure.add_stream('ros','comet_simulator.sensors.pressureROS.PressureROS', topic='/'+ self.name +'/pressure')

        self.Range = Range()
        self.Range.frequency(1.0)
        self.Range.add_stream('ros','comet_simulator.sensors.rangeROS.RangeROS', topic='/'+ self.name +'/range')
        
        self.GPS = Gps()
        self.Range.frequency(1.0)
        self.Range.add_stream('ros','comet_simulator.sensors.rangeROS.RangeROS', topic='/'+ self.name +'/gps')

        self.append(self.IMU)
        self.append(self.DVL)
        self.append(self.Pressure)
        self.append(self.Range)
        self.append(self.GPS)

        # Debug
        self.boatGPS = Boatgps()
        self.boatGPS.frequency(7.0)
        self.boatGPS.add_stream('ros','comet_simulator.sensors.boatGPSROS.BoatGPSROS_pose', topic='/'+ self.name +'/debug/pose')
        self.append(self.boatGPS)

