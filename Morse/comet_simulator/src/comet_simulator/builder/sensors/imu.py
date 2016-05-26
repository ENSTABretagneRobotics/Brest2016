from morse.builder.creator import SensorCreator

class Imu(SensorCreator):
    _classpath = "comet_simulator.sensors.imu.Imu"
    _blendname = "imu"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)

