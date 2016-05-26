from morse.builder.creator import SensorCreator

class Dvl(SensorCreator):
    _classpath = "comet_simulator.sensors.dvl.Dvl"
    _blendname = "dvl"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)

