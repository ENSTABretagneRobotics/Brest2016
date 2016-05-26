from morse.builder.creator import SensorCreator

class Gps(SensorCreator):
    _classpath = "comet_simulator.sensors.gps.Gps"
    _blendname = "gps"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)

