from morse.builder.creator import SensorCreator

class Pressure(SensorCreator):
    _classpath = "comet_simulator.sensors.pressure.Pressure"
    _blendname = "pressure"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)

