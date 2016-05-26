from morse.builder.creator import SensorCreator

class Boatcompas(SensorCreator):
    _classpath = "comet_simulator.sensors.boatCompas.Boatcompas"
    _blendname = "boatCompas"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)

