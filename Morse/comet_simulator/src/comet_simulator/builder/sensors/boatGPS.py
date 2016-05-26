from morse.builder.creator import SensorCreator

class Boatgps(SensorCreator):
    _classpath = "comet_simulator.sensors.boatGPS.Boatgps"
    _blendname = "boatGPS"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)

