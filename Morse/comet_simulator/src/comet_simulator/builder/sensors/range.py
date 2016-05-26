from morse.builder.creator import SensorCreator

class Range(SensorCreator):
    _classpath = "comet_simulator.sensors.range.Range"
    _blendname = "range"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)

