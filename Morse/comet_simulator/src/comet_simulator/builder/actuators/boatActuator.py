from morse.builder.creator import ActuatorCreator

class Boatactuator(ActuatorCreator):
    _classpath = "comet_simulator.actuators.boatActuator.Boatactuator"
    _blendname = "boatActuator"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

