from morse.builder.creator import ActuatorCreator

class Auvactuator(ActuatorCreator):
    _classpath = "comet_simulator.actuators.auvActuator.Auvactuator"
    _blendname = "auvActuator"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

