import maestro as m

servo = m.Controller()

servo.setTarget(1, 6000)

servo.setTarget(2, 6000)

servo.close()
