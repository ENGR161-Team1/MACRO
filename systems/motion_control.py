from buildhat import Motor, MotorPair

class MotionController:
  def __init__(self, **kwargs):
    motors = MotorPair(kwargs["motors"][0], kwargs["motors"][1])
    motorLeft = Motor(kwargs["motors"][0])
    motorRight = Motor(kwargs["motors"][1])
    max_speed = kwargs.get("max_speed", 30)
  
