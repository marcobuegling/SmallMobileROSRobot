import sys
if sys.platform == "linux":
    import RPi.GPIO as GPIO
else:
    GPIO = None

class Motor:
    """
    Class defining a single motor using the corresponding output pins.
    This class provides basic functionality for speed control of the
    corresponding motor.
    """
    def __init__(self, pwm: int, pwm_frequency: float, in1: int, in2: int):
        self._in1 = in1
        self._in2 = in2
        GPIO.setup(pwm, GPIO.OUT)
        GPIO.setup(in1, GPIO.OUT)
        GPIO.setup(in2, GPIO.OUT)
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        self._pwm = GPIO.PWM(pwm, pwm_frequency)
        self._speed = 0.0
        self.start()

    @property
    def speed(self) -> float:
        """Current speed the motor runs at."""
        return self._speed

    def update_speed(self, speed: float):
        """
        Updates speed of motor to fixed value considering forward vs. backward driving.
        Speed has to be in range (-100, 100), with -100 meaning full speed backwards.
        """
        self._speed = speed
        if self._speed > 0:
            GPIO.output(self._in1, GPIO.HIGH)
            GPIO.output(self._in2, GPIO.LOW)
        elif self._speed < 0:
            GPIO.output(self._in1, GPIO.LOW)
            GPIO.output(self._in2, GPIO.HIGH)
        else:
            GPIO.output(self._in1, GPIO.LOW)
            GPIO.output(self._in2, GPIO.LOW)
        self._pwm.ChangeDutyCycle(abs(self._speed))

    def change_speed(self, value: float):
        """
        Updates speed of motor by a given value considering forward vs. backward driving.
        Speed has to be in range (-100, 100), with -100 meaning full speed backwards.
        """
        self.update_speed(self._speed + value)

    def start(self):
        """Starts motors pwm."""
        self._pwm.start(0)

    def stop(self):
        """Stops motors pwm, stopping the motor entirely."""
        self._pwm.stop()
    

class MotorGroup:
    """
    Class representing a group of motors that should be controlled together, e.g. the 
    two motors on one side of a four wheel robot. 
    This class provides an interface for controlling the motors, such that the single
    motors do not need to be controlled separately.
    """
    def __init__(self, motors: list[Motor], max_speed: float = 100.0):
        if len(motors) > 0:
            self._motors = motors
        else:
            self._motors = []
        for m in self._motors:
            m.update_speed(0)
        self._speed = 0
        self._max_speed = max_speed

    @property
    def speed(self) -> float: 
        """The current speed of the motors in the group."""
        return self._speed
    
    def add_motor(self, motor: Motor):
        """Adds a motor to the group and updates its speed to match the group."""
        self._motors.append(motor)
        motor.update_speed(self._speed)

    def update_speed(self, speed: float):
        """
        Updates speed of all motors in the group.
        Speed has to be in range (-max_speed, max_speed), with -max_speed meaning maximum speed backwards.
        """
        self._speed = max(-self._max_speed, min(speed, self._max_speed))
        for m in self._motors:
            m.update_speed(self._speed)

    def start(self):
        """Starts motors pwm."""
        for m in self._motors:
            m.start()

    def stop(self):
        """Stops all motors entirely by stopping pwm."""
        for m in self._motors:
            m.stop()
    
