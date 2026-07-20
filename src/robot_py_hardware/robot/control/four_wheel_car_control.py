import sys
if sys.platform == "linux":
    import RPi.GPIO as GPIO
else:
    GPIO = None
from robot.hardware.motors import MotorGroup, Motor
from robot.utils.config import FourWheelsMotorConfig, MotorSide, MotorPins

class FourWheelCarControl:
    """
    Provides an interface for controlling a basic four wheel (and motor) car with common motor drivers like the TB6612FNG.
    Each motor has three pins: PWM, IN1 and IN2. The motor drivers share one STBY pin.
    """
    def __init__(
        self, 
        stby: int,
        pwma_l: int, ain1_l: int, ain2_l: int, 
        pwmb_l: int, bin1_l: int, bin2_l: int, 
        pwma_r: int, ain1_r: int, ain2_r: int, 
        pwmb_r: int, bin1_r: int, bin2_r: int,
        pwm_frequency: float = 1000.0,
        base_speed : float = 100.0, 
        speed_step: float = 0.1,
        steering_step: float = 0.2,
    ):
        """
        Args:
            stby:           stby pin shared by all motor drivers
            pwma_l:         pwm pin of front left motor
            ain1_l:         pin of front left motor
            ain2_l:         in2 pin of front left motor
            pwmb_l:         pwm pin of rear left motor
            bin1_l:         in1 pin of rear left motor
            bin2_l:         in2 pin of rear left motor
            pwma_r:         pwm pin of front right motor
            ain1_r:         in1 pin of front right motor
            ain2_r:         in2 pin of front right motor
            pwmb_r:         pwm pin of rear right motor
            bin1_r:         in1 pin of rear right motor
            bin2_r:         in2 pin of rear right motor
            pwm_frequency:  pulse width modulation frequency in Hz
            base_speed:     defines max duty cycles for motors, between 0 and 100
            speed_step:     defines effect of single speed increase/decrease, relative, between 0 and 1
            steering_step:  defines effect of single steering increase/decrease, relative, between 0 and 1
        """
        self.base_speed = base_speed
        self._speed = 0.0
        self._steering = 0.0
        self.speed_step = speed_step
        self.steering_step = steering_step

        self._motors_left = MotorGroup(
            [
                Motor(pwma_l, pwm_frequency, ain1_l, ain2_l), 
                Motor(pwmb_l, pwm_frequency, bin1_l, bin2_l)
            ],
            self._base_speed
        )
        self._motors_right = MotorGroup(
            [
                Motor(pwma_r, pwm_frequency, ain1_r, ain2_r), 
                Motor(pwmb_r, pwm_frequency, bin1_r, bin2_r)
            ],
            self._base_speed
        )
        self._stby = stby
        GPIO.setup(self._stby, GPIO.OUT)
        
        self._active = True
        self._allow_forward = True
        self._allow_backward = True
        self.start()

    @classmethod
    def from_config(
        cls, 
        cfg: FourWheelsMotorConfig,
        pwm_frequency: float = 1000.0,
        base_speed : float = 100.0, 
        speed_step: float = 0.1,
        steering_step: float = 0.2,
    ):
        """
        Creates an instance using a motor config. 

        Args:
            cfg:            config containing all pin positions
            pwm_frequency:  pulse width modulation frequency in Hz
            base_speed:     defines max duty cycles for motors, between 0 and 100
            speed_step:     defines effect of single speed increase/decrease, relative, between 0 and 1
            steering_step:  defines effect of single steering increase/decrease, relative, between 0 and 1
        """
        return cls(
            cfg.stby,
            cfg.left.front.pwm, cfg.left.front.in1, cfg.left.front.in2,
            cfg.left.rear.pwm,  cfg.left.rear.in1,  cfg.left.rear.in2,
            cfg.right.front.pwm, cfg.right.front.in1, cfg.right.front.in2,
            cfg.right.rear.pwm,  cfg.right.rear.in1,  cfg.right.rear.in2,
            pwm_frequency, base_speed, speed_step, steering_step
        )

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------
    
    @property
    def base_speed(self) -> float:
        """
        Base speed which corresponds to the max duty cycles of the motors.
        This value is multiplied with the relative speed (-1.0 to 1.0) of the corresponding motor.
        """
        return self._base_speed
    
    @base_speed.setter
    def base_speed(self, value: float) -> None:
        if value < 0.0 or value > 100.0:
            raise ValueError("base_speed must be in range (0.0, 100.0)")
        self._base_speed = value

    @property
    def active(self) -> bool:
        """
        True if the motors are currently active, False otherwise
        """
        return self._active

    @property
    def speed(self) -> float:
        """
        Current forward/backward speed of car
        """
        return self._speed
    
    @property
    def steering(self) -> float:
        """
        Current left/right steering strength: negative for steering to the right, positive for steering to the left
        """
        return self._steering
    
    @property
    def speed_step(self) -> float:
        """
        Current speed increase/decrease step size relative to the maximum speed, between 0.0 and 1.0
        """
        return self._speed_step
    
    @speed_step.setter
    def speed_step(self, value: float) -> None:
        if value < 0.0 or value > 1.0:
            raise ValueError("Speed step size must be between 0 (no increment) and 1 (maximum speed increment)")
        self._speed_step = value
    
    @property
    def steering_step(self) -> float:
        """
        Current steering increase/decrease step size relative to the maximum steering, between 0.0 and 1.0
        """
        return self._steering_step
    
    @steering_step.setter
    def steering_step(self, value: float) -> None:
        if value < 0.0 or value > 1.0:
            raise ValueError("Steering step size must be between 0 (no increment) and 1 (maximum steering increment)")
        self._steering_step = value
    
    @property
    def allow_forward(self) -> bool:
        """
        True if car is allowed forward movements, False otherwise.
        If set to false, the car will not perform any forward movements.
        """
        return self._allow_forward
    
    @allow_forward.setter
    def allow_forward(self, value: bool) -> None:
        """
        Sets whether the car is allowed to perform forward movements.
        """
        self._allow_forward = value
    
    @property
    def allow_backward(self) -> bool:
        """
        True if car is allowed backward movements and False otherwise.
        If set to false, the car will not perform any backward movements.
        """
        return self._allow_backward
    
    @allow_backward.setter
    def allow_backward(self, value: bool) -> None:
        """
        Sets whether the car is allowed to perform backward movements.
        """
        self._allow_backward = value

    # ------------------------------------------------------------------
    # Start and stop functionality
    # ------------------------------------------------------------------

    def start(self) -> None:
        """
        Activate motor drivers by setting a high signal on the stby pin
        """
        if GPIO: 
            GPIO.output(self._stby, True)
        self._active = True

    def stop(self) -> None:
        """
        Set car into stby mode, stopping completely until start() is called
        """
        if GPIO:
            GPIO.output(self._stby, False)
        self._active = False

    # ------------------------------------------------------------------
    # Speed and steering alteration
    # ------------------------------------------------------------------

    def set_speed(self, speed: float) -> float:
        """
        Sets speed to specified value. Clamps speed to (-1.0, 1.0)

        Returns the applied speed value after clamping
        """
        self._speed = self._clamp_speed(speed)
        self._update_motor_speed()
        return self._speed

    def change_speed(self, change: float) -> float:
        """
        Changes speed by specified value. Clamps speed to (-1.0, 1.0)

        Returns the applied speed value after clamping
        """
        self._speed = self._clamp_speed(self._speed + change)
        self._update_motor_speed()
        return self._speed

    def set_steering(self, steering: float) -> float:
        """
        Sets steering to specified value. Clamps steering to (-1.0, 1.0)

        Returns the applied steering value after clamping
        """
        self._steering = self._clamp(steering, -1.0, 1.0)
        self._update_motor_speed()
        return self._steering

    def change_steering(self, change: float) -> float:
        """
        Changes steering by specified value. Clamps steering to (-1.0, 1.0)

        Returns the applied steering value after clamping
        """
        self._steering = self._clamp(self._steering + change, -1.0, 1.0)
        self._update_motor_speed()
        return self._steering

    # ------------------------------------------------------------------
    # Relative / incremental change of speed and steering (designed for key-press handlers)
    # ------------------------------------------------------------------

    def accelerate(self, step: float = None) -> float:
        """Increases speed according to specified step or current step size. Returns new speed."""
        return self.change_speed(step or self._speed_step)

    def decelerate(self, step: float = None) -> float:
        """Decreases speed according to specified step or current step size. Returns new speed."""
        return self.change_speed(-(step or self._speed_step))

    def turn_left(self, step: float = None) -> float:
        """Increases/decreases steering motion according to specified step or current step size. Returns new steering value."""
        return self.change_steering(step or self._steering_step)

    def turn_right(self, step: float = None) -> float:
        """Increases/decreases steering motion according to specified step or current step size. Returns new steering value."""
        return self.change_steering(-(step or self._steering_step))

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def cleanup(self) -> None:
        """
        Performs cleanup, stopping all motor pwms.
        """
        self._motors_left.stop()
        self._motors_right.stop()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _update_motor_speed(self) -> None:
        """
        Sets speed of left and right motor group with respect to desired speed and steering
        """
        rel_speed_left = self._clamp(self._speed - self._steering, -1.0, 1.0)
        rel_speed_right = self._clamp(self._speed + self._steering, -1.0, 1.0)
        self._motors_left.update_speed(self._base_speed * rel_speed_left)
        self._motors_right.update_speed(self._base_speed * rel_speed_right)

    def _clamp_speed(self, value: float) -> float:
        """
        Clamps the speed of the car to allowed range with respect to current restrictions on forward or backward movement

        Returns the clamped speed
        """
        # use type conversion from bool to allow/disallow forward/backward movement
        return self._clamp(value, -float(self._allow_backward), float(self._allow_forward))

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    # Potentially smoother mapping using exponential behavior for more fine-tuning near 0
    # def map_controls(v, s):
    #     # exponential response
    #     k = 0.5
    #     v = (1-k)*v + k*(v**3)
    #     s = (1-k)*s + k*(s**3)

    #     # speed-dependent steering
    #     s *= (1 - abs(v))

    #     # arcade mix
    #     left  = v + s
    #     right = v - s

    #     # normalize
    #     max_val = max(1, abs(left), abs(right))
    #     left  /= max_val
    #     right /= max_val

    #     return left, right
