from adafruit_servokit import ServoKit


class PanTiltUnit:
    """
    Interface for a two-axis pan-tilt unit driven by a PCA9685-based servo controller.

    Axis conventions:
        Pan  (horizontal) - channel 0 by default, 0° = full left, 180° = full right
        Tilt (vertical)   - channel 1 by default, 0° = full down, 180° = full up

    Mechanical soft limits (lo/hi) keep the servos away from their physical
    endpoints to avoid binding.  The defaults (10°-170°) match the test script;
    tighten them if your rig has less clearance.
    """

    DEFAULT_PAN_CH  = 0
    DEFAULT_TILT_CH = 1

    # Servo hardware calibration defaults (safe starting point for micro servos)
    ACTUATION_RANGE    = 180   # degrees; try 160 if your servo has smaller travel
    PULSE_WIDTH_MIN_US = 600   # microseconds
    PULSE_WIDTH_MAX_US = 2400

    def __init__(
        self,
        i2c_address: int   = 0x40,
        pan_ch:  int       = DEFAULT_PAN_CH,
        tilt_ch: int       = DEFAULT_TILT_CH,
        pan_lo:  float     = 0.0,
        pan_hi:  float     = 180.0,
        tilt_lo: float     = 0.0,
        tilt_hi: float     = 180.0,
        step:    float     = 5.0,
    ):
        """
        Args:
            i2c_address: I2C address of the PCA9685 board (default 0x40;
                         run `i2cdetect -y 1` to verify).
            pan_ch:      Servo channel for pan axis.
            tilt_ch:     Servo channel for tilt axis.
            pan_lo:      Minimum allowed pan angle (soft limit).
            pan_hi:      Maximum allowed pan angle (soft limit).
            tilt_lo:     Minimum allowed tilt angle (soft limit).
            tilt_hi:     Maximum allowed tilt angle (soft limit).
            step:        Default degrees moved per discrete step (used by
                         the step_* methods intended for key-press handling).
        """
        kit = ServoKit(channels=16, address=i2c_address)

        self._pan_servo  = kit.servo[pan_ch]
        self._tilt_servo = kit.servo[tilt_ch]

        for servo in (self._pan_servo, self._tilt_servo):
            servo.actuation_range = self.ACTUATION_RANGE
            servo.set_pulse_width_range(self.PULSE_WIDTH_MIN_US, self.PULSE_WIDTH_MAX_US)

        self._pan_lo  = pan_lo
        self._pan_hi  = pan_hi
        self._tilt_lo = tilt_lo
        self._tilt_hi = tilt_hi
        self.step    = step

        # Track current angles internally
        self._pan_angle  = 90.0
        self._tilt_angle = 90.0

        self.center()

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def pan_angle(self) -> float:
        """Current pan angle in degrees."""
        return self._pan_angle

    @property
    def tilt_angle(self) -> float:
        """Current tilt angle in degrees."""
        return self._tilt_angle

    @property
    def step(self) -> float:
        """Degrees moved per discrete step."""
        return self._step

    @step.setter
    def step(self, value: float) -> None:
        if value <= 0:
            raise ValueError("step must be positive")
        self._step = value

    # ------------------------------------------------------------------
    # Absolute positioning
    # ------------------------------------------------------------------

    def set_pan(self, angle: float) -> float:
        """
        Move the pan axis to `angle`, clamped to [pan_lo, pan_hi].

        Returns the angle actually applied (after clamping).
        """
        angle = self._clamp(angle, self._pan_lo, self._pan_hi)
        self._pan_servo.angle = angle
        self._pan_angle = angle
        return angle

    def set_tilt(self, angle: float) -> float:
        """
        Move the tilt axis to `angle`, clamped to [tilt_lo, tilt_hi].

        Returns the angle actually applied (after clamping).
        """
        angle = self._clamp(angle, self._tilt_lo, self._tilt_hi)
        self._tilt_servo.angle = angle
        self._tilt_angle = angle
        return angle

    def set_position(self, pan: float, tilt: float) -> tuple[float, float]:
        """
        Move both axes simultaneously.

        Returns (pan, tilt) angles actually applied (after clamping).
        """
        return self.set_pan(pan), self.set_tilt(tilt)

    def center(self) -> None:
        """Move both axes to 90° (mechanical centre)."""
        self.set_pan(90.0)
        self.set_tilt(90.0)

    # ------------------------------------------------------------------
    # Relative / incremental movement  (designed for key-press handlers)
    # ------------------------------------------------------------------

    def step_left(self, step: float = None) -> float:
        """Decrease pan angle by `step` degrees. Returns new pan angle."""
        return self.set_pan(self._pan_angle - (step or self._step))

    def step_right(self, step: float = None) -> float:
        """Increase pan angle by `step` degrees. Returns new pan angle."""
        return self.set_pan(self._pan_angle + (step or self._step))

    def step_up(self, step: float = None) -> float:
        """Increase tilt angle by `step` degrees. Returns new tilt angle."""
        return self.set_tilt(self._tilt_angle + (step or self._step))

    def step_down(self, step: float = None) -> float:
        """Decrease tilt angle by `step` degrees. Returns new tilt angle."""
        return self.set_tilt(self._tilt_angle - (step or self._step))

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))
