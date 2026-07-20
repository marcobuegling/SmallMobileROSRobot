import sys
if sys.platform == "linux":
    import RPi.GPIO as GPIO
else:
    GPIO = None
import time
from collections import deque
from robot.utils.config import UltrasonicSensorConfig, BasicSensorConfig
from typing import List, get_args
from abc import ABC

class Sensor[T](ABC):
    """
    Abstract base class for sensor modules. Provides optional buffer functionality to store recent sensor values.
    """
    def __init__(self, buffer_size: int = 0) -> None:
        """
        Args:
            buffer_size: Number of most recent sensor values to be stored.
                         If set to 0, no buffer will be created.
        """
        self._buffer = deque[T](maxlen=buffer_size) if buffer_size > 0 else None

    @property
    def output_type(self) -> type:
        """Output type of the sensor."""
        return get_args(self._buffer)[0]
    
    def get_buffer_values(self) -> List[T]:
        """Returns all values currently stored in the buffer."""
        if self._buffer is None:
            raise ValueError("Sensor is not assigned a buffer")
        return list(self._buffer)
    
    def get_buffer_avg(self) -> float:
        """Calculate average of measurements stored in buffer."""
        if self._buffer is None:
            raise ValueError("Sensor is not assigned a buffer")
        if len(self._buffer) == 0:
            return 0.00
        return round(sum(self._buffer) / len(self._buffer), 2)
    

class BasicSensor(Sensor[bool]):
    """
    Class representing a simple sensor returning a high or low signal.
    Use this class for all sensor types that have only one signal pin and return 
    a clear high or low signal depending on whether detection was positive.
    This includes simple line trackers (e.g. HW-511 or similar) or 
    passive infrared (PIR) sensor modules (e.g. HW-416A or similar)
    as well as other sensor types.
    """
    def __init__(self, signal_pin: int, buffer_size: int = 0):
        """
        Args:
            signal_pin:  Input pin for the signal
            buffer_size: Number of most recent sensor values to be stored.
                         If set to 0, no buffer will be created.
        """
        super().__init__(buffer_size)
        self._signal = signal_pin
        GPIO.setup(signal_pin, GPIO.IN)

    @classmethod
    def from_config(cls, cfg: BasicSensorConfig, buffer_size: int = 0):
        return cls(cfg.signal, buffer_size)

    # Returns true if line detected, false otherwise
    def read_value(self) -> bool:
        """Reads the current signal, writes it to the buffer and returns it."""
        signal = bool(GPIO.input(self._signal))
        if not self._buffer is None:
            self._buffer.append(signal)
        return signal

    
class UltrasonicSensor(Sensor[float]):
    """
    Class representing an ultrasonic sensor module (HC-SR04 or similar).
    """
    def __init__(self, trig_pin: int, echo_pin: int, buffer_size: int = 0):
        """
        Args:
            signal_pin:  Input pin for the signal
            buffer_size: Number of most recent sensor values to be stored.
                         If set to 0, no buffer will be created.
        """
        super().__init__(buffer_size)
        self._trig = trig_pin
        self._echo = echo_pin
        GPIO.setup(self._trig, GPIO.OUT)
        GPIO.setup(self._echo, GPIO.IN)
        GPIO.output(self._trig, GPIO.LOW)

    @classmethod
    def from_config(cls, cfg: UltrasonicSensorConfig, buffer_size: int = 0):
        return cls(cfg.trig, cfg.echo, buffer_size)

    def read_value(self) -> float:
        """Performs a single measurement, calculates the distance,
            writes the distance to the buffer and returns it."""
        # Trigger pulse
        GPIO.output(self._trig, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self._trig, GPIO.LOW)

        # Measure time of pulse start and end
        pulse_start = time.time()
        pulse_end = time.time()
        while not GPIO.input(self._echo):
            pulse_start = time.time()
        while GPIO.input(self._echo):
            pulse_end = time.time()

        # Calculate distance based on ultrasonic travel time
        pulse_duration = pulse_end - pulse_start
        distance_cm = pulse_duration * 17150
        distance_cm = round(distance_cm, 2)
        if not self._buffer is None:
            self._buffer.append(distance_cm)
        return distance_cm
