from pathlib import Path
import yaml
from pydantic import BaseModel

# --- Model definitions mirroring the structure defined in config ---

class MotorPins(BaseModel):
    pwm: int
    in1: int
    in2: int

class MotorSide(BaseModel):
    front: MotorPins
    rear: MotorPins

class FourWheelsMotorConfig(BaseModel):
    left: MotorSide
    right: MotorSide
    stby: int

class ControlConfig(BaseModel):
    max_speed: float
    acceleration_sensitivity: float
    steering_sensitivity: float
    pwm_frequency: float

class CameraServoConfig(BaseModel):
    channels: int
    address: int  # hex values like 0x40 are parsed as ints automatically

class UltrasonicSensorConfig(BaseModel):
    trig: int
    echo: int

class BasicSensorConfig(BaseModel):
    signal: int

class RobotConfig(BaseModel):
    robot_type: str
    motors: FourWheelsMotorConfig
    control: ControlConfig
    ultrasonic: UltrasonicSensorConfig
    line: BasicSensorConfig


# --- Loader function ---

def load_config(path: str | Path = "test_config.yaml") -> RobotConfig:
    with open(path) as f:
        raw = yaml.safe_load(f)
    return RobotConfig(**raw)
