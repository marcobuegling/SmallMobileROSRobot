import sys
if sys.platform == "linux":
    import RPi.GPIO as GPIO
else:
    GPIO = None
from pathlib import Path

from robot.robots.registry import REGISTRY
from robot.utils.config import load_config

def launch(config_file: str):
    """
    Launches .yaml file from /config.
    """
    config_path = Path(__file__).parent.parent.parent.parent / "config" / config_file
    cfg = load_config(config_path)

    # Refer to pins by the Broadcom SOC channel (aka GPIO) numbering
    GPIO.setmode(GPIO.BCM)

    # Search in registry for robot type defined in config
    robot_class = REGISTRY.get(cfg.robot_type)
    if robot_class is None:
        raise ValueError(f"Unknown robot_type '{cfg.robot_type}'. Known robots: {list(REGISTRY)}")

    # Create robot instance and run it
    robot = robot_class(cfg)
    robot.run()
