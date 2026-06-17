"""
Registry containing all available robots.
Maps robot_type as stated in .yaml config to the corresponding class.
To add a robot, import the class and add a corresponding line to the REGISTRY dictionary.
"""

from robot.robots.basic_car_us_lt import BasicCarUSLT

REGISTRY: dict[str, type] = {
    "basic_car_us_lt": BasicCarUSLT,
}