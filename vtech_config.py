"""
Configuration constants for the vtech pressure control system
"""

from dataclasses import dataclass
from typing import List


@dataclass
class NetworkConfig:
    """Network configuration constants"""

    PC_ADDRESS = "10.203.49.197"
    ARDUINO_PORTS = [10001, 10002, 10003, 10004, 10005, 10006, 10007, 10008]
    ZMQ_PUB_PORT = "tcp://127.0.0.1:5555"
    ZMQ_SUB_PORT = "tcp://127.0.0.1:3885"


@dataclass
class ControlConfig:
    """Control system configuration constants"""

    DEFAULT_ARDUINO_IDS = [8]
    DEFAULT_POSITION_PROFILE = 2  # 0: sum of sine waves, 1: single sine wave, 2: step
    DEFAULT_TRIAL_DURATION = 120.0  # seconds

    # Control modes
    CONTROL_MODE_BASELINE_SMC = 0
    CONTROL_MODE_SMC_ILC = 1
    CONTROL_MODE_SMC_SPO = 2


@dataclass
class DataConfig:
    """Data recording configuration constants"""

    MOCAP_DATA_SIZE = 21  # base(x y z qw qx qy qz) + top(x1 y1 z1 qw1 qx1 qy1 qz1)
    PRESSURE_SENSORS_PER_ARDUINO = 4


@dataclass
class ThreadConfig:
    """Threading configuration constants"""

    THREAD_SLEEP_INTERVAL = 0.01  # seconds
    COMMUNICATION_TIMEOUT = 1.0  # seconds
