from dataclasses import dataclass


@dataclass
class Saveable:
    @classmethod
    def headers(cls) -> list[str]:
        return list(cls.__annotations__.keys())


@dataclass
class Position6DOF(Saveable):
    timestamp: int  # ns
    step_id: int
    x: float  # m
    y: float  # m
    z: float  # m
    roll: float  # rad
    pitch: float  # rad
    yaw: float  # rad


@dataclass
class Speed6DOF(Saveable):
    timestamp: int  # ns
    step_id: int
    speed_x: float  # m/s
    speed_y: float  # m/s
    speed_z: float  # m/s
    speed_roll: float  # rad/s
    speed_pitch: float  # rad/s
    speed_yaw: float  # rad/s


@dataclass
class Acceleration6DOF(Saveable):
    timestamp: int  # ns
    step_id: int
    a_x: float  # m/(s**2)
    a_y: float  # m/(s**2)
    a_z: float  # m/(s**2)
    a_roll: float  # rad/(s**2)
    a_pitch: float  # rad/(s**2)
    a_yaw: float  # rad/(s**2)


@dataclass
class DriveStep(Saveable):
    step_start_timestamp: int  # ns
    step_id: int
    commanded_linear_velocity: float  # m/s
    commanded_angular_velocity: float  # rad/s
