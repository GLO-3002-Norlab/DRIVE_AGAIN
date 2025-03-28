from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
import logging

import numpy as np

from DRIVE_AGAIN.common import Command
from DRIVE_AGAIN.geofencing import Geofence
from DRIVE_AGAIN.robot import Robot
from DRIVE_AGAIN.sampling import CommandSamplingStrategy


@dataclass
class Step:
    command: Command
    start_timestamp_ns: float


class IllegalStateTransition(Exception):
    def __init__(self, current_state: str, action: str):
        super().__init__(f"Cannot perform {action} from current state ({current_state})")


class DriveState(ABC):
    def __init__(self, drive):
        self.drive = drive

    @abstractmethod
    def run(self, timestamp_ns: float):
        pass


class WaitingState(DriveState):
    def run(self, timestamp_ns: float):
        pass


class GeofenceCreationState(DriveState):
    def __init__(self, drive):
        super().__init__(drive)

        self.geofence_points: list[np.ndarray] = [self.drive.robot.pose[:2]]
        self.distance_thresold_meters = 0.5

    def run(self, timestamp_ns: float):
        last_point = self.geofence_points[-1][:2]
        current_point = self.drive.robot.pose[:2]

        if np.linalg.norm(current_point - last_point) > self.distance_thresold_meters:
            self.geofence_points.append(current_point)


class ReadyState(DriveState):
    def run(self, timestamp_ns: float):
        pass


class RunningState(DriveState):
    def __init__(self, drive, timestamp_ns: float, step: Step):
        super().__init__(drive)

        self.step_duration_s = 6.0

    def run(self, timestamp_ns: float):
        current_step: Step = self.drive.current_step

        # TODO: https://github.com/GLO-3002-Norlab/DRIVE_AGAIN/issues/44
        # If outside geofence, return to center
        # current_point = self.drive.robot.pose[:2]
        # geofence = self.drive.geofence
        # next_command = self.drive.next_command
        # if not geofence.is_point_inside(current_point):
        #     goal_pose = np.array([self.geofence.origin[0], self.geofence.origin[1], 0])
        #     self.robot.send_goal(goal_pose)
        #     self.pause()
        #     return

        if timestamp_ns - current_step.start_timestamp_ns > self.step_duration_s * 1e9:
            self.drive.sample_next_step(timestamp_ns)
            return

        self.drive.robot.send_command(current_step.command)


class PausedState(DriveState):
    def run(self, timestamp_ns: float):
        pass


class StoppedState(DriveState):
    def run(self, timestamp_ns: float):
        pass


class Drive:
    def __init__(self, robot: Robot, command_sampling_strategy: CommandSamplingStrategy):
        self.robot = robot
        self.command_sampling_strategy = command_sampling_strategy
        self.current_state = WaitingState(self)

        self.geofence: None | Geofence = None
        self.current_step: None | Step = None

        self.commands = []

    def run(self, timestamp_ns: float):
        if self.robot.deadman_switch_pressed:
            self.current_state.run(timestamp_ns)
        else:
            logging.info(f"Deadman switch not pressed")

    def sample_next_step(self, timestamp_ns: float):
        command = self.command_sampling_strategy.sample_command()
        self.current_step = Step(command, timestamp_ns)
        logging.info(f"Sampling next command {command} at timestamp {timestamp_ns}")

        self.current_state = RunningState(self, timestamp_ns, self.current_step)

    def get_commands(self) -> np.ndarray:
        if self.current_state.__class__ == RunningState:
            return np.array(self.commands)

        return np.array([])

    def get_geofence_points(self) -> np.ndarray:
        if self.current_state.__class__ == GeofenceCreationState:
            return np.array(self.current_state.geofence_points)  # type: ignore
        elif self.geofence is not None:
            return np.array([np.array(point) for point in self.geofence.polygon.exterior.coords])

        return np.array([])

    # ============================================ State transitions ============================================
    def start_geofence(self, timestamp_ns: float):
        if self.current_state.__class__ == WaitingState:
            logging.info(f"Starting geofence creation at timestamp {timestamp_ns}")
            self.current_state = GeofenceCreationState(self)
            return

        raise IllegalStateTransition(self.current_state.__class__.__name__, "start_geofence")

    def restart_geofence(self, timestamp_ns: float):
        if self.current_state.__class__ == GeofenceCreationState:
            logging.info(f"Restarting geofence creation at timestamp {timestamp_ns}")
            self.current_state = GeofenceCreationState(self)
            return

        raise IllegalStateTransition(self.current_state.__class__.__name__, "restart_geofence")

    def confirm_geofence(self, timestamp_ns: float):
        if self.current_state.__class__ == GeofenceCreationState:
            logging.info(f"Confirmed geofence at timestamp {timestamp_ns}")
            self.geofence = Geofence(self.current_state.geofence_points)  # type: ignore
            self.current_state = ReadyState(self)
            return

        raise IllegalStateTransition(self.current_state.__class__.__name__, "confirm_geofence")

    def start_drive(self, timestamp_ns: float):
        if self.current_state.__class__ == ReadyState and self.current_step is None:
            logging.info(f"Starting drive at timestamp {timestamp_ns}")
            self.sample_next_step(timestamp_ns)
            return

        raise IllegalStateTransition(self.current_state.__class__.__name__, "start_drive")

    def pause_drive(self, timestamp_ns: float):
        if self.current_state.__class__ == RunningState:
            logging.info(f"Pausing drive at timestamp {timestamp_ns}")
            self.current_state = PausedState(self)
            return

        raise IllegalStateTransition(self.current_state.__class__.__name__, "pause_drive")

    def resume_drive(self, timestamp_ns: float):
        if self.current_state.__class__ == PausedState and self.current_step is not None:
            logging.info(f"Resuming drive at timestamp {timestamp_ns}")
            self.current_step.start_timestamp_ns = timestamp_ns
            self.current_state = RunningState(self, timestamp_ns, self.current_step)
            return

        raise IllegalStateTransition(self.current_state.__class__.__name__, "resume_drive")

    def stop_drive(self, timestamp_ns: float):
        if self.current_state.__class__ == RunningState:
            logging.info(f"Stopped at timestamp {timestamp_ns}")
            self.current_state = StoppedState(self)
            return

        raise IllegalStateTransition(self.current_state.__class__.__name__, "stop_drive")
