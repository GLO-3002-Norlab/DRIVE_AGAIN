from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Literal
from DRIVE_AGAIN.geofencing import Geofence
from DRIVE_AGAIN.robot import Robot
from DRIVE_AGAIN.sampling import CommandSamplingStrategy
from DRIVE_AGAIN.common import Command
import numpy as np


@dataclass
class Step:
    command: Command
    start_timestamp_ns: float


class DriveState(ABC):
    @abstractmethod
    def start(self, timestamp_ns: float):
        pass

    @abstractmethod
    def pause(self):
        pass

    @abstractmethod
    def run(self, timestamp_ns: float):
        pass


class GeofenceCreationState(DriveState):
    def __init__(self, robot: Robot):
        self.robot = robot
        self.geofence_points: list[np.ndarray] = [self.robot.pose[:2]]
        self.geofence_started = False

    def start(self, timestamp_ns: float):
        self.geofence_started = True

    def pause(self):
        self.geofence_started = False

    def run(self, _: float):
        if not self.geofence_started:
            print("Geofence creation paused")
            return

        last_point = self.geofence_points[-1]

        distance_thresold_meters = 0.5
        if np.linalg.norm(self.robot.pose[:2] - last_point[:2]) > distance_thresold_meters:
            self.geofence_points.append(self.robot.pose[:2])


class CommandSamplingState(DriveState):
    def __init__(
        self,
        robot: Robot,
        command_sampling_strategy: CommandSamplingStrategy,
        step_duration_s: float,
        geofence: Geofence,
    ):
        self.robot = robot
        self.command_sampling_strategy = command_sampling_strategy
        self.step_duration_s = step_duration_s
        self.geofence = geofence

        self.next_command = command_sampling_strategy.sample_command()
        self.current_step: None | Step = None

        self.commands = []

    def start(self, timestamp_ns: float):
        if self.current_step is not None:
            return

        self._new_step(self.next_command, timestamp_ns)

    def pause(self):
        if self.current_step is None:
            return

        self.next_command = self.current_step.command
        self.current_step = None

    def run(self, timestamp_ns: float):
        if self.current_step is None:
            print("Command sampling is paused")
            return

        current_pose = self.robot.pose[:2]
        if not self.geofence.is_point_inside(current_pose):
            goal_pose = np.array([self.geofence.origin[0], self.geofence.origin[1], 0])
            self.robot.send_goal(goal_pose)
            self.pause()
            return

        if timestamp_ns - self.current_step.start_timestamp_ns > self.step_duration_s * 1e9:  # type: ignore
            self.save_step()
            self._new_step(self.next_command, timestamp_ns)

        self.robot.send_command(self.current_step.command)  # type: ignore

    def save_step(self):
        if self.current_step is None:
            print("No step to save!")
            return

        print("Step completed! Saving step data...")
        self.commands.append(self.current_step.command)

    def _new_step(self, command: Command, timestamp_ns: float):
        print(f"Starting step with command: {command}")
        self.current_step = Step(command, timestamp_ns)
        self.next_command = self.command_sampling_strategy.sample_command()


class Drive:
    def __init__(self, robot: Robot, command_sampling: CommandSamplingStrategy, step_duration_s: float):
        self.robot = robot
        self.command_sampling = command_sampling
        self.step_duration_s = step_duration_s
        self.drive_state = GeofenceCreationState(self.robot)
        self.geofence: None | Geofence = None

    def change_state(self, new_state_str: Literal["geofence_creation", "command_sampling"]):
        if new_state_str == "geofence_creation":
            print("Switching to geofence creation state...")
            self.drive_state = GeofenceCreationState(self.robot)
        elif new_state_str == "command_sampling" and isinstance(self.drive_state, GeofenceCreationState):
            print("Switching to command sampling state...")
            self.geofence = Geofence(self.drive_state.geofence_points)  # type: ignore
            self.drive_state = CommandSamplingState(
                self.robot, self.command_sampling, self.step_duration_s, self.geofence
            )

    def start(self, timestamp_ns: float):
        self.drive_state.start(timestamp_ns)

    def pause(self):
        self.drive_state.pause()

    def run(self, timestamp_ns: float):
        self.drive_state.run(timestamp_ns)

    def get_commands(self) -> np.ndarray:
        if isinstance(self.drive_state, CommandSamplingState):
            return np.array(self.drive_state.commands)

        return np.array([])

    def get_geofence_points(self) -> np.ndarray:
        if isinstance(self.drive_state, GeofenceCreationState):
            return np.array(self.drive_state.geofence_points)

        return np.array([])

    def get_geofence(self) -> None | Geofence:
        return self.geofence

    def _new_step(self, command: Command, timestamp_ns: float):
        print(f"Starting step with command: {command}")
        self.current_step = Step(command, timestamp_ns)
        self.next_command = self.command_sampling.sample_command()
