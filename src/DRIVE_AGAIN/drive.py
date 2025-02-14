from dataclasses import dataclass
from DRIVE_AGAIN.robot import Robot
from DRIVE_AGAIN.sampling import CommandSamplingStrategy
from DRIVE_AGAIN.common import Command
import numpy as np


@dataclass
class Step:
    command: Command
    start_timestamp_ns: float


class Drive:
    def __init__(self, robot: Robot, command_sampling: CommandSamplingStrategy, step_duration_s: float):
        self.robot = robot
        self.command_sampling = command_sampling
        self.step_duration_s = step_duration_s

        self.current_step: Step | None = None

        self.commands = []

    def get_commands(self) -> np.ndarray:
        return np.array(self.commands)

    def run(self, timestamp_ns: float):
        if self.current_step is None:
            self.start_new_step(timestamp_ns)

        if timestamp_ns - self.current_step.start_timestamp_ns > self.step_duration_s * 1e9:  # type: ignore
            self.start_new_step(timestamp_ns)

        self.robot.send_command(self.current_step.command)  # type: ignore

    def start_new_step(self, timestamp_ns: float):
        command = self.command_sampling.sample_command()
        self.commands.append(command)
        print(f"New command: {command}")

        self.current_step = Step(command, timestamp_ns)
