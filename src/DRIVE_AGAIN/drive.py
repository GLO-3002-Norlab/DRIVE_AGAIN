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

        self.next_command = command_sampling.sample_command()
        self.current_step: Step | None = None

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
            return

        if timestamp_ns - self.current_step.start_timestamp_ns > self.step_duration_s * 1e9:  # type: ignore
            self.save_step()
            self._new_step(self.next_command, timestamp_ns)

        self.robot.send_command(self.current_step.command)  # type: ignore

    def get_commands(self) -> np.ndarray:
        return np.array(self.commands)

    def save_step(self):
        if self.current_step is None:
            print("No step to save!")
            return
        
        print("Step completed! Saving step data...")
        self.commands.append(self.current_step.command)

    def _new_step(self, command: Command, timestamp_ns: float):
        print(f"Starting step with command: {command}")
        self.current_step = Step(command, timestamp_ns)
        self.next_command = self.command_sampling.sample_command()
