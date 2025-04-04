from abc import ABC, abstractmethod
from typing import Literal

import numpy as np

from DRIVE_AGAIN.common import Command


class CommandSamplingStrategy(ABC):
    @abstractmethod
    def sample_command(self) -> Command:
        pass


class RandomSampling(CommandSamplingStrategy):
    def __init__(self, min_linear_speed, max_linear_speed, min_angular_speed, max_angular_speed):
        self.min_linear_speed = min_linear_speed
        self.max_linear_speed = max_linear_speed
        self.min_angular_speed = min_angular_speed
        self.max_angular_speed = max_angular_speed

    def sample_command(self) -> Command:
        v_x = np.random.uniform(self.min_linear_speed, self.max_linear_speed)
        omega_z = np.random.uniform(self.min_angular_speed, self.max_angular_speed)

        return np.array([v_x, omega_z])


CommandSamplingStrategyType = Literal["random"]


class CommandSamplingFactory:
    @staticmethod
    def create_sampling_strategy(
        command_sampling_strategy_str: CommandSamplingStrategyType,
        min_linear_speed: float,
        max_linear_speed: float,
        min_angular_speed: float,
        max_angular_speed: float,
    ) -> CommandSamplingStrategy:
        if command_sampling_strategy_str == "random":
            return RandomSampling(min_linear_speed, max_linear_speed, min_angular_speed, max_angular_speed)
        else:
            raise ValueError(f"Unknown command sampling strategy: {command_sampling_strategy_str}")
