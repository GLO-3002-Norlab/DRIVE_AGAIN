from abc import ABC, abstractmethod

import numpy as np

from DRIVE_AGAIN.common import Command


class CommandSamplingStrategy(ABC):
    @abstractmethod
    def sample_command(self) -> Command:
        pass


class RandomSampling(CommandSamplingStrategy):
    def sample_command(self) -> Command:
        v_x = np.random.uniform(0, 0.4)
        omega_z = np.random.uniform(-1, 1)

        return np.array([v_x, omega_z])
