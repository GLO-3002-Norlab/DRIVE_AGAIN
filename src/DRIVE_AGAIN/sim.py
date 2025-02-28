from collections.abc import Callable
from DRIVE_AGAIN.common import Command, Pose
import numpy as np


class Sim:
    def __init__(self, initial_pose: Pose):
        self.pose = initial_pose
        self.robot_pose_update_fn = lambda _: None

    def set_robot_update_fn(self, robot_pose_update_fn: Callable[[Pose], None]):
        self.robot_pose_update_fn = robot_pose_update_fn

    def update(self):
        # Simulating localization noise
        noisy_pose = self.pose + np.random.normal(0, 0.05, 3)
        self.robot_pose_update_fn(noisy_pose)

    def apply_goal(self, goal: Pose) -> bool:
        self.pose = goal
        return True

    def apply_command(self, u: Command) -> None:
        x, y, yaw = self.pose
        v_x, omega_z = u

        x += v_x * np.cos(yaw)
        y += v_x * np.sin(yaw)
        yaw += omega_z

        self.pose = np.array([x, y, yaw])
