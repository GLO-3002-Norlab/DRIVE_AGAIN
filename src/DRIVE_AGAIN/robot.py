from typing import Callable

from common import Command, Pose


class Robot:
    def __init__(self, initial_pose: Pose, send_command_fn: Callable[[Command], None]) -> None:
        self.pose = initial_pose
        self.send_command_fn = send_command_fn

    def pose_callback(self, pose: Pose) -> None:
        self.pose = pose

    def send_command(self, command: Command) -> None:
        self.send_command_fn(command)
