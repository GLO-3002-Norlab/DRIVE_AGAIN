from typing import Callable
from DRIVE_AGAIN.common import Pose, Command


class Robot:
    def __init__(
        self, initial_pose: Pose, send_command_fn: Callable[[Command], None], send_goal_fn: Callable[[Pose], None]
    ) -> None:
        self.pose = initial_pose
        self.send_command_fn = send_command_fn
        self.send_goal_fn = send_goal_fn
        self.goal_reached = False

    def pose_callback(self, pose: Pose) -> None:
        self.pose = pose

    def send_command(self, command: Command) -> None:
        self.send_command_fn(command)

    def send_goal(self, goal_pose: Pose) -> None:
        self.goal_reached = False
        self.send_goal_fn(goal_pose)

    def goal_reached_callback(self):
        self.goal_reached = True
