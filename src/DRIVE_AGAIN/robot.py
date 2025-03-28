from typing import Callable
from DRIVE_AGAIN.common import Pose, Command


class Robot:
    def __init__(
        self, initial_pose: Pose, send_command_fn: Callable[[Command], None], send_goal_fn: Callable[[Pose], bool]
    ) -> None:
        self.pose = initial_pose
        self.deadman_switch_pressed = False
        self.send_command_fn = send_command_fn
        self.send_goal_fn = send_goal_fn
        self.goal_reached = False

    def pose_callback(self, pose: Pose) -> None:
        self.pose = pose

    def deadman_switch_callback(self, pressed: bool) -> None:
        self.deadman_switch_pressed = pressed

    def goal_reached_callback(self) -> None:
        self.goal_reached = True

    def send_command(self, command: Command) -> None:
        self.send_command_fn(command)

    def send_goal(self, goal_pose: Pose) -> None:
        self.goal_reached = self.send_goal_fn(goal_pose)
