import sys
import time
from typing import Literal
import matplotlib
import matplotlib.animation
from matplotlib.axes import Axes
import matplotlib.pyplot as plt
import numpy as np
from DRIVE_AGAIN.keyboard_teleop import KeyboardTeleop
from DRIVE_AGAIN.drive import Drive
from DRIVE_AGAIN.robot import Robot
from DRIVE_AGAIN.common import Command, Pose
from DRIVE_AGAIN.sampling import RandomSampling

WHEEL_BASE = 0.5


def draw_robot(ax: Axes, pose: Pose) -> None:
    x, y, yaw = pose

    ax.clear()
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)

    # Draw circle for robot
    circle = plt.Circle((x, y), WHEEL_BASE / 2, color="r")  # type: ignore

    # Draw wheels
    wheel_width = 0.1
    wheel_height = WHEEL_BASE * 0.8

    left_wheel_x = x - WHEEL_BASE / 2 - wheel_width
    left_wheel_y = y - wheel_height / 2

    right_wheel_x = x + WHEEL_BASE / 2
    right_wheel_y = y - wheel_height / 2

    wheel_rotation_deg = np.rad2deg(yaw) + 90
    left_wheel = plt.Rectangle((left_wheel_x, left_wheel_y), wheel_width, wheel_height, rotation_point=(x, y), angle=wheel_rotation_deg, color="black")  # type: ignore
    right_wheel = plt.Rectangle((right_wheel_x, right_wheel_y), wheel_width, wheel_height, rotation_point=(x, y), angle=wheel_rotation_deg, color="black")  # type: ignore

    ax.add_artist(circle)
    ax.add_artist(left_wheel)
    ax.add_artist(right_wheel)


def apply_command(u: Command) -> None:
    global pose
    x, y, yaw = pose
    v_x, omega_z = u

    x += v_x * np.cos(yaw)
    y += v_x * np.sin(yaw)
    yaw += omega_z

    pose = np.array([x, y, yaw])


if __name__ == "__main__":
    is_teleop = True
    if len(sys.argv) > 1:
        if sys.argv[1] == "drive":
            is_teleop = False

    pose = np.array([1, 1, np.pi / 2])

    fig, ax = plt.subplots()

    robot = Robot(pose, apply_command)
    command_sampling_strategy = RandomSampling()
    drive = Drive(robot, command_sampling_strategy, step_duration_s=3.0)

    keyboard_teleop = KeyboardTeleop()

    def update(frame):
        global pose

        timestamp = time.time_ns()

        command = None
        if is_teleop:
            command = keyboard_teleop.get_command()
            robot.send_command(command)
        else:
            drive.run(timestamp)

        # Simulating localization noise
        noisy_pose = pose + np.random.normal(0, 0.1, 3)
        robot.pose_callback(noisy_pose)

        draw_robot(ax, pose)

    frequency = 20  # Hz
    interval_ms = 1000 / frequency
    ani = matplotlib.animation.FuncAnimation(fig, update, frames=60, interval=interval_ms)  # type: ignore
    plt.show()
