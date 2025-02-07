import sys
import time
import matplotlib
import matplotlib.animation
from matplotlib.axes import Axes
import matplotlib.patches
import matplotlib.pyplot as plt
import numpy as np
from DRIVE_AGAIN.keyboard_teleop import KeyboardTeleop
from DRIVE_AGAIN.drive import Drive
from DRIVE_AGAIN.robot import Robot
from DRIVE_AGAIN.common import Command, Pose
from DRIVE_AGAIN.geofencing import Geofence, GeofencingController
from DRIVE_AGAIN.sampling import RandomSampling


WHEEL_BASE = 0.5


def draw_robot(ax: Axes, pose: Pose, geofence: Geofence) -> None:
    x, y, yaw = pose

    robot_color = "green" if geofence.is_point_inside((x, y)) else "red"
    circle = matplotlib.patches.Circle((x, y), WHEEL_BASE / 2, color=robot_color)

    wheel_width = 0.1
    wheel_height = WHEEL_BASE * 0.8

    left_wheel_x = x - WHEEL_BASE / 2 - wheel_width
    left_wheel_y = y - wheel_height / 2

    right_wheel_x = x + WHEEL_BASE / 2
    right_wheel_y = y - wheel_height / 2

    wheel_rotation_deg = np.rad2deg(yaw) + 90
    left_wheel = matplotlib.patches.Rectangle(
        (left_wheel_x, left_wheel_y),
        wheel_width,
        wheel_height,
        rotation_point=(x, y),
        angle=wheel_rotation_deg,
        color="black",
    )
    right_wheel = matplotlib.patches.Rectangle(
        (right_wheel_x, right_wheel_y),
        wheel_width,
        wheel_height,
        rotation_point=(x, y),
        angle=wheel_rotation_deg,
        color="black",
    )

    ax.add_patch(circle)
    ax.add_patch(left_wheel)
    ax.add_patch(right_wheel)


def draw_geofence(ax: Axes, geofence: Geofence) -> None:
    geofence_coords = np.array(geofence.polygon.exterior.coords)
    polygon = matplotlib.patches.Polygon(
        geofence_coords, closed=True, edgecolor="blue", facecolor="none", lw=1, linestyle=":"
    )

    ax.add_patch(polygon)


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

    geofence_coords = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]
    geofence = Geofence(geofence_coords)
    geofencing_controller = GeofencingController(geofence)

    keyboard_teleop = KeyboardTeleop()

    def update(frame):
        global pose

        timestamp = time.time_ns()

        geofencing_controller.update(pose)

        if is_teleop:
            command = keyboard_teleop.get_command()
            robot.send_command(command)
        elif geofencing_controller.override:
            command = geofencing_controller.get_command(pose)
            robot.send_command(command)
        else:
            drive.run(timestamp)

        # Simulating localization noise
        noisy_pose = pose + np.random.normal(0, 0.1, 3)
        robot.pose_callback(noisy_pose)

        ax.clear()
        ax.set_xlim(-5, 5)
        ax.set_ylim(-5, 5)
        draw_geofence(ax, geofence)
        draw_robot(ax, pose, geofence)

    frequency = 20  # Hz
    interval_ms = 1000 / frequency
    ani = matplotlib.animation.FuncAnimation(fig, update, frames=30, interval=interval_ms)  # type: ignore
    plt.show()
