import sys
import io
import base64
import time
from time import sleep
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
from DRIVE_AGAIN.geofencing import Geofence
from DRIVE_AGAIN.sampling import RandomSampling
from DRIVE_AGAIN.server import Server
import threading

WHEEL_BASE = 0.5


class Sim:
    def __init__(self, server: Server, is_teleop: bool):
        plt.switch_backend("Agg")

        self.is_teleop = is_teleop
        self.server: Server = server

        self.pose = np.array([1, 1, np.pi / 2])
        self.robot = Robot(self.pose, self.apply_command)

        command_sampling_strategy = RandomSampling()
        self.drive = Drive(self.robot, command_sampling_strategy, step_duration_s=3.0)

        geofence_coords = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]
        self.geofence = Geofence(geofence_coords)

        self.keyboard_teleop = KeyboardTeleop()
        frequency = 20  # Hz
        self.sim_update_interval = 1 / frequency

    def draw_robot_visualization_figure(self, fig, ax):
        ax.clear()
        ax.set_xlim(-5, 5)
        ax.set_ylim(-5, 5)
        self.draw_geofence(ax, self.geofence)
        self.draw_robot(ax, self.pose, self.geofence)

        return fig

    def encode_fig_to_b64(self, fig):
        buffer = io.BytesIO()
        fig.savefig(buffer, format="png")
        return base64.b64encode(buffer.getvalue()).decode("utf-8")

    def update(self, fig_viz, ax_viz):
        timestamp = time.time_ns()

        command = None
        if self.is_teleop:
            command = self.keyboard_teleop.get_command(timestamp)
            self.robot.send_command(command)
        else:
            self.drive.run(timestamp)

        fig = self.draw_robot_visualization_figure(fig_viz, ax_viz)
        img_base64 = self.encode_fig_to_b64(fig)

        self.server.update_input_space(img_base64)
        self.server.update_robot_viz(img_base64)

    def run(self):
        fig_viz, ax_viz = plt.subplots()
        while True:
            self.update(fig_viz, ax_viz)
            time.sleep(self.sim_update_interval)

    def draw_robot(self, ax: Axes, pose: Pose, geofence: Geofence) -> None:
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

    def draw_geofence(self, ax: Axes, geofence: Geofence) -> None:
        geofence_coords = np.array(geofence.polygon.exterior.coords)
        polygon = matplotlib.patches.Polygon(
            geofence_coords, closed=True, edgecolor="blue", facecolor="none", lw=1, linestyle=":"
        )

        ax.add_patch(polygon)

    def apply_command(self, u: Command) -> None:
        x, y, yaw = self.pose
        v_x, omega_z = u

        x += v_x * np.cos(yaw)
        y += v_x * np.sin(yaw)
        yaw += omega_z

        self.pose = np.array([x, y, yaw])
