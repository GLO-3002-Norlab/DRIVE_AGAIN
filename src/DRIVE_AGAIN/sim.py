import base64
import matplotlib.pyplot as plt
from DRIVE_AGAIN.common import Command
import io
import time
from DRIVE_AGAIN.plot import draw_input_space, draw_robot_visualization_figure
import numpy as np
from DRIVE_AGAIN.drive import Drive
from DRIVE_AGAIN.geofencing import Geofence, GeofencingController
from DRIVE_AGAIN.keyboard_teleop import KeyboardTeleop
from DRIVE_AGAIN.robot import Robot
from DRIVE_AGAIN.sampling import RandomSampling
from DRIVE_AGAIN.server import Server


class Sim:
    WHEEL_BASE = 0.5

    def __init__(self, server: Server):
        self.server: Server = server

        self.pose = np.array([1, 1, np.pi / 2])
        self.robot = Robot(self.pose, self.apply_command)

        command_sampling_strategy = RandomSampling()
        self.drive = Drive(self.robot, command_sampling_strategy, step_duration_s=3.0)

        geofence_coords = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]
        self.geofence = Geofence(geofence_coords)
        self.geofencing_controller = GeofencingController(self.geofence)

        self.keyboard_teleop = KeyboardTeleop()
        frequency = 20  # Hz
        self.sim_update_interval = 1 / frequency

    def encode_fig_to_b64(self, fig):
        buffer = io.BytesIO()
        fig.savefig(buffer, format="png")
        return base64.b64encode(buffer.getvalue()).decode("utf-8")

    def update(self, fig_viz, ax_viz, fig_input_space, ax_input_space):
        timestamp = time.time_ns()

        self.geofencing_controller.update(self.pose)

        command = None
        if self.keyboard_teleop.is_active():
            command = self.keyboard_teleop.get_command(timestamp)
            self.robot.send_command(command)
        elif self.geofencing_controller.override:
            command = self.geofencing_controller.get_command(self.pose)
            self.robot.send_command(command)
        else:
            self.drive.run(timestamp)

        draw_robot_visualization_figure(ax_viz, self.pose, self.geofence, self.WHEEL_BASE)
        viz_b64 = self.encode_fig_to_b64(fig_viz)

        draw_input_space(ax_input_space, self.drive.get_commands())
        input_space_b64 = self.encode_fig_to_b64(fig_input_space)

        self.server.update_robot_viz(viz_b64)
        self.server.update_input_space(input_space_b64)

    def run(self):
        fig_viz, ax_viz = plt.subplots()
        fig_input_space, ax_input_space = plt.subplots()
        while True:
            self.update(fig_viz, ax_viz, fig_input_space, ax_input_space)
            time.sleep(self.sim_update_interval)

    def apply_command(self, u: Command) -> None:
        x, y, yaw = self.pose
        v_x, omega_z = u

        x += v_x * np.cos(yaw)
        y += v_x * np.sin(yaw)
        yaw += omega_z

        self.pose = np.array([x, y, yaw])
