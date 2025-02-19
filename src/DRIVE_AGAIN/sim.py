import base64
import io
import time

import matplotlib.pyplot as plt
import numpy as np
from common import Command, Pose
from drive import Drive
from geofencing import Geofence, GeofencingController
from keyboard_teleop import KeyboardTeleop
from plot import draw_input_space, draw_robot_visualization_figure
from robot import Robot
from sampling import RandomSampling
from server import Server


class Sim:
    WHEEL_BASE = 0.5

    def __init__(self, server: Server):
        self.server: Server = server

        self.pose = np.array([1, 1, np.pi / 2])
        self.robot = Robot(self.pose, self.apply_command)

        command_sampling_strategy = RandomSampling()
        self.drive = Drive(self.robot, command_sampling_strategy, step_duration_s=3.0)

        geofence_coords = [(-1.0, -1.0), (4.0, -1.0), (4.0, 4.0), (-1.0, 4.0)]
        self.geofence = Geofence(geofence_coords)
        self.geofencing_controller = GeofencingController(self.geofence)

        self.keyboard_teleop = KeyboardTeleop()
        frequency = 20  # Hz
        self.sim_update_interval = 1 / frequency

        server.connect_cb = self.connect_cb

    def update(self, fig_viz, ax_viz, fig_input_space, ax_input_space):
        timestamp = time.time_ns()

        self.geofencing_controller.update(self.pose)

        # if self.keyboard_teleop.is_deadman_switch_pressed():
        self.drive.start(timestamp)
        # else:
        #     self.drive.pause()

        if self.keyboard_teleop.is_teleop_active():
            command = self.keyboard_teleop.get_command(timestamp)
            self.robot.send_command(command)
        elif self.geofencing_controller.override:
            command = self.geofencing_controller.get_command(self.pose)
            self.drive.pause()
            self.robot.send_command(command)
        elif self.drive.run(timestamp):
            self.server.send_data_point(self.drive.next_command)

        self.server.update_robot_position(self.pose)

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

    def connect_cb(self):
        self.server.send_geofence(self.geofence)
        self.server.send_map_bounds(-8, -8, 8, 382)
        # TODO: Send the real data bounds
        self.server.send_data_bounds(
            [
                np.array([-1.0, -1.0]),
                np.array([1.0, -1.0]),
                np.array([1.0, 1.0]),
                np.array([-1.0, 1.0]),
                np.array([-1.0, -1.0]),
            ]
        )
