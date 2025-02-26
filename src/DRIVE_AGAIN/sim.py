from DRIVE_AGAIN.common import Command, Pose
import time
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
        self.robot = Robot(self.pose, self.apply_command, self.apply_goal)

        command_sampling_strategy = RandomSampling()
        self.drive = Drive(self.robot, command_sampling_strategy, step_duration_s=3.0)

        geofence_coords = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]
        self.geofence = Geofence(geofence_coords)
        self.geofencing_controller = GeofencingController(self.geofence)

        self.keyboard_teleop = KeyboardTeleop()
        frequency = 20  # Hz
        self.sim_update_interval = 1 / frequency

    def update(self):
        timestamp = time.time_ns()

        # TODO: Think of a better way to handle input to make sure to not switch multiple times
        if self.keyboard_teleop.is_geofence_key_pressed():
            self.drive.change_state("command_sampling")

        if self.keyboard_teleop.is_deadman_key_pressed():
            self.drive.start(timestamp)
        else:
            self.drive.pause()

        command = self.keyboard_teleop.get_robot_command(timestamp)
        self.robot.send_command(command)

        self.drive.run(timestamp)

        # Simulating localization noise
        noisy_pose = self.pose + np.random.normal(0, 0.1, 3)
        self.robot.pose_callback(noisy_pose)

        # TODO: Eventually, drawing stuff should not be in the update loop
        if self.drive.geofence is None:
            geofence_points = self.drive.get_geofence_points()
        else:
            geofence_points = np.array([np.array(point) for point in self.drive.geofence.polygon.exterior.coords])

        self.server.update_robot_viz(self.robot.pose, geofence_points, self.WHEEL_BASE)
        self.server.update_input_space(self.drive.get_commands())

    def run(self):
        while True:
            self.update()
            time.sleep(self.sim_update_interval)

    def apply_goal(self, goal: Pose) -> None:
        # TODO: Implement something for the robot to go to the goal like in the Geofencing controller
        self.pose = goal
        self.robot.goal_reached_callback()

    def apply_command(self, u: Command) -> None:
        x, y, yaw = self.pose
        v_x, omega_z = u

        x += v_x * np.cos(yaw)
        y += v_x * np.sin(yaw)
        yaw += omega_z

        self.pose = np.array([x, y, yaw])
