from DRIVE_AGAIN.drive import Drive, DriveStateEnum
from DRIVE_AGAIN.keyboard_teleop import KeyboardTeleop
from DRIVE_AGAIN.robot import Robot
from DRIVE_AGAIN.sampling import RandomSampling
from DRIVE_AGAIN.server import Server
from DRIVE_AGAIN.sim import Sim
import numpy as np
import time
from threading import Thread
import matplotlib.pyplot as plt


WHEEL_BASE = 0.5


class App:
    def __init__(self):
        initial_pose = np.array([1, 1, np.pi / 2])

        self.sim = Sim(initial_pose)
        self.robot = Robot(initial_pose, self.sim.apply_command, self.sim.apply_goal)
        self.sim.set_robot_update_fn(self.robot.pose_callback)

        command_sampling_strategy = RandomSampling()
        self.drive = Drive(self.robot, command_sampling_strategy, step_duration_s=3.0)

        self.server = Server(self.start_drive_cb, self.start_geofence_cb)
        self.keyboard_teleop = KeyboardTeleop()

        frequency = 20  # Hz
        self.update_interval = 1 / frequency

    def run(self):
        while True:
            self.update_loop()

    def update_loop(self):
        timestamp = time.time_ns()
        self.sim.update()

        if self.keyboard_teleop.is_deadman_key_pressed():
            self.drive.start(timestamp)
        else:
            self.drive.pause()

        command = self.keyboard_teleop.get_robot_command(timestamp)
        self.robot.send_command(command)

        self.drive.run(timestamp)

        if self.drive.geofence is None:
            geofence_points = self.drive.get_geofence_points()
        else:
            geofence_points = np.array([np.array(point) for point in self.drive.geofence.polygon.exterior.coords])

        self.server.update_robot_viz(self.robot.pose, geofence_points, WHEEL_BASE)
        self.server.update_input_space(self.drive.get_commands())
        time.sleep(self.update_interval)

    def start_drive_cb(self):
        self.drive.change_state(DriveStateEnum.command_sampling)

    def start_geofence_cb(self):
        self.drive.change_state(DriveStateEnum.geofence_creation)


def main():
    plt.switch_backend("Agg")

    app = App()

    update_loop = Thread(target=app.run)
    update_loop.start()

    app.server.run()
    update_loop.join()


if __name__ == "__main__":
    main()
