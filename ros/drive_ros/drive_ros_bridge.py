import logging
from threading import Thread

import numpy as np
import rclpy
import tf_transformations
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Bool
from rclpy.node import Node

from DRIVE_AGAIN.drive import Drive
from DRIVE_AGAIN.robot import Robot
from DRIVE_AGAIN.sampling import RandomSampling
from DRIVE_AGAIN.server import Server

WHEEL_BASE = 0.5


def redirect_logging_to_ros2():
    ros2_logger = rclpy.logging.get_logger("DRIVE_AGAIN")  # type: ignore

    class ROS2Handler(logging.Handler):
        def emit(self, record):
            log_entry = self.format(record)
            if record.levelno == logging.DEBUG:
                ros2_logger.debug(log_entry)
            elif record.levelno == logging.INFO:
                ros2_logger.info(log_entry)
            elif record.levelno == logging.WARNING:
                ros2_logger.warn(log_entry)
            elif record.levelno == logging.ERROR:
                ros2_logger.error(log_entry)
            elif record.levelno == logging.CRITICAL:
                ros2_logger.fatal(log_entry)

    root_logger = logging.getLogger()
    for handler in root_logger.handlers:
        root_logger.removeHandler(handler)

    ros2_handler = ROS2Handler()
    formatter = logging.Formatter("%(levelname)s: %(message)s")
    ros2_handler.setFormatter(formatter)
    root_logger.addHandler(ros2_handler)
    root_logger.setLevel(logging.INFO)


class DriveRosBridge(Node):
    def __init__(self):
        super().__init__("drive_ros_bridge", parameter_overrides=[])

        redirect_logging_to_ros2()

        initial_pose = np.array([0.0, 0.0, 0.0])

        # Drive core setup
        self.robot = Robot(initial_pose, self.send_command, lambda x: True)
        self.command_sampling_strategy = RandomSampling()
        self.drive = Drive(self.robot, self.command_sampling_strategy)
        self.server = Server(self.start_drive_cb, self.start_geofence_cb)

        # Interface setup
        self.interface_thread = Thread(target=self.server.run)
        self.interface_thread.start()

        # ROS setup
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.loc_sub = self.create_subscription(Pose, "pose", self.loc_callback, 10)
        self.deadman_sub = self.create_subscription(Bool, "deadman", self.deadman_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Drive ROS bridge started")

    def start_drive_cb(self):
        current_time_ns = self.get_clock().now().nanoseconds
        # For now doing both at the same time
        self.drive.confirm_geofence(current_time_ns)
        self.drive.start_drive(current_time_ns + 1)

    def start_geofence_cb(self):
        current_time_ns = self.get_clock().now().nanoseconds
        self.drive.start_geofence(current_time_ns)

    def send_command(self, command):
        msg = Twist()
        msg.linear.x = command[0]
        msg.angular.z = command[1]

        self.cmd_pub.publish(msg)

    def control_loop(self):
        current_time_ns = self.get_clock().now().nanoseconds
        self.drive.run(current_time_ns)

        geofence_points = self.drive.get_geofence_points()

        self.server.update_robot_viz(self.robot.pose, geofence_points, WHEEL_BASE)
        self.server.update_input_space(self.drive.get_commands())

    def loc_callback(self, pose_msg: Pose):
        quaternion = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
        pose = np.array([pose_msg.position.x, pose_msg.position.y, yaw])

        self.robot.pose_callback(pose)

    def deadman_callback(self, msg: Bool):
        self.robot.deadman_switch_callback(msg.data)


def main(args=None):
    rclpy.init(args=args)

    drive_ros_bridge = DriveRosBridge()

    rclpy.spin(drive_ros_bridge)

    drive_ros_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
