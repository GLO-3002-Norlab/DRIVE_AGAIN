import logging
import os
from threading import Thread

import numpy as np
import rclpy
import tf_transformations
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool

from DRIVE_AGAIN.common import Pose
from DRIVE_AGAIN.data.dataset_recorder import DatasetRecorder
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

        initial_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Drive core setup
        self.robot = Robot(initial_pose, self.send_command, self.send_goal)
        self.command_sampling_strategy = RandomSampling()
        self.drive = Drive(self.robot, self.command_sampling_strategy)
        self.server = Server(self.start_drive_cb, self.start_geofence_cb, self.drive.save_dataset, self.skip_command_cb)

        # Interface setup
        self.interface_thread = Thread(target=self.server.run)
        self.interface_thread.start()

        # ROS setup
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "goal", 10)
        self.loc_sub = self.create_subscription(PoseStamped, "pose", self.loc_callback, 10)
        self.goal_reached_sub = self.create_subscription(PoseStamped, "goal_reached", self.goal_reached_callback, 10)
        self.deadman_sub = self.create_subscription(Bool, "deadman", self.deadman_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Drive ROS bridge started")

    def start_drive_cb(self):
        current_time_ns = self.get_clock().now().nanoseconds
        # For now doing both at the same time
        self.drive.confirm_geofence(current_time_ns)
        self.drive.start_drive(current_time_ns + 1)
        self.command_sampling_started = True

    def start_geofence_cb(self):
        current_time_ns = self.get_clock().now().nanoseconds
        self.drive.start_geofence(current_time_ns)

    def skip_command_cb(self):
        current_time_ns = self.get_clock().now().nanoseconds
        self.drive.skip_current_step(current_time_ns)

    def send_command(self, command):
        msg = Twist()
        msg.linear.x = command[0]
        msg.angular.z = command[1]

        self.cmd_pub.publish(msg)

    def send_goal(self, goal_pose: Pose):
        quat = tf_transformations.quaternion_from_euler(goal_pose[3], goal_pose[4], goal_pose[5])

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = goal_pose[0]
        pose_msg.pose.position.y = goal_pose[1]
        pose_msg.pose.position.z = goal_pose[2]
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        self.goal_pub.publish(pose_msg)

    def goal_reached_callback(self, pose_msg: PoseStamped):
        self.robot.goal_reached_callback()

    def control_loop(self):
        current_time_ns = self.get_clock().now().nanoseconds
        self.drive.run(current_time_ns)

        geofence_points = self.drive.get_geofence_points()

        self.server.update_robot_viz(self.robot.pose, geofence_points, WHEEL_BASE)
        self.server.update_input_space(self.drive.get_commands())

        if self.drive.can_skip_command():
            self.server.skippable_state_start()
        else:
            self.server.skippable_state_end()

    def loc_callback(self, pose_msg: PoseStamped):
        quaternion = [
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w,
        ]
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        pose = np.array(
            [pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z, roll, pitch, yaw]
        )

        current_time_ns = self.get_clock().now().nanoseconds

        self.robot.pose_callback(pose, current_time_ns)

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
