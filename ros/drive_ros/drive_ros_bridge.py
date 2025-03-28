from threading import Thread

import numpy as np
import rclpy
import tf_transformations
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node

from DRIVE_AGAIN.drive import Drive, DriveStateEnum
from DRIVE_AGAIN.robot import Robot
from DRIVE_AGAIN.sampling import RandomSampling
from DRIVE_AGAIN.server import Server

WHEEL_BASE = 0.5


class DriveRosBridge(Node):
    def __init__(self):
        super().__init__("drive_ros_bridge", parameter_overrides=[])

        initial_pose = np.array([0.0, 0.0, 0.0])

        # Drive core setup
        self.robot = Robot(initial_pose, self.send_command, lambda x: True)
        self.command_sampling_strategy = RandomSampling()
        self.drive = Drive(self.robot, self.command_sampling_strategy, step_duration_s=3.0)
        self.server = Server(self.start_drive_cb, self.start_geofence_cb, self.skip_command_cb)

        # Interface setup
        self.interface_thread = Thread(target=self.server.run)
        self.interface_thread.start()

        # ROS setup
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.loc_sub = self.create_subscription(Pose, "pose", self.loc_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Drive ROS bridge started")

    def start_drive_cb(self):
        self.drive.change_state(DriveStateEnum.command_sampling)

    def start_geofence_cb(self):
        self.drive.change_state(DriveStateEnum.geofence_creation)

    # TODO
    def skip_command_cb(self):
        self.get_logger().info("SKIPPING COMMAND YEY")
        pass

    def send_command(self, command):
        msg = Twist()
        msg.linear.x = command[0]
        msg.angular.z = command[1]

        self.cmd_pub.publish(msg)

    def control_loop(self):
        current_time_ns = self.get_clock().now().nanoseconds
        self.drive.start(current_time_ns)
        self.drive.run(current_time_ns)

        # TODO: Do something cleaner with this geofence_points hack
        if self.drive.geofence is None:
            geofence_points = self.drive.get_geofence_points()
        else:
            geofence_points = np.array([np.array(point) for point in self.drive.geofence.polygon.exterior.coords])

        self.server.update_robot_viz(self.robot.pose, geofence_points, WHEEL_BASE)
        self.server.update_input_space(self.drive.get_commands())

    def loc_callback(self, pose_msg: Pose):
        quaternion = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
        pose = np.array([pose_msg.position.x, pose_msg.position.y, yaw])

        self.robot.pose_callback(pose)


def main(args=None):
    rclpy.init(args=args)

    drive_ros_bridge = DriveRosBridge()

    rclpy.spin(drive_ros_bridge)

    drive_ros_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
