import numpy as np
import rclpy
from DRIVE_AGAIN.drive import Drive
from DRIVE_AGAIN.robot import Robot
from DRIVE_AGAIN.sampling import RandomSampling
from geometry_msgs.msg import Twist
from rclpy.node import Node


class DriveRosBridge(Node):
    def __init__(self):
        super().__init__("drive_ros_bridge", parameter_overrides=[])

        initial_pose = np.array([0.0, 0.0, 0.0])

        self.robot = Robot(initial_pose, self.send_command)
        self.command_sampling_strategy = RandomSampling()
        self.drive = Drive(self.robot, self.command_sampling_strategy, step_duration_s=3.0)

        self.cmd_pub = self.create_publisher(Twist, "cmd_drive", 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Drive ROS bridge started")

    def send_command(self, command):
        msg = Twist()
        msg.linear.x = command[0]
        msg.angular.z = command[1]

        self.cmd_pub.publish(msg)

    def control_loop(self):
        current_time_ns = self.get_clock().now().nanoseconds
        self.drive.run(current_time_ns)

    def loc_callback(self, pose):
        self.robot.pose_callback(pose)


def main(args=None):
    rclpy.init(args=args)

    drive_ros_bridge = DriveRosBridge()

    rclpy.spin(drive_ros_bridge)

    drive_ros_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
