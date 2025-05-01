import logging
from threading import Thread

import numpy as np
import rclpy
import tf_transformations
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool

from DRIVE_AGAIN.common import Pose
from DRIVE_AGAIN.drive import Drive
from DRIVE_AGAIN.robot import Robot
from DRIVE_AGAIN.sampling import CommandSamplingFactory
from DRIVE_AGAIN.server import Server


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
    server: Server

    def __init__(self):
        super().__init__("drive_ros_bridge", parameter_overrides=[])

        redirect_logging_to_ros2()

        initial_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Get ROS parameters
        self.declare_parameter("nb_steps", 100)
        self.declare_parameter("step_duration_s", 6.0)
        self.declare_parameter("command_sampling_strategy", "random")
        self.declare_parameter("min_linear_speed", 0.0)
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", -1.0)
        self.declare_parameter("min_angular_speed", 1.0)

        self.nb_steps: int = self.get_parameter("nb_steps").get_parameter_value().integer_value
        self.step_duration_s: float = self.get_parameter("step_duration_s").get_parameter_value().double_value
        self.command_sampling_strategy_str = self.get_parameter("command_sampling_strategy").value
        self.min_linear_speed: float = self.get_parameter("min_linear_speed").get_parameter_value().double_value
        self.max_linear_speed: float = self.get_parameter("max_linear_speed").get_parameter_value().double_value
        self.min_angular_speed: float = self.get_parameter("min_angular_speed").get_parameter_value().double_value
        self.max_angular_speed: float = self.get_parameter("max_angular_speed").get_parameter_value().double_value

        # Drive core setup
        self.robot = Robot(initial_pose, self.send_command, self.send_goal)
        self.command_sampling_strategy = CommandSamplingFactory.create_sampling_strategy(
            self.command_sampling_strategy_str,  # type: ignore
            self.min_linear_speed,
            self.max_linear_speed,
            self.min_angular_speed,
            self.max_angular_speed,
        )
        self.drive = Drive(
            self.robot,
            self.command_sampling_strategy,
            self.nb_steps,
            self.step_duration_s,
            self.state_transition_cb,
            self.sample_next_step_cb,
        )

        # Interface setup
        self.interface_server = Server(self.drive, self.get_timestamp_ns)
        self.interface_thread = Thread(target=self.interface_server.run)
        self.interface_thread.start()

        # ROS setup
        self.timer = self.create_timer(0.1, self.control_loop)

        # Pubs
        self.loc_sub = self.create_subscription(PoseStamped, "pose", self.loc_callback, 10)
        self.deadman_sub = self.create_subscription(Bool, "deadman", self.deadman_callback, 10)

        # Subs
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "goal", 10)
        self.goal_reached_sub = self.create_subscription(PoseStamped, "goal_reached", self.goal_reached_callback, 10)

        self.get_logger().info("Drive ROS bridge started")

    def control_loop(self):
        current_time_ns = self.get_timestamp_ns()

        # Drive core loop
        self.drive.run(current_time_ns)

        # Interface visualization
        self.interface_server.update_visualization()

    def load_geofence_cb(self, dataset_name: str):
        current_time_ns = self.get_clock().now().nanoseconds
        self.drive.load_geofence(dataset_name, current_time_ns)

    def state_transition_cb(self, state: str):
        self.interface_server.state_transition(state)

    def sample_next_step_cb(self, command, step_count):
        self.interface_server.sample_next_step(command, step_count)

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

    def get_timestamp_ns(self) -> float:
        return self.get_clock().now().nanoseconds


def main(args=None):
    rclpy.init(args=args)

    drive_ros_bridge = DriveRosBridge()

    rclpy.spin(drive_ros_bridge)

    drive_ros_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
