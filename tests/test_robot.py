import numpy as np
import pytest

from DRIVE_AGAIN.robot import *


class Mocker:
    def __init__(self):
        self.args = None

    def __call__(self, *args):
        self.args = args


@pytest.fixture
def pose() -> Pose:
    """Set up a pose for testing"""
    return np.array([1, 2, 3])


@pytest.fixture
def mocker() -> Mocker:
    """Set up a mocker for testing"""
    return Mocker()


@pytest.fixture
def command() -> Command:
    """Set up a pose for testing"""
    return np.array([1, 2])


@pytest.fixture
def robot() -> Robot:
    """Set up a robot for testing"""
    return Robot(np.array([0, 0, 0]), lambda _: None, lambda _: None)


def test_pose_callback_sets_the_pose(robot: Robot):
    new_pose = np.array([1, 1, 1])
    robot.pose_callback(new_pose)

    assert (robot.pose == new_pose).all()


def test_send_command_calls_send_command_fn(robot: Robot, command: Command, mocker: Mocker):
    robot.send_command_fn = mocker

    robot.send_command(command)

    assert mocker.args is not None
    assert (mocker.args[0] == command).all()


def test_send_goal_calls_send_goal_fn(robot: Robot, pose: Pose, mocker: Mocker):
    robot.send_goal_fn = mocker

    robot.send_goal(pose)

    assert not robot.goal_reached
    assert mocker.args is not None
    assert (mocker.args[0] == pose).all()


def test_goal_reached_callback(robot: Robot):
    robot.goal_reached = False
    robot.goal_reached_callback()

    assert robot.goal_reached
