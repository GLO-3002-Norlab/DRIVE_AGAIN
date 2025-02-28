import numpy as np
import pytest

from DRIVE_AGAIN.common import Command, Pose
from DRIVE_AGAIN.drive import *
from DRIVE_AGAIN.geofencing import *
from DRIVE_AGAIN.robot import Robot
from DRIVE_AGAIN.sampling import CommandSamplingStrategy, RandomSampling


@pytest.fixture
def robot() -> Robot:
    """Set up a robot for testing"""
    return Robot(np.array([0, 0, 0]), lambda _: None, lambda _: None)


@pytest.fixture
def geofence() -> Geofence:
    """Set up a geofence for testing"""
    polygon_coords = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]  # A square
    return Geofence(polygon_coords)


@pytest.fixture
def geofence_creation_state(robot: Robot) -> GeofenceCreationState:
    """Set up a geofence creation state for testing"""
    return GeofenceCreationState(robot)


@pytest.fixture
def command_sampling_strategy() -> CommandSamplingStrategy:
    """Set up a command sampling strategy for testing"""
    return RandomSampling()


@pytest.fixture
def command_sampling_state(
    robot: Robot, command_sampling_strategy: CommandSamplingStrategy, geofence: Geofence
) -> CommandSamplingState:
    """Set up a command sampling state for testing"""
    return CommandSamplingState(robot, command_sampling_strategy, 1, geofence)


@pytest.fixture
def command() -> Command:
    """Set up a command for testing"""
    return np.array([0, 0])


@pytest.fixture
def step(command: Command) -> Step:
    """Set up a step object for testing"""
    return Step(command, 0)


@pytest.fixture
def drive(robot: Robot, command_sampling_strategy: CommandSamplingStrategy) -> Drive:
    """Set up a drive object for testing"""
    return Drive(robot, command_sampling_strategy, 1)


def test_GeofenceCreationState_start(geofence_creation_state: GeofenceCreationState):
    """Test if start sets the flag to true"""
    geofence_creation_state.start(0)
    assert geofence_creation_state.geofence_started


def test_GeofenceCreationState_pause(geofence_creation_state: GeofenceCreationState):
    """Test if pause sets the flag to false"""
    geofence_creation_state.geofence_started = True
    geofence_creation_state.pause()
    assert not geofence_creation_state.geofence_started


def test_GeofenceCreationState_run_not_started(geofence_creation_state: GeofenceCreationState):
    """Test if when not started then geofence points is not changed"""
    start_length = len(geofence_creation_state.geofence_points)

    geofence_creation_state.geofence_started = False
    geofence_creation_state.run(0)
    assert len(geofence_creation_state.geofence_points) == start_length


def test_GeofenceCreationState_run_in_thresold(geofence_creation_state: GeofenceCreationState):
    """Test if when robot is still in the distance thresold then no point is added to geofence"""
    start_length = len(geofence_creation_state.geofence_points)

    geofence_creation_state.geofence_started = True
    geofence_creation_state.distance_thresold_meters = 1
    geofence_creation_state.run(0)
    assert len(geofence_creation_state.geofence_points) == start_length


def test_GeofenceCreationState_run_out_of_thresold(geofence_creation_state: GeofenceCreationState):
    """Test if when robot is out of the distance thresold then its point is added to geofence"""
    geofence_creation_state.geofence_started = True
    geofence_creation_state.robot.pose = np.array([1, 0, 0])
    geofence_creation_state.distance_thresold_meters = 0.5
    geofence_creation_state.run(0)
    assert geofence_creation_state.geofence_points[-1][0] == geofence_creation_state.robot.pose[:2][0]
    assert geofence_creation_state.geofence_points[-1][1] == geofence_creation_state.robot.pose[:2][1]


def test_CommandSamplingState_start(command_sampling_state: CommandSamplingState):
    """Test if the start method adds a step if none is present"""
    assert command_sampling_state.current_step == None
    command_sampling_state.start(0)
    assert command_sampling_state.current_step != None


def test_CommandSamplingState_start_already_started(command_sampling_state: CommandSamplingState, step: Step):
    """Test if start does nothing when already started"""
    command_sampling_state.current_step = step
    command_sampling_state.start(0)
    assert command_sampling_state.current_step == step


def test_CommandSamplingState_pause(command_sampling_state: CommandSamplingState, step: Step):
    """Test if pause sets current step to None"""
    command_sampling_state.current_step = step
    command_sampling_state.pause()
    assert command_sampling_state.current_step == None


def test_CommandSamplingState_pause_already_paused(command_sampling_state: CommandSamplingState, step: Step):
    """Test if pause does nothing when already paused"""
    command_sampling_state.pause()
    assert command_sampling_state.current_step == None


def test_CommandSamplingState_run_when_paused(command_sampling_state: CommandSamplingState, step: Step):
    """Test if run does nothing when the state is paused"""
    command_sampling_state.run(0)

    assert command_sampling_state.current_step == None


def test_CommandSamplingState_run_outside_of_geofence(command_sampling_state: CommandSamplingState, step: Step):
    """Test if run sends the robot a new goal when outside of the geofence"""
    command_sampling_state.current_step = step
    command_sampling_state.robot.pose = np.array([2, 0, 0])
    command_sampling_state.robot.goal_reached = True
    command_sampling_state.run(0)

    assert command_sampling_state.current_step == None
    assert not command_sampling_state.robot.goal_reached


def test_CommandSamplingState_run_after_timestamp_delay(
    command_sampling_state: CommandSamplingState, command: Command, step: Step
):
    """Test if run sets the next command after the delay has passed"""
    command_sampling_state.current_step = step
    command_sampling_state.robot.pose = np.array([0.5, 0.5, 0])
    next_command = np.array([1, 0])
    command_sampling_state.next_command = next_command

    command_sampling_state.run(2000000000)

    assert (command_sampling_state.current_step.command == next_command).all()
    assert (command_sampling_state.next_command != next_command).all()


def test_Drive_change_state_changes_to_GeofenceCreationState(drive: Drive):
    """Test if change state changes the state to GeofenceCreationState if it is chosen"""
    drive.change_state(DriveStateEnum.geofence_creation)

    assert isinstance(drive.drive_state, GeofenceCreationState)


def test_Drive_change_state_no_change_to_state_if_is_CommandSamplingState(
    drive: Drive, command_sampling_state: CommandSamplingState
):
    """Test if change state does not modify the state if CommandSamplingState is chosen
    and the current state is CommandSamplingState"""
    drive.drive_state = command_sampling_state
    drive.change_state(DriveStateEnum.command_sampling)
    assert drive.drive_state == command_sampling_state


def test_Drive_start_starts_state(drive: Drive, geofence_creation_state: GeofenceCreationState):
    """Test if drive calls the drive() function of its state"""
    drive.drive_state = geofence_creation_state
    drive.start(0)
    assert geofence_creation_state.geofence_started


def test_Drive_pause_pauses_state(drive: Drive, geofence_creation_state: GeofenceCreationState):
    """Test if pause calls the pause() function of its state"""
    geofence_creation_state.geofence_started = True
    drive.drive_state = geofence_creation_state
    drive.pause()
    assert not geofence_creation_state.geofence_started


def test_Drive_run_runs_state(drive: Drive, geofence_creation_state: GeofenceCreationState):
    """Test if run calls the run() function of its state"""
    start_length = len(geofence_creation_state.geofence_points)
    drive.drive_state = geofence_creation_state
    geofence_creation_state.geofence_started = True
    geofence_creation_state.distance_thresold_meters = 1

    drive.run(0)

    assert len(geofence_creation_state.geofence_points) == start_length


def test_Drive_get_commands_if_GeofenceCreationState_returns_empty(drive: Drive):
    """Test if get_commands returns empty if it is in GeofenceCreationState"""
    result = drive.get_commands()

    assert len(result) == 0


def test_Drive_get_commands_if_CommandSamplingState_returns_commands(
    drive: Drive, command_sampling_state: CommandSamplingState
):
    """Test if get_commands returns the commands if it is in CommandSamplingState"""
    command_sampling_state.commands = [np.array([1, 1])]
    drive.drive_state = command_sampling_state
    result = drive.get_commands()

    assert (result == command_sampling_state.commands).all()


def test_Drive_get_geofence_points_if_GeofenceCreationState_returns_geofence_points(
    drive: Drive, geofence_creation_state: GeofenceCreationState
):
    """Test if get_geofence_points returns the geofence points if it is in GeofenceCreationState"""
    geofence_creation_state.geofence_points = [np.array([1, 1])]
    drive.drive_state = geofence_creation_state
    result = drive.get_geofence_points()

    assert (result == geofence_creation_state.geofence_points).all()


def test_Drive_get_geofence_points_if_CommandSamplingState_returns_empty(
    drive: Drive, command_sampling_state: CommandSamplingState
):
    """Test if get_geofence_points returns empty if it is in CommandSamplingState"""
    command_sampling_state.commands = [np.array([1, 1])]
    drive.drive_state = command_sampling_state
    result = drive.get_geofence_points()

    assert len(result) == 0


def test_Drive_get_geofence_returns_geofence(drive: Drive, geofence: Geofence):
    """Test if get_geofence returns the geofence"""
    drive.geofence = geofence

    assert drive.get_geofence() == geofence
