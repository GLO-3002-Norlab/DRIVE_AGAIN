from DRIVE_AGAIN.common import Command, Pose, is_same_command
import os
from DRIVE_AGAIN.data.csv_writer import CsvWriter
from DRIVE_AGAIN.data.exported_types import DriveStep, Position6DOF


class DatasetRecorder:
    STEPS_FILENAME = "steps.csv"
    POSITIONS_FILENAME = "positions.csv"

    def __init__(self, datasets_folder: str):
        self.datasets_folder = datasets_folder
        self.step_id = 0

        self.positions_recorder = CsvWriter(Position6DOF.headers())
        self.step_recorder = CsvWriter(DriveStep.headers())

    def save_command(self, command: Command, timestamp_ns: int):
        new_step = DriveStep(timestamp_ns, self.step_id, *command)
        self.step_recorder.save_line(new_step)
        self.step_id += 1

    def save_pose(self, pose: Pose, timestamp_ns: int):
        pose_6DOF = Position6DOF(timestamp_ns, self.step_id, *pose)
        self.positions_recorder.save_line(pose_6DOF)

    def save_poses(self, poses_array):
        for pose, timestamp_ns in poses_array:
            self.save_pose(pose, timestamp_ns)

    def save_experience(self, dataset_name: str):
        save_folder_path = os.path.join(self.datasets_folder, dataset_name)

        if not os.path.exists(save_folder_path):
            os.makedirs(save_folder_path)

        positions_filepath = os.path.join(save_folder_path, self.POSITIONS_FILENAME)
        steps_filepath = os.path.join(save_folder_path, self.STEPS_FILENAME)

        self.positions_recorder.save_data_to_file(positions_filepath)
        self.step_recorder.save_data_to_file(steps_filepath)
