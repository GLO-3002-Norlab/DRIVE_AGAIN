import os

from DRIVE_AGAIN.common import Command, Pose
from DRIVE_AGAIN.saveable_types import Saveable
from DRIVE_AGAIN.csv_writer import CsvWriter
from DRIVE_AGAIN.saveable_types import DriveStep, Position6DOF


class DatasetRecorder:
    def __init__(self, datasets_folder: str):
        self.datasets_folder = datasets_folder
        self.step_id = 0

        self.writers: dict[type[Saveable], CsvWriter] = {}

        # Built-in recorders
        self._register(DriveStep)
        self._register(Position6DOF)

    def _register(self, saveable_type: type[Saveable]):
        self.writers[saveable_type] = CsvWriter(saveable_type)

    def register_custom_saveable(self, saveable_type: type[Saveable]):
        if saveable_type in self.writers:
            raise ValueError(f"{saveable_type.__name__} is already registered.")
        self._register(saveable_type)

    def save_custom_row(self, row: Saveable):
        saveable_type = type(row)
        if saveable_type not in self.writers:
            raise ValueError(f"{saveable_type.__name__} has not been registered.")
        self.writers[saveable_type].save_line(row)

    def save_command(self, command: Command, timestamp_ns: int):
        new_step = DriveStep(timestamp_ns, self.step_id, *command)
        self.writers[DriveStep].save_line(new_step)
        self.step_id += 1

    def save_pose(self, pose: Pose, timestamp_ns: int):
        pose_6DOF = Position6DOF(timestamp_ns, self.step_id, *pose)
        self.writers[Position6DOF].save_line(pose_6DOF)

    def save_poses(self, poses_array):
        for pose, timestamp_ns in poses_array:
            self.save_pose(pose, timestamp_ns)

    def save_experience(self, dataset_name: str):
        save_folder_path = os.path.join(self.datasets_folder, dataset_name)
        os.makedirs(save_folder_path, exist_ok=True)

        for writer in self.writers.values():
            writer.save_data_to_file(save_folder_path)
