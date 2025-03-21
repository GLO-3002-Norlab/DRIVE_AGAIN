import csv
import os
from typing import Any

from data.robot_data import RobotData


# TODO : Find a better name for this class
class DataUtil:
    folder: str

    def __init__(self, folder: str) -> None:
        self.folder = folder

        if not os.path.isdir(self.folder):
            os.makedirs(self.folder)

    def exportCSV(self, obj: dict, fileName: str) -> None:
        assert os.path.isdir(self.folder)

        f = open(self.folder + "/" + fileName, mode="w", newline="")

        writer = csv.DictWriter(f, fieldnames=obj.keys())
        writer.writeheader()
        writer.writerow(obj)

        f.close()

    def importCSV(self, fileName: str) -> Any:
        assert os.path.isdir(self.folder)

        f = open(self.folder + "/" + fileName, mode="r")

        reader = csv.DictReader(f)
        obj = [row for row in reader][0]

        f.close()

        return obj
