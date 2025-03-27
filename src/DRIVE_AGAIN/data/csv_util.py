import csv
import os
from pathlib import Path
from typing import Any


class CSVUtil:
    folder: Path

    def __init__(self, folder: Path) -> None:
        self.folder = folder

        if not os.path.isdir(self.folder):
            os.makedirs(self.folder)

    def export_csv(self, obj: dict, fileName: Path) -> None:
        assert os.path.isdir(self.folder)

        f = open(self.folder / fileName, mode="w", newline="")

        writer = csv.DictWriter(f, fieldnames=obj.keys())
        writer.writeheader()
        writer.writerow(obj)

        f.close()

    def import_csv(self, fileName: str) -> Any:
        assert os.path.isdir(self.folder)

        f = open(self.folder / fileName, mode="r")

        reader = csv.DictReader(f)
        obj = [row for row in reader][0]

        f.close()

        return obj
