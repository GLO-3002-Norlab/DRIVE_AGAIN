import csv
import os
from dataclasses import asdict
from typing import Generic, Type, TypeVar

from DRIVE_AGAIN.data_types import Serializable

T = TypeVar("T", bound=Serializable)


class CsvReader(Generic[T]):
    def __init__(self, loadable_type: Type[T]):
        self.loadable_type = loadable_type
        self.data: list[T] = []

    def from_dict(self, data: dict) -> T:
        return self.loadable_type(**data)

    # def load_line(self, line: T):
    #     if not isinstance(line, self.loadable_type):
    #         raise TypeError(f"Expected type {self.loadable_type.__name__}, got {type(line).__name__}")
    #     self.data.append(line)

    def load_data_from_file(self, path: str):
        filename = os.path.join(path, self.loadable_type.get_filename() + ".csv")
        with open(filename, mode="r", newline="\n", encoding="utf-8") as file:
            reader = csv.DictReader(file, fieldnames=self.loadable_type.headers())

            self.data = [self.from_dict(row) for row in reader]
