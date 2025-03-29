from dataclasses import asdict
import csv

from DRIVE_AGAIN.data.exported_types import Saveable


class CsvWriter:
    def __init__(self, headers):
        self.headers = headers
        self.data: list[Saveable] = []

    def save_data_to_file(self, filename: str):
        with open(filename, mode="w", newline="\n", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=self.headers)

            writer.writeheader()

            for row in self.data:
                writer.writerow(asdict(row))

    def save_line(self, line: Saveable):
        self.data.append(line)
