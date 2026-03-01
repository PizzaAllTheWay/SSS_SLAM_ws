import os
import csv
from datetime import datetime

class StateEstimateLogger:
    HEADER = [
        "t",
        "px", "py", "pz",
        "roll", "pitch", "yaw",
        "vx", "vy", "vz",
        "px_cov", "py_cov", "pz_cov",
        "roll_cov", "pitch_cov", "yaw_cov",
        "vx_cov", "vy_cov", "vz_cov"
    ]

    def __init__(self):
        ws_root = os.getcwd()
        log_dir = os.path.join(ws_root, "src/state_estimator/logs/data")
        os.makedirs(log_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filepath = os.path.join(log_dir, f"state_estimate_{timestamp}.csv")

        self.file = open(self.filepath, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.HEADER)

    def log(self, data: dict):
        row = [data.get(key, None) for key in self.HEADER]
        self.writer.writerow(row)

    def close(self):
        self.file.close()

class NISAHRSLogger:
    HEADER = [
        "t",
        "nis_ahrs"
    ]

    def __init__(self):
        ws_root = os.getcwd()
        log_dir = os.path.join(ws_root, "src/state_estimator/logs/data")
        os.makedirs(log_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filepath = os.path.join(log_dir, f"nis_ahrs_{timestamp}.csv")

        self.file = open(self.filepath, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.HEADER)

    def log(self, data: dict):
        row = [data.get(key, None) for key in self.HEADER]
        self.writer.writerow(row)

    def close(self):
        self.file.close()

class BenchmarkLogger:
    HEADER = [
        "t",
        "px", "py", "pz",
        "roll", "pitch", "yaw",
        "vx", "vy", "vz",
        "px_cov", "py_cov", "pz_cov",
        "roll_cov", "pitch_cov", "yaw_cov",
        "vx_cov", "vy_cov", "vz_cov"
    ]

    def __init__(self):
        ws_root = os.getcwd()
        log_dir = os.path.join(ws_root, "src/state_estimator/logs/data")
        os.makedirs(log_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filepath = os.path.join(log_dir, f"benchmark_{timestamp}.csv")

        self.file = open(self.filepath, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.HEADER)

    def log(self, data: dict):
        row = [data.get(key, None) for key in self.HEADER]
        self.writer.writerow(row)

    def close(self):
        self.file.close()