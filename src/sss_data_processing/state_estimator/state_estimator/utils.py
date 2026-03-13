import os
import time
import psutil
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
        log_dir = os.path.join(ws_root, "src/sss_data_processing/state_estimator/logs/data")
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

class NISLogger:
    HEADER = [
        "t",
        "nis"
    ]

    def __init__(self, datafile_name):
        ws_root = os.getcwd()
        log_dir = os.path.join(ws_root, "src/sss_data_processing/state_estimator/logs/data")
        os.makedirs(log_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filepath = os.path.join(log_dir, f"nis_{datafile_name}_{timestamp}.csv")

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
        log_dir = os.path.join(ws_root, "src/sss_data_processing/state_estimator/logs/data")
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

class PerformanceLogger:
    HEADER = [
        "t",
        "runtime_s",
        "cpu_percent",
        "ram_mb"
    ]

    def __init__(self):
        ws_root = os.getcwd()
        log_dir = os.path.join(ws_root, "src/sss_data_processing/state_estimator/logs/data")
        os.makedirs(log_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filepath = os.path.join(log_dir, f"performance_{timestamp}.csv")

        self.file = open(self.filepath, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.HEADER)

        self.proc = psutil.Process(os.getpid())
        self.t0 = None

    def start(self):
        self.t0 = time.perf_counter()

    def stop(self, t):
        if self.t0 is None:
            return

        runtime = time.perf_counter() - self.t0
        cpu = self.proc.cpu_percent(None) / psutil.cpu_count()
        ram = self.proc.memory_info().rss / (1024**2)

        self.writer.writerow([t, runtime, cpu, ram])
        self.t0 = None

    def close(self):
        self.file.close()