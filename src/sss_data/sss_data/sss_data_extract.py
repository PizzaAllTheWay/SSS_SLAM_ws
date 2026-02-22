import csv
from pathlib import Path
from tf_transformations import quaternion_from_euler
import math
import bisect



class SSSDataExtract:
    def __init__(self, data_dir: str):
        self.data_dir = Path(data_dir)

        self.start_time = self._compute_global_start_time()
        
        # IMU data
        self.acc_reader     = self._open_csv("Acceleration.csv")
        self.ang_vel_reader = self._open_csv("AngularVelocity.csv")
        self.euler_reader   = self._open_csv("EulerAngles.csv")

        # Depth data (From Pressure sensor)
        self.depth_reader = self._open_csv("Depth.csv")

        # DVL data
        self.ground_vel_reader = self._open_csv("GroundVelocity.csv")
        self.distance_reader   = self._open_csv("Distance.csv")
        self.sound_reader      = self._open_csv("SoundSpeed.csv")

        self.distance_alt_time = 0
        self.distance_alt_last = 0.0

        self.sound_speed_time = 0
        self.sound_speed_last = 0.0

        # Sonar data (Side scan sonar + Echo sounder sonar)
        self.sonar_reader = self._open_csv("SonarData.csv")

    def _open_csv(self, name):
        fp = open(self.data_dir / name, "r")
        reader = csv.reader(fp)
        next(reader) # Skip head
        return reader

    def _compute_global_start_time(self):
        min_time = None
        for file in self.data_dir.glob("*.csv"):
            with open(file, "r") as f:
                reader = csv.reader(f)
                next(reader, None)
                first = next(reader, None)
                if first:
                    t = float(first[0])
                    min_time = t if min_time is None else min(min_time, t)

        if min_time is None:
            raise RuntimeError("No valid CSV data found")

        return min_time

    def get_next_imu(self):
        acc_row = next(self.acc_reader, None)
        ang_row = next(self.ang_vel_reader, None)
        eul_row = next(self.euler_reader, None)

        if not acc_row or not ang_row or not eul_row:
            return None

        # timestamp (use acceleration as reference because angular velocity and euler angle have the same time stamps)
        timestamp = float(acc_row[0])
        rel_time = timestamp - self.start_time

        # linear acceleration
        ax = float(acc_row[4])
        ay = float(acc_row[5])
        az = float(acc_row[6])

        # angular velocity (assumed rad/s already)
        wx = float(ang_row[4])
        wy = float(ang_row[5])
        wz = float(ang_row[6])

        # euler angles (assumed radians: roll, pitch, yaw)
        roll  = float(eul_row[4])
        pitch = float(eul_row[5])
        yaw   = float(eul_row[6])

        # convert euler → quaternion
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

        return {
            "t": rel_time,
            "timestamp": timestamp,
            "acc": (ax, ay, az),
            "gyro": (wx, wy, wz),
            "quat": (qx, qy, qz, qw),
        }
    
    def get_next_depth(self):
        depth_row = next(self.depth_reader, None)

        if not depth_row:
            return None

        # timestamp
        timestamp = float(depth_row[0])
        rel_time = timestamp - self.start_time

        # depth value (m)
        depth = float(depth_row[3])  

        return {
            "t": rel_time,
            "timestamp": timestamp,
            "depth": depth,
        }
    
    def get_next_dvl(self):
        while True:
            gv_row = next(self.ground_vel_reader, None)
            if not gv_row:
                return None

            vx = float(gv_row[4])
            vy = float(gv_row[5])
            vz = float(gv_row[6])

            if abs(vx) > 30 or abs(vy) > 30 or abs(vz) > 30:
                continue

            timestamp = float(gv_row[0])
            rel_time = timestamp - self.start_time

            speed_gnd = math.sqrt(vx**2 + vy**2)
            course_gnd = math.atan2(vy, vx)

            # ALTITUDE + BEAM RANGES -----
            #
            # We do NOT try to align timestamps with DVL.
            # Distance.csv is a forward-only stream.
            #
            # Every time beam 66 appears, a full beam block (62–66) is complete.
            # So we:
            #   • Keep a rolling buffer of the last 5 rows
            #   • When we see beam 66 → that buffer is one full beam set
            #
            # Geometry:
            #   62–65 → tilted 30° beams (slant range)
            #   66    → vertical beam (true altitude)
            #
            # Altitude logic:
            #   • Prefer vertical beam (66)
            #   • Fallback → mean(slant) * cos(30°)
            #   • cos(30°) = sqrt(3)/2

            altitude = 0.0
            num_good_beams = 0
            beam_ranges = [0.0, 0.0, 0.0, 0.0]

            latest_block = []

            while True:
                row = next(self.distance_reader, None)
                if not row:
                    break

                # Maintain rolling last 5 rows (max beams per block)
                latest_block.append(row)
                if len(latest_block) > 5:
                    latest_block.pop(0)

                # Beam 66 marks end of a complete beam set
                if int(row[2]) == 66:
                    break

            # Process the completed block
            vertical_altitude = None
            tilted_ranges = []

            for row in latest_block:
                if row[3].strip().upper() != "VALID":
                    continue

                beam_id = int(row[2])
                rng = float(row[6])

                if beam_id == 66:
                    vertical_altitude = rng

                elif beam_id in (62, 63, 64, 65):
                    idx = beam_id - 62
                    beam_ranges[idx] = rng
                    tilted_ranges.append(rng)

            # Decide altitude source
            if vertical_altitude is not None:
                altitude = vertical_altitude
                num_good_beams = 1
            elif tilted_ranges:
                num_good_beams = len(tilted_ranges)
                mean_slant = sum(tilted_ranges) / num_good_beams
                altitude = (math.sqrt(3) / 2.0) * mean_slant

            beam_ranges_valid = num_good_beams > 0

            # SOUND SPEED -----
            sound_speed = 0.0

            while True:
                if self.sound_speed_time > timestamp:
                    break

                row = next(self.sound_reader, None)
                if not row:
                    break

                self.sound_speed_time = float(row[0])
                self.sound_speed_last = float(row[3])

            if abs(timestamp - self.sound_speed_time) < 1.0:
                sound_speed = self.sound_speed_last

            return {
                "t": rel_time,
                "timestamp": timestamp,

                "velocity": (vx, vy, vz),

                "altitude": altitude,
                "course_gnd": course_gnd,
                "speed_gnd": speed_gnd,
                "num_good_beams": num_good_beams,
                "sound_speed": sound_speed,
                "beam_ranges_valid": beam_ranges_valid,

                "beam_ranges": beam_ranges,
            }
    
    def get_next_sonar(self):
        # Keep reading until we find a SIDESCAN entry
        while True:
            row = next(self.sonar_reader, None)

            # End of file
            if not row:
                return None

            # Skip non-sidescan rows (e.g. ECHOSOUNDER)
            if row[3].strip() != "SIDESCAN":
                continue

            # Extract fields clearly and explicitly -----
            timestamp = float(row[0])               # absolute UNIX time
            rel_time  = timestamp - self.start_time # playback relative time

            frequency = float(row[4]) # sonar center frequency [Hz]
            min_range = float(row[5]) # minimum range [m]
            max_range = float(row[6]) # maximum range [m]
            bits      = int(row[7])   # bits per sample
            scale     = float(row[8]) # scale factor

            hex_string = row[10].strip()           # raw hex intensity string
            raw_bytes  = bytes.fromhex(hex_string) # convert hex → uint8 bytes

            samples_per_beam = len(raw_bytes) # number of intensity samples

            # Return clean structured packet -----
            return {
                "t": rel_time,
                "timestamp": timestamp,
                "frequency": frequency,
                "min_range": min_range,
                "max_range": max_range,
                "bits": bits,
                "scale": scale,
                "samples_per_beam": samples_per_beam,
                "data": raw_bytes,
            }