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

        # Benchmarks data for state estimate testing, this is current implementation of navigation on board
        self.estimate_reader             = self._open_csv("EstimatedState.csv")
        self.estimate_uncertainty_reader = self._open_csv("NavigationUncertainty.csv")

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
    
    def get_next_dvl_velocity_ground(self):
        while True:
            vel_g_row = next(self.ground_vel_reader, None)

            if not vel_g_row:
                return None
            
            # Extract the velocities
            vx = float(vel_g_row[4])
            vy = float(vel_g_row[5])
            vz = float(vel_g_row[6])

            # If any dont make sense it will be ignored
            if abs(vx) > 30 or abs(vy) > 30 or abs(vz) > 30:
                continue

            # timestamp
            timestamp = float(vel_g_row[0])
            rel_time = timestamp - self.start_time

            # Calculate the other extra options
            course_gnd = math.atan2(vy, vx)
            speed_gnd = math.sqrt(vx**2 + vy**2)

            return {
                "t": rel_time,
                "timestamp": timestamp,

                "velocity": (vx, vy, vz),

                "altitude": 0.0,
                "course_gnd": course_gnd,
                "speed_gnd": speed_gnd,
                "num_good_beams": 0,
                "sound_speed": 0.0,

                "range": [0.0]*4,
            }
        
    def get_next_dvl_range(self):
        # Read Distance.csv until we can form ONE valid "range/altitude measurement"
        # for a single timestamp. If there are no valid beams at a timestamp,
        # keep scanning forward until we find a timestamp that has at least one VALID beam.
        while True:
            # Step 1: read one row (this gives us the next timestamp group)
            row = next(self.distance_reader, None)
            if not row:
                return None 

            # Step 2: extract the timestamp and compute playback relative time
            timestamp = float(row[0])
            rel_time = timestamp - self.start_time

            # We will collect all beams that share this exact timestamp
            current_time = timestamp

            # This list will store ONLY beams marked VALID in this 62..66 block
            beams_valid = []

            # We assume row is already at beam 62 when entering here
            while True:
                # Keep only VALID beams
                if row[3].strip().upper() == "VALID":
                    beams_valid.append(row)

                # If this is beam 66, the block is complete → STOP
                if int(row[2]) == 66:
                    break

                # Move to next beam in the fixed 62..66 sequence
                row = next(self.distance_reader, None)
                if not row:
                    return None  # EOF safety

            # Step 4: if there were zero valid beams at this timestamp, try next timestamp
            if not beams_valid:
                continue

            # Step 5: prepare outputs with safe defaults (ROS messages cannot use None)
            altitude = 0.0                 # final "altitude" output [m]
            beam_ranges = [0.0] * 4        # ranges for angled beams 62..65 (slant ranges)
            num_good_beams = 0             # how many beams contributed (for Dvl.msg field)

            # vertical beam (66) is preferred if present
            vertical_altitude = None

            # store valid slant ranges from tilted beams so we can average them if needed
            tilted_ranges = []

            # Step 6: parse the valid beams we collected
            for b in beams_valid:
                beam_id = int(b[2])   # beam number (62..66)
                rng = float(b[6])     # measured range [m]

                # Primary vertical beam: gives true altitude directly
                if beam_id == 66:
                    vertical_altitude = rng

                # Secondary tilted beams (slant ranges). Save them and also store into array.
                elif beam_id in (62, 63, 64, 65):
                    idx = beam_id - 62           # map 62->0, 63->1, 64->2, 65->3
                    beam_ranges[idx] = rng
                    tilted_ranges.append(rng)

            # Step 7: decide how to compute altitude
            # If vertical beam exists and is valid -> use it (best)
            if vertical_altitude is not None:
                altitude = vertical_altitude
                num_good_beams = 1

            # Otherwise, use the mean of the tilted/slant beams and convert to altitude.
            # For a 30° Janus configuration:
            # altitude = cos(30°) * mean(slant_range) = (sqrt(3)/2) * mean(slant_range)
            else:
                num_good_beams = len(tilted_ranges)
                mean_slant = sum(tilted_ranges) / num_good_beams
                altitude = (math.sqrt(3) / 2.0) * mean_slant

            # tep 8: return a fully-populated dict with defaults for unused fields
            return {
                "t": rel_time,
                "timestamp": timestamp,

                "velocity": (0.0, 0.0, 0.0),

                "altitude": altitude,
                "course_gnd": 0.0,
                "speed_gnd": 0.0,
                "num_good_beams": num_good_beams,
                "sound_speed": 0.0,

                "range": beam_ranges,
            }
        
    def get_next_dvl_speed_sound(self):
        row = next(self.sound_reader, None)
        if not row:
            return None

        timestamp = float(row[0])
        rel_time = timestamp - self.start_time

        sound_speed = float(row[3])
    
        return {
            "t": rel_time,
            "timestamp": timestamp,

            "velocity": (0.0, 0.0, 0.0),

            "altitude": 0.0,
            "course_gnd": 0.0,
            "speed_gnd": 0.0,
            "num_good_beams": 0,
            "sound_speed": sound_speed,

            "range": [0.0]*4,
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
        
    def get_next_benchmark_state_estimate(self):
        # Read one row from each (assumed synced)
        est_row = next(self.estimate_reader, None)
        unc_row = next(self.estimate_uncertainty_reader, None)
        if not est_row or not unc_row:
            return None

        # Timestamp
        timestamp = float(est_row[0])
        rel_time = timestamp - self.start_time

        # EstimatedState.csv
        # cols: 
        # t, system, entity
        # x, y, z,
        # phi (roll), theta (pitch), psi (yaw),
        # p (roll rate), q (pitch rate), r (yaw rate),
        # u (surge), v (sway), w (heave),
        # bias_psi (yaw bias), bias_r (yaw rate bias)
        x = float(est_row[6])
        y = float(est_row[7])
        z = float(est_row[8])

        roll  = float(est_row[9])    # phi
        pitch = float(est_row[10])   # theta
        yaw   = float(est_row[11])   # psi

        x_vel = float(est_row[12])   # u (body x)
        y_vel = float(est_row[13])   # v (body y)
        z_vel = float(est_row[14])   # w (body z)

        # NavigationUncertainty.csv (same column layout as EstimatedState.csv)
        # cols:
        # 0 t, 1 system, 2 entity,
        # 3 x, 4 y, 5 z,
        # 6 phi(roll), 7 theta(pitch), 8 psi(yaw),
        # 9 p(roll_rate), 10 q(pitch_rate), 11 r(yaw_rate),
        # 12 u, 13 v, 14 w,
        # 15 bias_psi, 16 bias_r
        # Pose 
        var_x     = float(unc_row[3])
        var_y     = float(unc_row[4])
        var_z     = float(unc_row[5])
        var_roll  = float(unc_row[6])
        var_pitch = float(unc_row[7])
        var_yaw   = float(unc_row[8])

        # Twist
        var_x_vel      = float(unc_row[12])
        var_y_vel      = float(unc_row[13])
        var_z_vel      = float(unc_row[14])

        # Index (6x6 start form 0 index so +1)
        row_size = 6
        i = row_size + 1
        matrix_size = row_size * row_size

        # Pose covariance is [x y z roll pitch yaw] as 6x6 row-major (36 elems)
        # [var_x, 0    , 0    , 0       , 0        , 0      ]
        # [0    , var_y, 0    , 0       , 0        , 0      ]
        # [0    , 0    , var_z, 0       , 0        , 0      ]
        # [0    , 0    , 0    , var_roll, 0        , 0      ]
        # [0    , 0    , 0    , 0       , var_pitch, 0      ]
        # [0    , 0    , 0    , 0       , 0        , var_yaw]
        pose_cov = [0.0] * matrix_size
        pose_cov[i*0] = var_x
        pose_cov[i*1] = var_y
        pose_cov[i*2] = var_z
        pose_cov[i*3] = var_roll
        pose_cov[i*4] = var_pitch
        pose_cov[i*5] = var_yaw

        # Twist covariance is [vx vy vz wx wy wz] as 6x6 row-major
        # Map body linear (u,v,w) -> (vx,vy,vz) and body angular (p,q,r) -> (wx,wy,wz)
        # [var_x_vel, 0        , 0        , 0            , 0             , 0           ]
        # [0        , var_y_vel, 0        , 0            , 0             , 0           ]
        # [0        , 0        , var_z_vel, 0            , 0             , 0           ]
        # [0        , 0        , 0        , var_roll_rate, 0             , 0           ]
        # [0        , 0        , 0        , 0            , var_pitch_rate, 0           ]
        # [0        , 0        , 0        , 0            , 0             , var_yaw_rate]
        twist_cov = [0.0] * matrix_size
        twist_cov[i*0] = var_x_vel
        twist_cov[i*1] = var_y_vel
        twist_cov[i*2] = var_z_vel

        # convert euler -> quaternion (roll, pitch, yaw)
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

        return {
            "t": rel_time,
            "timestamp": timestamp,

            # Pose
            "position": (x, y, z),
            "orientation": (qx, qy, qz, qw),
            "pose_covariance": pose_cov,

            # Twist
            "lin_vel": (x_vel, y_vel, z_vel),
            "twist_covariance": twist_cov,
        }
