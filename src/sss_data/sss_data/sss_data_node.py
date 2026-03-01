#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped
from marine_acoustic_msgs.msg import Dvl, RawSonarImage, SonarImageData
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time
from ament_index_python.packages import get_package_share_directory

from pathlib import Path
import time
import scipy.spatial.transform as transform
import numpy as np

from sss_data.sss_data_extract import SSSDataExtract



class SSSDataNode(Node):
    # Init (START) --------------------------------------------------
    def __init__(self):
        # Initialize ROS node
        super().__init__("sss_data_node")

        # Get ROS params
        self.declare_parameter("benchmark.orientation", [0.0, 0.0, 0.0])

        bench_orientation = self.get_parameter("benchmark.orientation").get_parameter_value().double_array_value

        self.R_bench_to_ned = transform.Rotation.from_euler('xyz', bench_orientation).as_matrix()
        
        # Specify path to CSV data -----
        pkg_share = Path(get_package_share_directory("sss_data"))
        data_path = pkg_share / "data"
        self.extractor = SSSDataExtract(data_path)

        # Publishers -----
        self.imu_pub   = self.create_publisher(Imu,           "/hardware/imu",             10)
        self.depth_pub = self.create_publisher(PointStamped,  "/hardware/depth",           10)
        self.dvl_pub   = self.create_publisher(Dvl,           "/hardware/dvl",             10)
        self.sonar_pub = self.create_publisher(RawSonarImage, "/hardware/side_scan_sonar", 10)

        self.benchmark_pub = self.create_publisher(Odometry, "/benchmark/state_estimate", 10)

        self.start_wall = time.time()

        # Preload -----
        self.next_imu             = self.extractor.get_next_imu()
        self.next_depth           = self.extractor.get_next_depth()
        self.next_dvl_vel_g       = self.extractor.get_next_dvl_velocity_ground()
        self.next_dvl_range       = self.extractor.get_next_dvl_range()
        self.next_dvl_speed_sound = self.extractor.get_next_dvl_speed_sound()
        self.next_sonar           = self.extractor.get_next_sonar()

        self.next_benchmark = self.extractor.get_next_benchmark_state_estimate()
        
        # Independent timers -----
        self.imu_timer             = self.create_timer(0.00001, self.imu_loop)
        self.depth_timer           = self.create_timer(0.001,   self.depth_loop)
        self.dvl_vel_g_timer       = self.create_timer(0.001,   self.dvl_vel_g_loop)
        self.dvl_range_timer       = self.create_timer(0.001,   self.dvl_range_loop)
        self.dvl_speed_sound_timer = self.create_timer(0.001,   self.dvl_speed_sound_loop)
        self.sonar_timer           = self.create_timer(0.001,   self.sonar_loop)

        self.benchmark_timer = self.create_timer(0.00001, self.benchmark_loop)
    # Init (START) --------------------------------------------------

    # Loops (START) --------------------------------------------------
    def imu_loop(self):
        if self.next_imu is None:
            self.imu_timer.cancel()
            self.check_shutdown()
            return

        elapsed = time.time() - self.start_wall
        if elapsed < self.next_imu["t"]:
            return

        self.publish_imu(self.next_imu)
        self.next_imu = self.extractor.get_next_imu()

    def depth_loop(self):
        if self.next_depth is None:
            self.depth_timer.cancel()
            self.check_shutdown()
            return

        elapsed = time.time() - self.start_wall
        if elapsed < self.next_depth["t"]:
            return

        self.publish_depth(self.next_depth)
        self.next_depth = self.extractor.get_next_depth()

    def dvl_vel_g_loop(self):
        if self.next_dvl_vel_g is None:
            self.dvl_vel_g_timer.cancel()
            self.check_shutdown()
            return

        elapsed = time.time() - self.start_wall
        if elapsed < self.next_dvl_vel_g["t"]:
            return
        
        dvl_flags = {
            "velocity_mode": Dvl.DVL_MODE_BOTTOM,
            "dvl_type": Dvl.DVL_TYPE_PISTON,
            "beam_ranges_valid": False,
            "beam_velocities_valid": True,
        }

        self.publish_dvl(self.next_dvl_vel_g, dvl_flags)
        self.next_dvl_vel_g = self.extractor.get_next_dvl_velocity_ground()

    def dvl_range_loop(self):
        if self.next_dvl_range is None:
            self.dvl_range_timer.cancel()
            self.check_shutdown()
            return

        elapsed = time.time() - self.start_wall
        if elapsed < self.next_dvl_range["t"]:
            return
        
        dvl_flags = {
            "velocity_mode": Dvl.DVL_MODE_BOTTOM,
            "dvl_type": Dvl.DVL_TYPE_PISTON,
            "beam_ranges_valid": self.next_dvl_range["num_good_beams"] > 0,
            "beam_velocities_valid": False,
        }

        self.publish_dvl(self.next_dvl_range, dvl_flags)
        self.next_dvl_range = self.extractor.get_next_dvl_range()

    def dvl_speed_sound_loop(self):
        if self.next_dvl_speed_sound is None:
            self.dvl_speed_sound_timer.cancel()
            self.check_shutdown()
            return

        elapsed = time.time() - self.start_wall
        if elapsed < self.next_dvl_speed_sound["t"]:
            return
        
        dvl_flags = {
            "velocity_mode": Dvl.DVL_MODE_BOTTOM,
            "dvl_type": Dvl.DVL_TYPE_PISTON,
            "beam_ranges_valid": False,
            "beam_velocities_valid": False,
        }

        self.publish_dvl(self.next_dvl_speed_sound, dvl_flags)
        self.next_dvl_speed_sound = self.extractor.get_next_dvl_speed_sound()

    def sonar_loop(self):
        if self.next_sonar is None:
            self.sonar_timer.cancel()
            self.check_shutdown()
            return

        elapsed = time.time() - self.start_wall
        if elapsed < self.next_sonar["t"]:
            return

        self.publish_sonar(self.next_sonar)
        self.next_sonar = self.extractor.get_next_sonar()

    def benchmark_loop(self):
        if self.next_benchmark is None:
            self.benchmark_timer.cancel()
            self.check_shutdown()
            return

        elapsed = time.time() - self.start_wall
        if elapsed < self.next_benchmark["t"]:
            return

        self.publish_benchmark(self.next_benchmark)
        self.next_benchmark = self.extractor.get_next_benchmark_state_estimate()
    # Loops (STOP) --------------------------------------------------

    # Shutdown Control (START) --------------------------------------------------
    def check_shutdown(self):
        if (
            self.next_imu is None and
            self.next_dvl_vel_g is None and
            self.next_dvl_range is None and
            self.next_dvl_speed_sound is None and
            self.next_sonar is None and
            self.next_depth is None
        ):
            self.get_logger().info("Playback finished")
            rclpy.shutdown()
    # Shutdown Control (STOP) --------------------------------------------------

    # Publishers (START) --------------------------------------------------
    def publish_imu(self, data):
        msg = Imu()

        sec = int(data["timestamp"])
        nanosec = int((data["timestamp"] - sec) * 1e9)
        msg.header.stamp = Time(sec=sec, nanosec=nanosec)
        msg.header.frame_id = "imu_link"

        qx, qy, qz, qw = data["quat"]
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        wx, wy, wz = data["gyro"]
        msg.angular_velocity.x = wx
        msg.angular_velocity.y = wy
        msg.angular_velocity.z = wz

        ax, ay, az = data["acc"]
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        self.imu_pub.publish(msg)

    def publish_depth(self, data):
        msg = PointStamped()

        sec = int(data["timestamp"])
        nanosec = int((data["timestamp"] - sec) * 1e9)
        msg.header.stamp = Time(sec=sec, nanosec=nanosec)
        msg.header.frame_id = "depth_link"

        msg.point.x = 0.0
        msg.point.y = 0.0
        msg.point.z = data["depth"]

        self.depth_pub.publish(msg)

    def publish_dvl(self, data, flags):
        msg = Dvl()

        sec = int(data["timestamp"])
        nanosec = int((data["timestamp"] - sec) * 1e9)
        msg.header.stamp = Time(sec=sec, nanosec=nanosec)
        msg.header.frame_id = "dvl_link"

        msg.velocity_mode = flags["velocity_mode"]
        msg.dvl_type = flags["dvl_type"]

        vx, vy, vz = data["velocity"]
        msg.velocity.x = vx
        msg.velocity.y = vy
        msg.velocity.z = vz

        msg.altitude = data["altitude"]
        msg.course_gnd = data["course_gnd"]
        msg.speed_gnd = data["speed_gnd"]
        msg.num_good_beams = data["num_good_beams"]
        msg.sound_speed = data["sound_speed"]

        msg.beam_ranges_valid     = flags["beam_ranges_valid"]
        msg.beam_velocities_valid = flags["beam_velocities_valid"]

        msg.range = data["range"]

        self.dvl_pub.publish(msg)

    def publish_sonar(self, data):
        msg = RawSonarImage()

        sec = int(data["timestamp"])
        nanosec = int((data["timestamp"] - sec) * 1e9)
        msg.header.stamp = Time(sec=sec, nanosec=nanosec)
        msg.header.frame_id = "sonar_link"

        msg.ping_info.frequency = data["frequency"]
        msg.ping_info.sound_speed = 1500.0

        msg.sample0 = 0
        msg.samples_per_beam = data["samples_per_beam"]

        range_res = data["max_range"] / data["samples_per_beam"]
        msg.sample_rate = msg.ping_info.sound_speed / (2.0 * range_res)

        msg.image = SonarImageData()
        msg.image.is_bigendian = False
        msg.image.dtype = SonarImageData.DTYPE_UINT8
        msg.image.beam_count = 1
        msg.image.data = data["data"]

        self.sonar_pub.publish(msg)

    def publish_benchmark(self, data):
        R = self.R_bench_to_ned

        msg = Odometry()

        sec = int(data["timestamp"])
        nanosec = int((data["timestamp"] - sec) * 1e9)
        msg.header.stamp = Time(sec=sec, nanosec=nanosec)
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"

        # Position (ENU → NED)
        p_enu = np.array(data["position"])
        p_ned = R @ p_enu

        msg.pose.pose.position.x = p_ned[0]
        msg.pose.pose.position.y = p_ned[1]
        msg.pose.pose.position.z = p_ned[2]

        # Orientation (ENU → NED)
        q_enu = transform.Rotation.from_quat(data["orientation"])
        R_enu = q_enu.as_matrix()
        R_ned = R @ R_enu
        q_ned = transform.Rotation.from_matrix(R_ned).as_quat()

        msg.pose.pose.orientation.x = q_ned[0]
        msg.pose.pose.orientation.y = q_ned[1]
        msg.pose.pose.orientation.z = q_ned[2]
        msg.pose.pose.orientation.w = q_ned[3]

        # Pose Covariances
        msg.pose.covariance = data["pose_covariance"]

        # Twist
        v_enu = np.array(data["lin_vel"])
        v_ned = R @ v_enu

        msg.twist.twist.linear.x = v_ned[0]
        msg.twist.twist.linear.y = v_ned[1]
        msg.twist.twist.linear.z = v_ned[2]

        # Twist Covariances
        msg.twist.covariance = data["twist_covariance"]

        # Publish to ROS
        self.benchmark_pub.publish(msg)
    # Publishers (STOP) --------------------------------------------------



def main(args=None):
    rclpy.init(args=args)
    node = SSSDataNode()
    rclpy.spin(node)



if __name__ == "__main__":
    main()