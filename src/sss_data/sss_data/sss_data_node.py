#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from marine_acoustic_msgs.msg import Dvl, RawSonarImage, SonarImageData
from builtin_interfaces.msg import Time
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import time

from sss_data.sss_data_extract import SSSDataExtract



class SSSDataNode(Node):
    # Init (START) --------------------------------------------------
    def __init__(self):
        super().__init__("sss_data_node")
        
        # Specify path to CSV data -----
        pkg_share = Path(get_package_share_directory("sss_data"))
        data_path = pkg_share / "data"
        self.extractor = SSSDataExtract(data_path)

        # Publishers -----
        self.imu_pub   = self.create_publisher(Imu,           "/hardware/imu",             10)
        self.depth_pub = self.create_publisher(Float64,       "/hardware/depth",           10)
        self.dvl_pub   = self.create_publisher(Dvl,           "/hardware/dvl",             10)
        self.sonar_pub = self.create_publisher(RawSonarImage, "/hardware/side_scan_sonar", 10)

        self.start_wall = time.time()

        # Preload -----
        self.next_imu   = self.extractor.get_next_imu()
        self.next_depth = self.extractor.get_next_depth()
        self.next_dvl   = self.extractor.get_next_dvl()
        self.next_sonar = self.extractor.get_next_sonar()
        
        # Independent timers -----
        self.imu_timer   = self.create_timer(0.00001, self.imu_loop)
        self.depth_timer = self.create_timer(0.001,   self.depth_loop)
        self.dvl_timer   = self.create_timer(0.001,   self.dvl_loop)
        self.sonar_timer = self.create_timer(0.001,   self.sonar_loop)
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
            self.dvl_timer.cancel()
            self.check_shutdown()
            return

        elapsed = time.time() - self.start_wall
        if elapsed < self.next_depth["t"]:
            return

        self.publish_depth(self.next_depth)
        self.next_depth = self.extractor.get_next_depth()

    def dvl_loop(self):
        if self.next_dvl is None:
            self.dvl_timer.cancel()
            self.check_shutdown()
            return

        elapsed = time.time() - self.start_wall
        if elapsed < self.next_dvl["t"]:
            return

        self.publish_dvl(self.next_dvl)
        self.next_dvl = self.extractor.get_next_dvl()

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
    # Loops (STOP) --------------------------------------------------

    # Shutdown Control (START) --------------------------------------------------
    def check_shutdown(self):
        if (
            self.next_imu is None and
            self.next_dvl is None and
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
        msg = Float64()
        msg.data = data["depth"]
        self.depth_pub.publish(msg)

    def publish_dvl(self, data):
        msg = Dvl()

        sec = int(data["timestamp"])
        nanosec = int((data["timestamp"] - sec) * 1e9)
        msg.header.stamp = Time(sec=sec, nanosec=nanosec)
        msg.header.frame_id = "dvl_link"

        msg.velocity_mode = Dvl.DVL_MODE_BOTTOM
        msg.dvl_type = Dvl.DVL_TYPE_PISTON

        vx, vy, vz = data["velocity"]
        msg.velocity.x = vx
        msg.velocity.y = vy
        msg.velocity.z = vz

        msg.altitude = data["altitude"]
        msg.course_gnd = data["course_gnd"]
        msg.speed_gnd = data["speed_gnd"]
        msg.num_good_beams = data["num_good_beams"]
        msg.sound_speed = data["sound_speed"]
        msg.beam_ranges_valid = data["beam_ranges_valid"]

        msg.range = data["beam_ranges"]

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
    # Publishers (STOP) --------------------------------------------------



def main(args=None):
    rclpy.init(args=args)
    node = SSSDataNode()
    rclpy.spin(node)



if __name__ == "__main__":
    main()