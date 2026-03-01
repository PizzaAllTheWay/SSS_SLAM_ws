#!/usr/bin/env python3
import numpy as np
import scipy.spatial.transform as transform

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

from ukfm.ukf.ukf import UKF
from ukfm.model.inertial_navigation import INERTIAL_NAVIGATION as MODEL

from state_estimator.utils import BenchmarkLogger, NISAHRSLogger, StateEstimateLogger



class StateEstimatorNode(Node):
    # Aiding Measurement State Transforms (START) --------------------------------------------------
    def h_orientation(self, state: MODEL.STATE):
        return transform.Rotation.from_matrix(state.Rot).as_rotvec()
    # Aiding Measurement State Transforms (STOP) --------------------------------------------------

    # INITIALIZE (START) --------------------------------------------------
    def __init__(self):
        # Initialize node
        super().__init__('state_estimator_node')

        # Initialize ROS subscribers
        self.sub_imu   = self.create_subscription(Imu         , '/hardware/imu'  , self.imu_callback  , 1)
        self.sub_depth = self.create_subscription(PointStamped, '/hardware/depth', self.depth_callback, 1)

        # Initialize ROS publishers
        self.pub_odom = self.create_publisher(Odometry,'/sss_slam/data_processing/state_estimate',10)

        # Initialize and get ROS parameters
        self.declare_parameter("log", False)
        self.declare_parameter("imu.orientation", [0.0, 0.0, 0.0])
        self.declare_parameter("imu.freq"       , 0)
        self.declare_parameter("imu.acc.std"    , 0.0)
        self.declare_parameter("imu.gyro.std"   , 0.0)
        self.declare_parameter("imu.ahrs.std"   , 0.0)
        self.declare_parameter("ukfm.P_0", [0.0]*9)

        self.LOG = self.get_parameter("log").value # An extra flag you can set to enable loging of data for debugging and analysis of the filter
        imu_orientation = self.get_parameter("imu.orientation").get_parameter_value().double_array_value
        imu_freq        = self.get_parameter("imu.freq").get_parameter_value().integer_value
        imu_acc_std     = self.get_parameter("imu.acc.std").get_parameter_value().double_value
        imu_gyro_std    = self.get_parameter("imu.gyro.std").get_parameter_value().double_value
        imu_ahrs_std    = self.get_parameter("imu.ahrs.std").get_parameter_value().double_value
        P_0 = self.get_parameter("ukfm.P_0").get_parameter_value().double_array_value
        
        self.get_logger().info(f"log: {self.LOG}")
        self.get_logger().info(f"imu.orientation: {imu_orientation}")
        self.get_logger().info(f"imu.freq:        {imu_freq}")
        self.get_logger().info(f"imu.acc.std:     {imu_acc_std}")
        self.get_logger().info(f"imu.gyro.std:    {imu_gyro_std}")
        self.get_logger().info(f"imu.ahrs.std:    {imu_ahrs_std}")
        self.get_logger().info(f"ukfm.P_0: {P_0}")

        # UKF model
        self.model = MODEL(T=1, imu_freq=imu_freq)  # T dummy (not used online)

        # Initialize location of sensors
        self.R_imu_to_ned = transform.Rotation.from_euler('xyz', imu_orientation).as_matrix()
        self.get_logger().info(f"R_imu_to_ned:\n{np.array2string(self.R_imu_to_ned, precision=3, suppress_small=True)}")

        # Initialize state
        Rot0 = np.eye(3)
        v0 = np.zeros(3)
        p0 = np.zeros(3)

        state0 = self.model.STATE(Rot=Rot0, v=v0, p=p0)

        P_0_matrix = np.diag(P_0)

        # Initialize process noise
        Q = np.diag([
            imu_gyro_std**2, imu_gyro_std**2, imu_gyro_std**2,
            imu_acc_std**2 , imu_acc_std**2 , imu_acc_std**2
        ])

        # Initialize aiding measurement noise
        self.R_ahrs = imu_ahrs_std**2 * np.eye(3)

        # Initialize filter
        self.ukf = UKF(
            state0=state0,
            P0=P_0_matrix,
            f=self.model.f,
            h=self.h_orientation,
            Q=Q,
            R=self.R_ahrs,
            phi=self.model.phi,
            phi_inv=self.model.phi_inv,
            alpha=np.ones(9) * 1e-3
        )

        # For tracking time so filter can propagate through time accurately
        self.last_time = None

        # This part is purely for logging and debugging
        # If Logging flag is enabled we add a logger for post processing purposes
        # Then we can analyze data later down the line to debug state estimator
        if self.LOG:
            self.state_estimate_logger = StateEstimateLogger()

            self.nis_ahrs_logger = NISAHRSLogger()

            self.benchmark_logger = BenchmarkLogger()
            self.sub_benchmark = self.create_subscription(Odometry, '/benchmark/state_estimate', self.benchmark_callback, 1)
    # INITIALIZE (STOP) --------------------------------------------------

    # Topic Handlers (START) --------------------------------------------------
    def imu_callback(self, msg: Imu):
        # Convert ROS time data structure to readable time in precise seconds
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # For the first time when we get first IMU data package, the time frame has not yet been established
        # So we just record the time stamp and next time we get a new IMU data package we can finally start processing
        if self.last_time is None:
            self.last_time = t
            return

        # Get the time difference between last IMU data package and now
        dt = t - self.last_time
        self.last_time = t

        # Format ROS IMU data to UKF-M lib format
        # Important to translate form IMU to NED frame before passing down the values because filter lives in NED and IMU lives in ENU
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        gyro = self.R_imu_to_ned @ gyro
        acc  = self.R_imu_to_ned @ acc

        # Set up the input data structure that will propagate IMU data through the filter
        omega = self.model.INPUT(gyro=gyro, acc=acc)

        # Propagate IMU data
        self.ukf.propagation(omega, dt)

        # Format ROS AHRS data to be in same format as the filter wants
        # Important to translate form IMU to NED frame before passing down the values because filter lives in NED and IMU lives in ENU
        R_meas = transform.Rotation.from_quat([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]).as_matrix()

        R_meas = self.R_imu_to_ned @ R_meas @ self.R_imu_to_ned.T

        # AHRS update (orientation)
        y = transform.Rotation.from_matrix(R_meas).as_rotvec()
        self.ukf.update(y)

        # Publish state estimate
        self.publish_state_estimate(msg.header)

        # This only runs if "log" flag was activated during ROS launch
        if self.LOG:
            innovation = self.ukf.innovation
            S = self.ukf.S
            nis_ahrs = float(innovation.T @ np.linalg.solve(S, innovation))
            self.nis_ahrs_log_data(t, nis_ahrs)

            self.get_logger().info(f"[DEBUG] innovation: \n {innovation}")
            self.get_logger().info(f"[DEBUG] S: \n {S}")
            self.get_logger().info(f"[DEBUG] NIS: \n {nis_ahrs}")

    def depth_callback(self, msg: PointStamped):
        pass

    # This callback will only work in if the "log" flag is set
    def benchmark_callback(self, msg: Odometry):
        if not self.LOG:
            return

        self.benchmark_log_data(msg)
    # Topic Handlers (STOP) --------------------------------------------------

    # Topic Helpers (START) --------------------------------------------------
    def publish_state_estimate(self, header):
        state = self.ukf.state
        P = self.ukf.P  # 9x9

        odom = Odometry()
        odom.header = header
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"

        # Position
        odom.pose.pose.position.x = state.p[0]
        odom.pose.pose.position.y = state.p[1]
        odom.pose.pose.position.z = state.p[2]

        # Orientation
        qx, qy, qz, qw = transform.Rotation.from_matrix(state.Rot).as_quat()
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Velocity (body-frame as model defines)
        odom.twist.twist.linear.x = state.v[0]
        odom.twist.twist.linear.y = state.v[1]
        odom.twist.twist.linear.z = state.v[2]

        # Pose covariance (6x6 row-major: x y z roll pitch yaw)
        # Since INERTIAL_NAVIGATION is defined in rot, vel, pose. 
        # That means Uncertainty has teh same form so: 
        # [0:3]  → rotation (SO(3) tangent)
        # [3:6]  → velocity
        # [6:9]  → position
        row_size = 6
        idx = row_size + 1
        cov_matrix_size = row_size * row_size

        # Build Pose Uncertainty matrix
        pose_cov = [0.0] * cov_matrix_size

        # position variance
        pose_cov[idx*0] = P[6,6]
        pose_cov[idx*1] = P[7,7]
        pose_cov[idx*2] = P[8,8]

        # orientation variance (SO3 tangent ≈ roll pitch yaw small-angle)
        pose_cov[idx*3] = P[0,0]
        pose_cov[idx*4] = P[1,1]
        pose_cov[idx*5] = P[2,2]

        odom.pose.covariance = pose_cov

        # Build Twist Uncertainty matrix
        twist_cov = [0.0] * cov_matrix_size

        # Twist covariance (6x6: vx vy vz wx wy wz)
        twist_cov[idx*0] = P[3,3]
        twist_cov[idx*1] = P[4,4]
        twist_cov[idx*2] = P[5,5]

        odom.twist.covariance = twist_cov

        # Publish all the data to ROS
        self.pub_odom.publish(odom)

        # This only runs if "log" flag was activated during ROS launch
        if self.LOG:
            self.state_estimate_log_data(odom)
    # Topic Helpers (START) --------------------------------------------------

    # Logger Helpers (START) --------------------------------------------------
    def state_estimate_log_data(self, msg: Odometry):
        # Time
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Position
        p = msg.pose.pose.position

        # Velocity
        v = msg.twist.twist.linear

        # Orientation (quaternion → roll, pitch, yaw)
        q = msg.pose.pose.orientation
        rpy = transform.Rotation.from_quat([
            q.x, q.y, q.z, q.w
        ]).as_euler('xyz', degrees=False)

        roll, pitch, yaw = rpy

        # Covariance (ROS pose covariance is 6x6 row-major)
        row_size = 6
        idx = row_size + 1
        pose_cov = msg.pose.covariance
        twist_cov = msg.twist.covariance

        data = {
            "t": t,

            "px": p.x,
            "py": p.y,
            "pz": p.z,

            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,

            "vx": v.x,
            "vy": v.y,
            "vz": v.z,

            "px_cov": pose_cov[idx*0],
            "py_cov": pose_cov[idx*1],
            "pz_cov": pose_cov[idx*2],

            "roll_cov": pose_cov[idx*3],
            "pitch_cov": pose_cov[idx*4],
            "yaw_cov": pose_cov[idx*5],

            "vx_cov": twist_cov[idx*0],
            "vy_cov": twist_cov[idx*1],
            "vz_cov": twist_cov[idx*2],
        }

        self.state_estimate_logger.log(data)

    def nis_ahrs_log_data(self, t: float, nis_ahrs: float):
        data = {
            "t": t,
            "nis_ahrs": nis_ahrs,
        }

        self.nis_ahrs_logger.log(data)

    def benchmark_log_data(self, msg: Odometry):
        # Time
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Position
        p = msg.pose.pose.position

        # Velocity
        v = msg.twist.twist.linear

        # Orientation (quaternion → roll, pitch, yaw)
        q = msg.pose.pose.orientation
        rpy = transform.Rotation.from_quat([
            q.x, q.y, q.z, q.w
        ]).as_euler('xyz', degrees=False)

        roll, pitch, yaw = rpy

        # Covariance (ROS pose covariance is 6x6 row-major)
        row_size = 6
        idx = row_size + 1
        pose_cov = msg.pose.covariance
        twist_cov = msg.twist.covariance

        data = {
            "t": t,

            "px": p.x,
            "py": p.y,
            "pz": p.z,

            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,

            "vx": v.x,
            "vy": v.y,
            "vz": v.z,

            "px_cov": pose_cov[idx*0],
            "py_cov": pose_cov[idx*1],
            "pz_cov": pose_cov[idx*2],

            "roll_cov": pose_cov[idx*3],
            "pitch_cov": pose_cov[idx*4],
            "yaw_cov": pose_cov[idx*5],

            "vx_cov": twist_cov[idx*0],
            "vy_cov": twist_cov[idx*1],
            "vz_cov": twist_cov[idx*2],
        }

        self.benchmark_logger.log(data)
    # Logger Helpers (STOP) --------------------------------------------------

    # Node Helpers (START) --------------------------------------------------
    def destroy_node(self):
        super().destroy_node()

        # If logging was enabled close all the logging processes so the data can be saved perfectly
        if self.LOG:
            self.benchmark_logger.close()
    # Node Helpers (STOP) --------------------------------------------------

    



def main(args=None):
    rclpy.init(args=args)
    node = StateEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()