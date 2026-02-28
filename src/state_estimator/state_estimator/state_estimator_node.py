#!/usr/bin/env python3
import numpy as np
import scipy.spatial.transform as transform

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from ukfm.ukf.ukf import UKF
from ukfm.model.inertial_navigation import INERTIAL_NAVIGATION as MODEL



class StateEstimatorNode(Node):
    def __init__(self):
        # Initialize node
        super().__init__('state_estimator_node')

        # Initialize ROS subscribers
        self.sub_imu = self.create_subscription(Imu, '/hardware/imu', self.imu_callback, 1)

        # Initialize ROS publishers
        self.pub_odom = self.create_publisher(Odometry,'/sss_slam/data_processing/state_estimate',10)

        # Initialize and get ROS parameters
        self.declare_parameter("imu.orientation", [0.0, 0.0, 0.0])
        self.declare_parameter("imu.freq"       , 0)
        self.declare_parameter("imu.acc.std"    , 0.0)
        self.declare_parameter("imu.gyro.std"   , 0.0)
        self.declare_parameter("imu.ahrs.std"   , 0.0)
        self.declare_parameter("ukfm.P_0", [0.0]*9)

        imu_orientation = self.get_parameter("imu.orientation").get_parameter_value().double_array_value
        imu_freq        = self.get_parameter("imu.freq").get_parameter_value().integer_value
        imu_acc_std     = self.get_parameter("imu.acc.std").get_parameter_value().double_value
        imu_gyro_std    = self.get_parameter("imu.gyro.std").get_parameter_value().double_value
        imu_ahrs_std    = self.get_parameter("imu.ahrs.std").get_parameter_value().double_value
        P_0 = self.get_parameter("ukfm.P_0").get_parameter_value().double_array_value

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

        # initial state
        Rot0 = np.eye(3)
        v0 = np.zeros(3)
        p0 = np.zeros(3)

        state0 = self.model.STATE(Rot=Rot0, v=v0, p=p0)

        P_0_matrix = np.diag(P_0)

        Q = np.diag([
            imu_gyro_std**2, imu_gyro_std**2, imu_gyro_std**2,
            imu_acc_std**2 , imu_acc_std**2 , imu_acc_std**2
        ])

        R_ahrs = imu_ahrs_std**2 * np.eye(3)

        self.ukf = UKF(
            state0=state0,
            P0=P_0_matrix,
            f=self.model.f,
            h=self.h_orientation,
            Q=Q,
            R=R_ahrs,
            phi=self.model.phi,
            phi_inv=self.model.phi_inv,
            alpha=np.ones(9) * 1e-3
        )

        self.last_time = None
    
    def h_orientation(self, state: MODEL.STATE):
        return transform.Rotation.from_matrix(state.Rot).as_rotvec()

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
        self.publish_state(msg.header)

    def publish_state(self, header):
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



def main(args=None):
    rclpy.init(args=args)
    node = StateEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()