#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

from ukfm import UKF
from ukfm import SO3
from ukfm.model.inertial_navigation import INERTIAL_NAVIGATION as MODEL


# TODO: ! Will have to replace ther is a python package for this stuff
def quat_to_rot(q: Quaternion):
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
    ])
    return R

# TODO: ! Will have to replace ther is a python package for this stuff
def rot_to_quat(R):
    w = np.sqrt(1.0 + np.trace(R)) / 2.0
    x = (R[2, 1] - R[1, 2]) / (4.0 * w)
    y = (R[0, 2] - R[2, 0]) / (4.0 * w)
    z = (R[1, 0] - R[0, 1]) / (4.0 * w)
    return x, y, z, w



class StateEstimatorNode(Node):
    def __init__(self):
        super().__init__('state_estimator_node')

        self.sub_imu = self.create_subscription(Imu, '/hardware/imu', self.imu_callback, 1)

        self.pub_odom = self.create_publisher(Odometry,'/sss_slam/data_processing/state_estimate',10)

        # UKF model
        # TODO: imu_freq shoudl be a ros parameter
        self.model = MODEL(T=1, imu_freq=50)  # T dummy (not used online)

        # initial state
        # TODO: These shoudl be ROS parameters as well
        Rot0 = np.eye(3)
        v0 = np.zeros(3)
        p0 = np.zeros(3)

        state0 = self.model.STATE(Rot=Rot0, v=v0, p=p0)

        # TODO: This P_0 should also be a ROS parameter
        P0 = 1e-3 * np.eye(9)

        # TODO: These std should also be ROS parameters
        gyro_std = 0.05
        acc_std = 0.1

        Q = np.diag([
            gyro_std**2, gyro_std**2, gyro_std**2,
            acc_std**2,  acc_std**2,  acc_std**2
        ])

        # TODO: This Should also be a ROS parameter
        R_meas = 0.05**2 * np.eye(3)

        self.ukf = UKF(
            state0=state0,
            P0=P0,
            f=self.model.f,
            h=self.h_orientation,
            Q=Q,
            R=R_meas,
            phi=self.model.phi,
            phi_inv=self.model.phi_inv,
            alpha=np.ones(9) * 1e-3
        )

        self.last_time = None
    
    def h_orientation(state: MODEL.STATE):
        return state.Rot

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

        # Set up the input data structure that will propagate IMU data through the filter
        omega = self.model.INPUT(gyro=gyro, acc=acc)

        # Propagate IMU data
        self.ukf.propagation(omega, dt)

        # Format ROS AHRS data to be in same format as the filter wants
        R_meas = quat_to_rot(msg.orientation)

        # AHRS update (orientation)
        y = R_meas
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
        qx, qy, qz, qw = rot_to_quat(state.Rot)
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