#!/usr/bin/env python3
import numpy as np
import scipy.spatial.transform as transform
from pyproj import Transformer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped
from marine_acoustic_msgs.msg import Dvl
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

from ukfm.ukf.ukf import JUKF
from ukfm.model.inertial_navigation import INERTIAL_NAVIGATION as MODEL

from state_estimator.measurement_models import (
    ImuParams,
    DepthParams,
    DvlParams,
    GpsParams,
    SensorParams,
    MeasurementModels,
)

from state_estimator.utils import (
    BenchmarkLogger, 
    NISLogger, 
    StateEstimateLogger
)



class StateEstimatorNode(Node):
    # INITIALIZE (START) --------------------------------------------------
    def __init__(self):
        # Initialize node
        super().__init__('state_estimator_node')

        # Initialize ROS subscribers
        self.sub_imu   = self.create_subscription(Imu         , '/hardware/imu'  , self.imu_callback  , 1)
        self.sub_depth = self.create_subscription(PointStamped, '/hardware/depth', self.depth_callback, 1)
        self.sub_dvl   = self.create_subscription(Dvl         , '/hardware/dvl'  , self.dvl_callback  , 1)
        self.sub_gps   = self.create_subscription(NavSatFix   , '/hardware/gps'  , self.gps_callback  , 1)

        # Initialize ROS publishers
        self.pub_odom = self.create_publisher(Odometry,'/sss_slam/data_processing/state_estimate',10)

        # Initialize and get ROS parameters
        self.declare_parameter("log", False)
        self.declare_parameter("imu.orientation", [0.0, 0.0, 0.0])
        self.declare_parameter("imu.freq"       , 0)
        self.declare_parameter("imu.acc.std"    , 0.0)
        self.declare_parameter("imu.gyro.std"   , 0.0)
        self.declare_parameter("imu.ahrs.std"   , 0.0)
        self.declare_parameter("depth.position", [0.0, 0.0, 0.0])
        self.declare_parameter("depth.std"     , 0.0)
        self.declare_parameter("dvl.orientation", [0.0, 0.0, 0.0])
        self.declare_parameter("dvl.position"   , [0.0, 0.0, 0.0])
        self.declare_parameter("dvl.std"        , 0.0)
        self.declare_parameter("gps.frame_orientation", [0.0, 0.0, 0.0])
        self.declare_parameter("gps.position"         , [0.0, 0.0, 0.0])
        self.declare_parameter("ukfm.P_0"  , [0.0]*9)
        self.declare_parameter("ukfm.alpha", [0.0]*5)

        self.LOG = self.get_parameter("log").value # An extra flag you can set to enable loging of data for debugging and analysis of the filter
        imu_orientation = self.get_parameter("imu.orientation").get_parameter_value().double_array_value
        imu_freq        = self.get_parameter("imu.freq").get_parameter_value().integer_value
        imu_acc_std     = self.get_parameter("imu.acc.std").get_parameter_value().double_value
        imu_gyro_std    = self.get_parameter("imu.gyro.std").get_parameter_value().double_value
        imu_ahrs_std    = self.get_parameter("imu.ahrs.std").get_parameter_value().double_value
        depth_position = self.get_parameter("depth.position").get_parameter_value().double_array_value
        depth_std      = self.get_parameter("depth.std").get_parameter_value().double_value
        dvl_orientation = self.get_parameter("dvl.orientation").get_parameter_value().double_array_value
        dvl_position    = self.get_parameter("dvl.position").get_parameter_value().double_array_value
        dvl_std         = self.get_parameter("dvl.std").get_parameter_value().double_value
        gps_frame_orientation = self.get_parameter("gps.frame_orientation").get_parameter_value().double_array_value
        gps_position          = self.get_parameter("gps.position").get_parameter_value().double_array_value
        P_0   = self.get_parameter("ukfm.P_0").get_parameter_value().double_array_value
        alpha = self.get_parameter("ukfm.alpha").get_parameter_value().double_array_value
        
        self.get_logger().info(f"log: {self.LOG}")
        self.get_logger().info(f"imu.orientation: {imu_orientation}")
        self.get_logger().info(f"imu.freq:        {imu_freq}")
        self.get_logger().info(f"imu.acc.std:     {imu_acc_std}")
        self.get_logger().info(f"imu.gyro.std:    {imu_gyro_std}")
        self.get_logger().info(f"imu.ahrs.std:    {imu_ahrs_std}")
        self.get_logger().info(f"depth.position: {depth_position}")
        self.get_logger().info(f"depth.std:      {depth_std}")
        self.get_logger().info(f"dvl.orientation: {dvl_orientation}")
        self.get_logger().info(f"dvl.position:    {dvl_position}")
        self.get_logger().info(f"dvl.std:         {dvl_std}")
        self.get_logger().info(f"gps.frame_orientation: {gps_frame_orientation}")
        self.get_logger().info(f"gps.position:          {gps_position}")
        self.get_logger().info(f"ukfm.P_0:   {P_0}")
        self.get_logger().info(f"ukfm.alpha: {alpha}")

        # Initialize location of sensors
        R_body_to_imu = transform.Rotation.from_euler('xyz', imu_orientation).as_matrix()
        self.get_logger().info(f"R_imu_to_ned:\n{np.array2string(R_body_to_imu, precision=3, suppress_small=True)}")

        r_body_to_depth = np.array(depth_position, dtype=float)
        self.get_logger().info(f"r_body_to_depth:\n{np.array2string(r_body_to_depth, precision=3, suppress_small=True)}")

        R_body_to_dvl = transform.Rotation.from_euler('xyz', dvl_orientation).as_matrix()
        self.get_logger().info(f"R_body_to_dvl:\n{np.array2string(R_body_to_dvl, precision=3, suppress_small=True)}")
        r_body_to_dvl = np.array(dvl_position, dtype=float)
        self.get_logger().info(f"r_body_to_dvl:\n{np.array2string(r_body_to_dvl, precision=3, suppress_small=True)}")

        R_gps_fix_orientation = transform.Rotation.from_euler('xyz', gps_frame_orientation).as_matrix()
        self.get_logger().info(f"R_gps_fix_orientation:\n{np.array2string(R_gps_fix_orientation, precision=3, suppress_small=True)}")
        r_body_to_gps = np.array(gps_position, dtype=float)
        self.get_logger().info(f"r_body_to_gps:\n{np.array2string(r_body_to_gps, precision=3, suppress_small=True)}")

        # UKF model
        self.model = MODEL(T=1, imu_freq=imu_freq)  # T dummy (not used online)

        # Initialize state
        Rot0 = np.eye(3)
        v0 = np.zeros(3)
        p0 = np.zeros(3)

        state0 = self.model.STATE(Rot=Rot0, v=v0, p=p0)

        P_0_matrix = np.diag(P_0)
        
        # This is used as model input to propagate state through time
        self.omega = self.model.INPUT(
            gyro=np.zeros(3),
            acc=np.zeros(3)
        ) 

        # Initialize process noise
        Q = np.diag([
            imu_gyro_std**2, imu_gyro_std**2, imu_gyro_std**2,
            imu_acc_std**2 , imu_acc_std**2 , imu_acc_std**2
        ])

        # Initialize aiding measurement models
        sensor_params = SensorParams(
            imu=ImuParams(
                ahrs_std=imu_ahrs_std,
                orientation=R_body_to_imu
            ),
            depth=DepthParams(
                std=depth_std,
                position=r_body_to_depth
            ),
            dvl=DvlParams(
                std=dvl_std,
                orientation=R_body_to_dvl,
                position=r_body_to_dvl
            ),
            gps=GpsParams(
                orientation_fix=R_gps_fix_orientation,
                position=r_body_to_gps
            )
        )

        self.measurement_models = MeasurementModels(
            params=sensor_params
        )
        
        # Initialize filter
        self.ukf = JUKF(
            f=self.model.f,
            h=self.measurement_models.h_ahrs,  # default measurement model (can be changed before each update)
            phi=self.model.phi,
            Q=Q,
            alpha=alpha,
            state0=state0,
            P0=P_0_matrix,
            red_phi=self.model.phi,
            red_phi_inv=self.model.phi_inv,
            red_idxs=np.arange(9), # indices of state used during propagation (all 9 states propagate)
            up_phi=self.model.phi,
            up_idxs=np.arange(9) # indices of state allowed to be corrected during update (all 9 states updated)
        )

        # For tracking time so filter can propagate through time accurately
        self.last_time = None

        # For GNSS reference setup
        self.gps_ref_set = False

        # This part is purely for logging and debugging
        # If Logging flag is enabled we add a logger for post processing purposes
        # Then we can analyze data later down the line to debug state estimator
        if self.LOG:
            self.state_estimate_logger = StateEstimateLogger()

            self.nis_ahrs_logger  = NISLogger(datafile_name="ahrs")
            self.nis_depth_logger = NISLogger(datafile_name="depth")
            self.nis_dvl_logger   = NISLogger(datafile_name="dvl")
            self.nis_gps_logger   = NISLogger(datafile_name="gps")

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
        
        # Check for time corruption to ensure that doesn't happen
        if t <= self.last_time:
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

        # Set up the input data structure that will propagate IMU data through the filter
        self.omega = self.model.INPUT(gyro=gyro, acc=acc)

        # Propagate IMU data
        self.ukf.propagation(self.omega, dt)

        # Format ROS AHRS data to be in same format as the filter wants
        # Important to translate form IMU to NED frame before passing down the values because filter lives in NED and IMU lives in ENU
        R_meas = transform.Rotation.from_quat([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]).as_matrix()

        # AHRS update (orientation)
        y = transform.Rotation.from_matrix(R_meas).as_rotvec()
        h = self.measurement_models.h_ahrs
        R = self.measurement_models.R_ahrs
        self.ukf.h = h
        self.ukf.update(y, R)

        # Publish state estimate
        self.publish_state_estimate(msg.header)

        # This only runs if "log" flag was activated during ROS launch
        if self.LOG:
            innovation = self.ukf.innovation
            S = self.ukf.S
            nis_ahrs = float(innovation.T @ np.linalg.solve(S, innovation))
            self.nis_ahrs_log_data(t, nis_ahrs)

    def depth_callback(self, msg: PointStamped):
        # Convert ROS time data structure to readable time in precise seconds
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Usually IMU will get the first time stamp but just in case this is here to prevent edge cases and ensures we gave a time stamp
        if self.last_time is None:
            self.last_time = t
            return
        
        # Check for time corruption to ensure that doesn't happen
        if t <= self.last_time:
            return

        # Get the time difference between last filter update and now
        dt = t - self.last_time
        self.last_time = t

        # Propagate state between aiding measurement time and latest IMU data
        self.ukf.propagation(self.omega, dt)

        # Format ROS depth data to be in same format as the filter wants
        depth_meas = np.array([msg.point.z])

        # Depth update
        y = depth_meas
        h = self.measurement_models.h_depth
        R = self.measurement_models.R_depth
        self.ukf.h = h
        self.ukf.update(y, R)

        # Publish state estimate
        self.publish_state_estimate(msg.header)

        # This only runs if "log" flag was activated during ROS launch
        if self.LOG:
            innovation = self.ukf.innovation
            S = self.ukf.S
            nis_depth = float(innovation.T @ np.linalg.solve(S, innovation))
            self.nis_depth_log_data(t, nis_depth)

    def dvl_callback(self, msg: Dvl):
        # Before anything we must ensure DVL data is correct and what we want as it sends out different data at different times
        # Only use bottom track velocity (ignore water track or invalid modes)
        # In addition ensure beams are valid
        if msg.velocity_mode != Dvl.DVL_MODE_BOTTOM:
            return
        if not msg.beam_velocities_valid:
            return
    
        # Convert ROS time data structure to readable time in precise seconds
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Usually IMU will get the first time stamp but just in case this is here to prevent edge cases and ensures we gave a time stamp
        if self.last_time is None:
            self.last_time = t
            return
        
        # Check for time corruption to ensure that doesn't happen
        if t <= self.last_time:
            return

        # Get the time difference between last filter update and now
        dt = t - self.last_time
        self.last_time = t

        # Propagate state between aiding measurement time and latest IMU data
        self.ukf.propagation(self.omega, dt)

        # Extract measured velocity (already in DVL frame)
        vel_meas = np.array([
            msg.velocity.x,
            msg.velocity.y
        ])

        # DVL update
        y = vel_meas
        h = self.measurement_models.h_dvl
        R = self.measurement_models.R_dvl
        self.ukf.h = h
        self.ukf.update(y, R)

        # Publish estimate
        self.publish_state_estimate(msg.header)

        # This only runs if "log" flag was activated during ROS launch
        if self.LOG:
            innovation = self.ukf.innovation
            S = self.ukf.S
            nis_dvl = float(innovation.T @ np.linalg.solve(S, innovation))
            self.nis_dvl_log_data(t, nis_dvl)
    
    def gps_callback(self, msg: NavSatFix):
        # Double check that the message is valid
        if msg.status.status < 0:
            return
    
        # Convert ROS time data structure to readable time in precise seconds
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Usually IMU will get the first time stamp but just in case this is here to prevent edge cases and ensures we gave a time stamp
        if self.last_time is None:
            self.last_time = t
            return
        
        # Check for time corruption to ensure that doesn't happen
        if t <= self.last_time:
            return

        # Get the time difference between last filter update and now
        dt = t - self.last_time
        self.last_time = t

        # Propagate state between aiding measurement time and latest IMU data
        self.ukf.propagation(self.omega, dt)

        # Extract measured position
        # Convert from WGS84 -> ECF -> ENU format
        lat = msg.latitude
        lon = msg.longitude
        h   = msg.altitude

        # Create once at first fix
        if not self.gps_ref_set:
            self.transformer = Transformer.from_crs("EPSG:4326", "EPSG:4978", always_xy=True)
            self.east0, self.north0, self.up0 = self.transformer.transform(lon, lat, h)
            self.gps_ref_set = True

        # WGS84 → ENU (topocentric gives ENU directly)
        east, north, up = self.transformer.transform(lon, lat, h)
        east  -= self.east0
        north -= self.north0
        up    -= self.up0
        p_enu = np.array([north, east, up])
        p_xy = p_enu[0:2]

        # Covariance reshape
        cov = np.array(msg.position_covariance).reshape(3, 3)
        R_xy = cov[0:2, 0:2]

        # GPS update
        y = p_xy
        h = self.measurement_models.h_gps
        R = R_xy
        self.ukf.h = h
        self.ukf.update(y, R)

        # Publish estimate
        self.publish_state_estimate(msg.header)

        # This only runs if "log" flag was activated during ROS launch
        if self.LOG:
            innovation = self.ukf.innovation
            S = self.ukf.S
            nis_gps = float(innovation.T @ np.linalg.solve(S, innovation))
            self.nis_gps_log_data(t, nis_gps)

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

    def nis_ahrs_log_data(self, t: float, nis: float):
        data = {
            "t": t,
            "nis": nis,
        }

        self.nis_ahrs_logger.log(data)

    def nis_depth_log_data(self, t: float, nis: float):
        data = {
            "t": t,
            "nis": nis,
        }

        self.nis_depth_logger.log(data)  

    def nis_dvl_log_data(self, t: float, nis: float):
        data = {
            "t": t,
            "nis": nis,
        }

        self.nis_dvl_logger.log(data)   

    def nis_gps_log_data(self, t: float, nis: float):
        data = {
            "t": t,
            "nis": nis,
        }

        self.nis_gps_logger.log(data) 

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