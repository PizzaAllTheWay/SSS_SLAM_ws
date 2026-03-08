#!/usr/bin/env python3
import numpy as np
import scipy.spatial.transform as transform
from dataclasses import dataclass

from ukfm.model.inertial_navigation import INERTIAL_NAVIGATION as MODEL



@dataclass
class ImuParams:
    ahrs_std: float
    orientation: np.ndarray   # R_body_to_imu (3x3)

@dataclass
class DepthParams:
    std: float
    position: np.ndarray      # r_body_to_depth (3)

@dataclass
class DvlParams:
    std: float
    orientation: np.ndarray   # R_body_to_dvl (2x2)
    position: np.ndarray      # r_body_to_dvl (3)

@dataclass
class GpsParams:
    orientation_fix: np.ndarray   # R_body_to_gps_frame (2x2)
    position: np.ndarray          # r_body_to_gps (3)

@dataclass
class SensorParams:
    imu: ImuParams
    depth: DepthParams
    dvl: DvlParams
    gps: GpsParams



class MeasurementModels:
    def __init__(self, params: SensorParams):
        # Measurement noise covariance matrices
        # Built from sensor standard deviations provided in the configuration.
        self.R_ahrs  = params.imu.ahrs_std**2 * np.eye(3)
        self.R_depth = params.depth.std**2 * np.eye(1)
        self.R_dvl   = params.dvl.std**2 * np.eye(2)

        # Fixed rotations between vehicle body frame and sensor frames
        self.R_body_to_imu = params.imu.orientation
        self.R_body_to_dvl = params.dvl.orientation
        self.R_gps_fix     = params.gps.orientation_fix

        # Lever-arm offsets between vehicle body origin and sensors
        self.r_body_to_depth = params.depth.position
        self.r_body_to_gps   = params.gps.position

    # Aiding Measurement State Transforms (START) --------------------------------------------------
    def h_ahrs(self, state: MODEL.STATE):
        # AHRS provides orientation of the IMU frame.
        # The filter state stores orientation of the body frame in ENU.
        # Therefore we transform body → IMU using the known mounting rotation
        # and convert the resulting rotation matrix to a rotation vector.
        orientation_imu_est = state.Rot @ self.R_body_to_imu
        z_est = transform.Rotation.from_matrix(orientation_imu_est).as_rotvec()
        return z_est
    
    def h_depth(self, state: MODEL.STATE):
        # Depth sensor measures vertical position of the sensor head.
        # The filter state stores the vehicle body position, therefore
        # we add the lever-arm offset of the sensor in the body frame.
        depth_state = np.array([state.p[2]])
        z_est = depth_state + self.r_body_to_depth[2]
        return z_est

    def h_dvl(self, state: MODEL.STATE):
        # DVL measures velocity in the sensor frame.
        # Here we rotate body-frame velocity into the DVL frame and
        # only use the horizontal velocity components (vx, vy).
        #
        # Ideally the rigid-body velocity relation would also include
        # angular velocity compensation:
        #   v_sensor = v_body + ω × r
        # However angular velocity is not part of the filter state,
        # therefore only the linear velocity is used here.
        v_body = state.v
        z_est = self.R_body_to_dvl @ v_body
        z_est = v_body[0:2]
        return z_est

    def h_gps(self, state: MODEL.STATE):
        # GPS measures the position of the antenna.
        # The filter state stores the body position, therefore we apply
        # the GPS lever-arm offset and then keep only the horizontal
        # position components (x, y).
        p_body = state.p
        R_body = state.Rot
        z_est = p_body + R_body @ self.r_body_to_gps
        z_est = self.R_gps_fix @ z_est
        z_est = z_est[0:2]
        return z_est
    # Aiding Measurement State Transforms (STOP) --------------------------------------------------
