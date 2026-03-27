#[derive(Clone)]
pub struct Position {
    // Position in meters (ENU or chosen navigation frame)
    pub x: f64,
    pub y: f64,

    // Vertical position in meters (depth or altitude depending on frame convention)
    // NOTE: define clearly in system (e.g. positive up or down)
    pub z: f64,
}

#[derive(Clone)]
pub struct Orientation {
    // Orientation in radians
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

#[derive(Clone)]
pub struct Pose3D {
    pub position: Position,
    pub orientation: Orientation,
}

pub struct TransducerParams {
    // Blind zone compensation factor
    pub blind_zone_scale: f64, // empirically tuned from test data
}

pub struct SonarParams {
    pub transducer_port: TransducerParams,
    pub transducer_stb: TransducerParams,
}

pub struct GeometricCorrection {
    // Corrected vertical altitude from each transducer to seabed [m]
    pub h_port: f64,
    pub h_stb: f64,

    // First-bottom-return slant range for each channel [m]
    pub r_fbr_port: f64,
    pub r_fbr_stb: f64,
}

pub struct SwathProcessed {
    // Pose at ping time (interpolated)
    pub pose_interpolated: Pose3D,

    // Height above seabed [m]
    pub geometric_correction_data: GeometricCorrection,

    // Corrected intensity data (after filtering / normalization)
    pub port: Vec<u8>,
    pub starboard: Vec<u8>,
}
