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

#[derive(Clone)]
pub struct AltitudeMeasurement {
    // Distance to seabed / surface in meters
    pub value: f64,
}

#[derive(Clone)]
pub struct SwathRaw {
    // Timestamp of ping transmission (important for sync)
    pub timestamp: f64,

    // Raw intensity values (port and starboard)
    pub port: Vec<u8>,
    pub starboard: Vec<u8>,

    // Number of samples per beam (defines resolution)
    pub samples_per_beam: u64,

    // Max slant range [m] used to derive resolution (slant_res = range / samples_per_beam)
    pub max_range: f64
}

pub struct TransducerParams {
    // Sonar mounting offsets relative to vehicle/body reference frame [m]
    pub offset: Pose3D,

    // Sonar beam geometry [rad]
    pub alpha: f64, // vertical beam width

    // Sample storage direction
    pub is_reversed: bool, // true if samples are stored far->near

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
