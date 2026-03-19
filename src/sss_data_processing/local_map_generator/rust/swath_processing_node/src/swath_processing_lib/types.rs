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

pub struct SoundSpeed {
    // Speed of sound in water [m/s]
    // Used for sonar propagation and range correction
    pub value: f32,
}

pub struct SwathRaw {
    // Timestamp of ping transmission (important for sync)
    pub timestamp: f64,

    // Raw intensity values (port and starboard)
    pub port: Vec<u8>,
    pub starboard: Vec<u8>,

    // Number of samples per beam (defines resolution)
    pub samples_per_beam: u32,

    // Max slant range [m] used to derive resolution (slant_res = range / samples_per_beam)
    pub max_range: f32,
}

pub struct SwathProcessed {
    // Pose at ping time (interpolated)
    pub pose: Pose3D,

    // Height above seabed [m]
    pub altitude: AltitudeMeasurement,

    // Sampling frequency [Hz]
    pub sample_rate: f32,

    // Corrected intensity data (after filtering / normalization)
    pub port: Vec<f32>,
    pub starboard: Vec<f32>,
}