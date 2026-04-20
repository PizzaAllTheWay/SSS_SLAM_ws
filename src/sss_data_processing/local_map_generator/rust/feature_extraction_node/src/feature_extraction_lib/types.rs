use std::collections::HashMap;
use opencv::core::Mat;
use nalgebra::Matrix2;



#[derive(Clone, Debug, Default)]
pub struct Position {
    // Position in meters (ENU or chosen navigation frame)
    pub x: f64,
    pub y: f64,

    // Vertical position in meters (depth or altitude depending on frame convention)
    // NOTE: define clearly in system (e.g. positive up or down)
    pub z: f64,
}

#[derive(Clone, Debug, Default)]
pub struct Orientation {
    // Orientation in radians
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

#[derive(Clone, Debug)]
pub struct Pose3D {
    pub position: Position,
    pub orientation: Orientation,
}
impl Default for Pose3D {
    fn default() -> Self {
        Self {
            position: Position::default(),
            orientation: Orientation::default(),
        }
    }
}
impl Pose3D {
    pub fn new() -> Self {
        Self::default()
    }
}

#[derive(Clone, Copy)]
pub struct Pose2DMap {
    // Position in pixels on the map
    pub x: i64, // [pixel]
    pub y: i64, // [pixel]

    // Yaw in [rad]
    pub yaw: f64, // [rad]
}

pub struct Map {
    pub pose: Pose2DMap,    // Position of drone/origin in Map [x, y, angle] 
    pub resolution: f64,    // in meters per pixel [m/pixel]
    pub width: usize,       // [pixel]
    pub height: usize,      // [pixel]
    pub data: Vec<Vec<u8>>, // M[y][x]
}
impl Map {
    pub fn new(
        pose: Pose2DMap,
        resolution: f64,
        width: usize,
        height: usize
    ) -> Self {
        Self {
            pose,
            resolution,
            width,
            height,
            data: vec![vec![0u8; width]; height],
        }
    }

    #[inline]
    pub fn set(&mut self, x: usize, y: usize, value: u8) {
        self.data[y][x] = value;
    }

    #[inline]
    pub fn get(&self, x: usize, y: usize) -> u8 {
        self.data[y][x]
    }
}

#[derive(Clone, Copy)]
pub enum MorphOp {
    Dilate(i32), // Grow regions
    Erode(i32),  // Shrink regions
    Open(i32),   // Remove small noise
    Close(i32),  // Fill small gaps
}

#[derive(Clone, Copy)]
pub enum LocalThresholdOp {
    Bright, // Pixel is brighter than local mean
    Shadow, // Pixel is darker than local mean
}

// Measurement from vehicle/map origin to landmark.
#[derive(Debug, Clone, Copy, Default)]
pub struct LandmarkMeasurement {
    pub r: f64,     // Range [m]
    pub theta: f64, // Bearing [rad]
}

// Strong descriptors are usually more distinctive and useful for matching.
#[derive(Debug, Clone, Default)]
pub struct LandmarkDescriptorsStrong {
    pub mean_intensity: f64,
    pub std: f64,
    pub contrast: f64,
    pub entropy: f64,
}

// Weak descriptors are simpler geometric/contextual properties.
#[derive(Debug, Clone, Default)]
pub struct LandmarkDescriptorsWeak {
    pub area: i32,
    pub polar_coordinates: LandmarkMeasurement,
    pub height: f64,
    pub radial_intensity_gradient: f64,
}

// Full descriptor bundle for one landmark.
#[derive(Debug, Clone, Default)]
pub struct LandmarkDescriptors {
    pub strong: LandmarkDescriptorsStrong,
    pub weak: LandmarkDescriptorsWeak,
}

#[derive(Debug, Clone, Default)]
pub struct Centroid {
    pub cx: f64, // Centroid x [pixel]
    pub cy: f64, // Centroid y [pixel]
}

// Bounding box around the landmark in image/map pixel coordinates.
#[derive(Debug, Clone, Default)]
pub struct BoundingBox {
    pub x: i32,      // Left [pixel]
    pub y: i32,      // Top [pixel]
    pub width: i32,  // Width [pixel]
    pub height: i32, // Height [pixel]

    // Binary mask for just this landmark inside the bounding box region.
    // Foreground = 255
    // background = 0
    pub mask: Mat,
}

// Full landmark object.
//
// Big picture:
// - `z` is the measured landmark position relative to current vehicle/map origin
// - `R_z` is the uncertainty of that measurement
// - `d` stores descriptors for matching/classification later
// - `centroid` stores the geometric center of the labeled landmark region in image coordinates.
// - `bounding_box` stores where the landmark lives in the segmented image
// - `estimated_height` stores a rough landmark height estimate derived later from shadow geometry
#[derive(Debug, Clone)]
#[allow(non_snake_case)]
pub struct Landmark {
    pub z: LandmarkMeasurement,
    pub R_z: Matrix2<f64>,
    pub d: LandmarkDescriptors,

    pub centroid: Centroid,
    pub bounding_box: BoundingBox,

    pub estimated_height: f64,
}
impl Default for Landmark {
    fn default() -> Self {
        Self {
            z: Default::default(),
            R_z: Matrix2::zeros(),
            d: Default::default(),
            centroid: Default::default(),
            bounding_box: Default::default(),
            estimated_height: 0.0,
        }
    }
}
impl Landmark {
    pub fn new() -> Self {
        Self::default()
    }
}

// Collection of landmarks indexed by label ID.
#[derive(Debug, Clone, Default)]
pub struct LandmarkSet {
    pub landmarks: HashMap<i32, Landmark>, // Landmarks[ID, Landmark]
}
