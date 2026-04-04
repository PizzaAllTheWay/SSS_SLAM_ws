use std::collections::HashMap;



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

pub struct GeometricCorrection {
    // Corrected vertical altitude from each transducer to seabed [m]
    pub h_port: f64,
    pub h_stb: f64,

    // First-bottom-return slant range for each channel [m]
    pub r_fbr_port: f64,
    pub r_fbr_stb: f64,
}

pub struct SwathProcessed {
    // Timestamp of swath processed (important for sync)
    pub timestamp: f64,

    // Corrected intensity data (after filtering / normalization)
    pub port: Vec<u8>,
    pub starboard: Vec<u8>,

    // Number of samples per beam (defines resolution)
    pub samples_per_beam: u64,
}

pub struct TransducerParams {
    // Sonar mounting offsets relative to vehicle/body reference frame [m]
    pub offset: Pose3D,

    // Max slant range [m]
    pub max_range: f64,

    // Sonar beam geometry [rad]
    pub theta: f64, // horizontal beam width

    // Sample storage direction
    pub is_reversed: bool, // true if samples are stored far->near

    // Blind zone compensation factor
    pub blind_zone_scale: f64, // empirically tuned from test data
}

pub struct SonarParams {
    pub transducer_port: TransducerParams,
    pub transducer_stb: TransducerParams,
}

pub struct QPixel {
    // Pixel position in world/map space
    // Should be equal discrete steps between pixels
    pub x: f64,
    pub y: f64,
}

pub struct CellDataM {
    // Data for one map-cell candidate from the current measurement m
    pub q_m: QPixel, // Pixel itself coordinate
    pub v_m: f64,    // intensity value
    pub p_m: f64,    // probability value
}

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct CellCoordM {
    // Discrete cell coordinate inside the map/grid from the current measurement m
    pub x_m: i64,
    pub y_m: i64,
}

pub struct CellMapM {
    // Cell data structure from the current measurement m combining everything into 1
    pub map_m: HashMap<CellCoordM, CellDataM>,
}
impl CellMapM {
    pub fn new() -> Self {
        Self {
            map_m: HashMap::new(),
        }
    }
}

pub struct CellData {
    // Data inside each cell inside the chunk
    pub q: QPixel, // Pixel itself coordinate
    pub v: f64,    // Accumulated intensity value
    pub p: f64,    // Accumulated probability value
}

pub struct CellCoord {
    // Discrete cell coordinate inside the map/grid
    pub x: i64,
    pub y: i64,
}

pub struct Chunk {
    // Chunk data structure combining everything into 1
    // Should be used in combination with a chunk manager to handle this data structure properly
    pub age: u32,
    pub data: HashMap<CellCoord, CellData>,
}

pub struct ChunkCoord {
    // Coordinate of the chunk in chunk grid
    pub x: i64,
    pub y: i64,
}

pub struct ChunkMap {
    // Sparse storage of all active chunks
    // Key = chunk coordinate
    // Value = chunk data
    pub chunks: HashMap<ChunkCoord, Chunk>,
}
impl ChunkMap {
    pub fn new() -> Self {
        Self {
            chunks: HashMap::new(),
        }
    }
}