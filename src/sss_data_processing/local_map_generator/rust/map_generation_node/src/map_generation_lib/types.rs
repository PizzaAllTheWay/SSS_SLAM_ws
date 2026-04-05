// ? NOTE: Use FxHashMap here instead of the default std HashMap because this code is
// ? performance-critical and does a very large number of hashmap operations on internal,
// ? non-hostile numeric keys. FxHashMap uses a faster non-cryptographic hasher, which
// ? reduces hashing overhead and improves runtime in hot paths like map/chunk/cell access.
use rustc_hash::FxHashMap;



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
    pub x: f64, // [m]
    pub y: f64, // [m]
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
    pub x_m: i64, // [pixel]
    pub y_m: i64, // [pixel]
}

pub struct CellMapM {
    // Cell data structure from the current measurement m combining everything into 1
    pub map_m: FxHashMap<CellCoordM, CellDataM>,
}
impl CellMapM {
    pub fn new() -> Self {
        Self {
            map_m: Default::default(),
        }
    }
}

pub struct CellData {
    // Data inside each cell inside the chunk
    pub v: f64, // Accumulated intensity value
    pub p: f64, // Accumulated probability value
}

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct CellCoord {
    // Discrete cell coordinate inside the map/grid
    pub x: i64, // [pixel]
    pub y: i64, // [pixel]
}

pub struct Chunk {
    // Chunk data structure combining everything into 1
    // Should be used in combination with a chunk manager to handle this data structure properly
    pub age: u32,
    pub data: FxHashMap<CellCoord, CellData>,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct ChunkCoord {
    // Coordinate of the chunk in chunk grid
    pub x: i64, // [pixel]
    pub y: i64, // [pixel]
}

pub struct ChunkMap {
    // Sparse storage of all active chunks
    // Key = chunk coordinate
    // Value = chunk data
    pub chunk_size: i64,
    pub chunk_max_age: u32,
    pub chunks: FxHashMap<ChunkCoord, Chunk>,
}
impl ChunkMap {
    pub fn new(
        chunk_size: i64,
        chunk_max_age: u32
    ) -> Self {
        Self {
            chunk_size,
            chunk_max_age,
            chunks: Default::default(),
        }
    }
}

pub struct Pose2DMap {
    // Position in pixels on the map
    pub x: i64, // [pixel]
    pub y: i64, // [pixel]

    // Yaw in [rad]
    pub yaw: f64, // [rad]
}

pub struct Map {
    pub pose: Pose2DMap,
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