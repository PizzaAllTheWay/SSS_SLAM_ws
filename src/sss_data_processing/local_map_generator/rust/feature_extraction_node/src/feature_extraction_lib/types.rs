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

// TODO: Explain what thsi is for later
#[derive(Clone, Copy)]
pub enum MorphOp {
    Open(i32),
    Close(i32),
}

