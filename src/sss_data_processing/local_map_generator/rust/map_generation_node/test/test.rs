// For simple test run:
// $ cargo run --bin test
// For profiling run:
// $ cargo flamegraph --release --bin test



use std::f64::consts::PI;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::time::Instant;

use map_generation_node::map_generation_lib::types::*;
use map_generation_node::map_generation_lib::generator::MapGenerator;



// LOGGER
use std::{
    fs::create_dir_all,
    time::{SystemTime, UNIX_EPOCH},
};
use serde::Serialize;

// ---------- BASE (Skeleton for all loggers) ----------
pub struct Logger {
    writer: csv::Writer<File>,
}

impl Logger {
    pub fn new(name: &str, header: &[&str]) -> Self {
        let dir = "test/logs/data";
        create_dir_all(dir).unwrap();

        let ts = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs();
        let path = format!("{}/{}_{}.csv", dir, name, ts);

        let file = File::create(path).unwrap();
        let mut writer = csv::Writer::from_writer(file);
        writer.write_record(header).unwrap();

        Self { writer }
    }

    pub fn log<T: Serialize>(&mut self, row: T) {
        self.writer.serialize(row).unwrap();
    }
}

// ---------- CellMapM ----------
pub struct LoggerCellMapM {
    logger: Logger,
}

impl LoggerCellMapM {
    pub fn new() -> Self {
        Self {
            logger: Logger::new("cell_map_m", &["t", "cell_map_m"]),
        }
    }

    pub fn log(&mut self, t: f64, cell_map_m: &CellMapM) {
        let cell_str = cell_map_m
            .map_m
            .values()
            .map(|cell| format!(
                "{} {} {} {}",
                cell.q_m.x,
                cell.q_m.y,
                cell.p_m,
                cell.v_m
            ))
            .collect::<Vec<_>>()
            .join(" | ");

        self.logger.writer.write_record(&[
            t.to_string(),
            cell_str,
        ]).unwrap();
    }
}

// ---------- ChunkMap ----------
pub struct LoggerChunkMap {
    logger: Logger,
}

impl LoggerChunkMap {
    pub fn new() -> Self {
        Self {
            logger: Logger::new("chunk_map", &["t", "chunk_map"]),
        }
    }

    pub fn log(&mut self, t: f64, chunk_map: &ChunkMap) {
        let chunk_str = chunk_map
            .chunks
            .iter()
            .flat_map(|(chunk_coord, chunk)| {
                chunk.data.iter().map(move |(cell_coord, cell)| {
                    format!(
                        "{} {} {} {} {} {} {}",
                        chunk_coord.x,
                        chunk_coord.y,
                        chunk.age,
                        cell_coord.x,
                        cell_coord.y,
                        cell.p,
                        cell.v,
                    )
                })
            })
            .collect::<Vec<_>>()
            .join(" | ");

        self.logger.writer.write_record(&[
            t.to_string(),
            chunk_str,
        ]).unwrap();
    }
}

// ---------- Map ----------
pub struct LoggerMap {
    logger: Logger,
}

impl LoggerMap {
    pub fn new() -> Self {
        Self {
            logger: Logger::new(
                "map",
                &["t", "pose_x", "pose_y", "pose_yaw", "resolution", "width", "height", "map"],
            ),
        }
    }

    pub fn log(&mut self, t: f64, map: &Map) {
        let map_str = map
            .data
            .iter()
            .flat_map(|row| row.iter().map(|v| v.to_string()))
            .collect::<Vec<_>>()
            .join(" ");

        self.logger.writer.write_record(&[
            t.to_string(),
            map.pose.x.to_string(),
            map.pose.y.to_string(),
            map.pose.yaw.to_string(),
            map.resolution.to_string(),
            map.width.to_string(),
            map.height.to_string(),
            map_str,
        ]).unwrap();
    }
}



fn parse_vec_u8(s: &str) -> Vec<u8> {
    s.split_whitespace().map(|v| v.parse().unwrap()).collect()
}



fn main() {
    let start_n = 3_500;
    //let start_n = 1;

    //let n = 10_000;
    //let n = 5_000;
    //let n = 2_000;
    //let n = 1_000;
    let n = 500;
    //let n = 100;

    let pose_interpolated_file = BufReader::new(File::open("test/data/pose_interpolated.csv").unwrap());
    let geometric_correction_file  = BufReader::new(File::open("test/data/geometric_correction.csv").unwrap());
    let swath_processed_file= BufReader::new(File::open("test/data/swath_processed.csv").unwrap());

    let mut pose_interpolated_iter = pose_interpolated_file.lines().skip(start_n);
    let mut geometric_correction_iter  = geometric_correction_file.lines().skip(start_n);
    let mut swath_processed_iter= swath_processed_file.lines().skip(start_n);

    let sonar = SonarParams {
        transducer_port: TransducerParams {
            offset: Pose3D {
                position: Position {
                    x: -0.2532,
                    y:  0.082, // changed sign for port side
                    z:  0.033,
                },
                orientation: Orientation {
                    roll: 25.0_f64.to_radians(), // same for both sides
                    pitch: 0.0,
                    yaw: 180.0_f64.to_radians(),
                },
            },
            max_range: 30.0,
            theta: 25.0_f64.to_radians(), // horizontal beam width
            is_reversed: true, // port is stored 1000 -> 0
            blind_zone_scale: 0.45, // empirically tuned from test data
        },
        transducer_stb: TransducerParams {
            offset: Pose3D {
                position: Position {
                    x: -0.2532,
                    y: -0.082, // changed sign for starboard side
                    z:  0.033,
                },
                orientation: Orientation {
                    roll: 25.0_f64.to_radians(), // same for both sides
                    pitch: 0.0,
                    yaw: 0.0,
                },
            },
            max_range: 30.0,
            theta: 25.0_f64.to_radians(), // horizontal beam width
            is_reversed: false, // starboard is stored 0 -> 1000
            blind_zone_scale: 0.35, // empirically tuned from test data
        },
    };

    // Dense map update triggers.
    //
    // The dense map is rebuilt either:
    // - after `MAP_UPDATE_EVERY_N_SWATHS` buffered swaths, or
    // - if the time gap between consecutive swaths exceeds `MAP_UPDATE_MAX_TIME_GAP`
    //
    // This keeps dense map generation from running too often during high-rate data,
    // while still forcing an update if swaths arrive too sparsely.
    const MAP_UPDATE_EVERY_N_SWATHS: usize = 100; // [swath samples]
    const MAP_UPDATE_MAX_TIME_GAP: f64 = 5.0; // [s]

    // Dense map resolution in meters per pixel [m/pixel]
    //
    // This is chosen to match the sonar slant-range sample spacing:
    //     map_resolution = max_range/samples_per_beam = 30.0/1000 = 0.03 [m/pixel]
    //
    // Using the sonar sample spacing is a practical default because it keeps the map
    // resolution consistent with the raw measurement resolution without creating
    // unnecessary extra pixels.
    //
    // A smaller map resolution than the sonar resolution is possible, but usually not useful:
    // it increases the number of pixels and compute cost while not adding real new information,
    // since the sonar did not measure that finely.
    //
    // A larger map resolution is also possible and may be useful when compute is limited.
    // That reduces the number of pixels that must be processed and stored, so runtime and memory
    // usage improve, but some fine sonar detail will be lost because multiple measurements are
    // merged into a coarser map representation.
    let map_resolution = 0.03;

    // Chunk size in pixels per direction for the sparse chunked map.
    //
    // Each chunk stores a square local patch of the global map.
    // For example:
    //     chunk_size = 64
    // means each chunk represents a 64 x 64 pixel region.
    //
    // This parameter controls the tradeoff between chunk-management overhead and per-chunk work/memory cost.
    //
    // Larger chunk size:
    // - fewer chunks are needed for the same total map area
    // - less hashmap overhead, since fewer chunk entries must be created and tracked
    // - fewer chunk-boundary crossings during map updates
    // - but each chunk covers a larger area, so local operations inside one chunk become heavier
    // - and more empty/unused cells may be carried inside an active chunk footprint
    //
    // Smaller chunk size:
    // - more chunks are needed for the same total map area
    // - more hashmap/chunk-management overhead
    // - but active map storage becomes more spatially selective, which is useful if only small local regions are occupied at a time and per-chunk local scans / temporary masks become cheaper
    //
    // In practice, this should be tuned based on the expected map sparsity and runtime constraints.
    // A medium value like 64 is a reasonable starting point because it keeps the number of chunks
    // manageable while still keeping per-chunk local processing relatively cheap.
    let chunk_size = 64;

    // Maximum chunk age before removal.
    //
    // Age counts how many full map-update cycles have passed since the chunk
    // was last touched by new data.
    //
    // Higher value:
    // - keeps old map regions longer
    // - but increases active map size and slows processing
    //
    // Lower value:
    // - keeps the active map smaller and faster
    // - but removes old regions sooner
    let chunk_max_age = 1;

    // Pruning thresholds.
    //
    // `beam_weight_threshold` is the fast first-stage pruning threshold based on the
    // cheap beam approximation. Its value is in the range [0, 1].
    // Higher value -> more aggressive early rejection, fewer samples kept, faster runtime
    // Lower value  -> more samples survive to the later pruning stage
    //
    // `probabilistic_map_threshold` is the second-stage pruning threshold based on the
    // more exact probabilistic beam model. This value is also nonnegative, but in practice
    // it is usually kept very small because the integrated probabilities are small.
    // Higher value -> more aggressive pruning, fewer samples kept
    // Lower value  -> more samples preserved, more detail kept, higher compute cost
    //
    // ? NOTE:
    // ? Small changes in either of these thresholds can cause large changes in how many
    // ? samples survive pruning, so even small tuning adjustments may have a big effect
    // ? on both the final map and the runtime.
    let beam_weight_threshold = 0.9993;
    let probabilistic_map_threshold = 0.002000;

    // Local gap-fill tuning.
    //
    // `fill_inn_min_neighbors` sets how many valid neighbors are required
    // before an empty pixel is filled.
    // Higher value -> stricter fill
    // Lower value  -> more aggressive fill
    // Values above 8 can never succeed, since only the 8 surrounding pixels are checked.
    //
    // `fill_inn_passes` sets how many fill rounds are allowed.
    // More passes fill deeper connected gaps, but increase runtime.
    // Around 1 to 10 passes is usually enough; 5 worked well here.
    let fill_inn_min_neighbors = 3;
    let fill_inn_passes = 5;

    // ? NOTE:
    // ? Temporary yaw alignment correction used during map generation.
    // ? The current pose yaw convention and the map/sonar ground-plane convention
    // ? are offset by 90 degrees, so this fixed correction is applied to align them.
    // ? This is only a workaround for the current frame mismatch and should ideally
    // ? be removed once the underlying frame definitions are made fully consistent.
    let map_offset_yaw = -PI/2.0;

    let mut map_generator = MapGenerator::new(
        sonar,
        map_resolution,
        chunk_size,
        chunk_max_age,
        beam_weight_threshold,
        probabilistic_map_threshold,
        fill_inn_min_neighbors,
        fill_inn_passes,
        map_offset_yaw,
    );

    // init logger before loop
    let mut logger_cell_map_m = LoggerCellMapM::new();
    let mut logger_chunk_map = LoggerChunkMap::new();
    let mut logger_map = LoggerMap::new();

    // Mapping logic
    let mut prev_t: Option<f64> = None;
    let mut ping_counter: usize = 0;

    // Timing for performance analytics buffers
    let mut total_buffer_processed_swath_into_map_s = 0.0;
    let mut total_calculate_map_s = 0.0;

    let mut count_buffer_processed_swath_into_map: usize = 0;
    let mut count_calculate_map: usize = 0;

    for _ in 0..n {
        let pose_interpolated_line = pose_interpolated_iter.next().unwrap().unwrap();
        let geometric_correction_line  = geometric_correction_iter.next().unwrap().unwrap();
        let swath_processed_line= swath_processed_iter.next().unwrap().unwrap();

        let p: Vec<f64> = pose_interpolated_line.split(',').map(|v| v.parse().unwrap()).collect();
        let g: Vec<f64> = geometric_correction_line.split(',').map(|v| v.parse().unwrap()).collect();
        let s: Vec<&str> = swath_processed_line.split(',').collect();

        let t = p[0];

        let pose = Pose3D {
            position: Position { x: p[1], y: p[2], z: p[3] },
            orientation: Orientation { roll: p[4], pitch: p[5], yaw: p[6] },
        };

        let geometric_correction = GeometricCorrection {
            h_port: g[1],
            h_stb: g[2],
            r_fbr_port: g[3],
            r_fbr_stb: g[4],
        };

        let swath_processed = SwathProcessed {
            timestamp: t,
            port: parse_vec_u8(s[1]),
            starboard: parse_vec_u8(s[2]),
            samples_per_beam: 1000,
        };

        // run map buffering
        let t_start = Instant::now();
        map_generator.buffer_processed_swath_into_map(
            &pose,
            geometric_correction,
            swath_processed,
        );
        total_buffer_processed_swath_into_map_s += t_start.elapsed().as_secs_f64();
        count_buffer_processed_swath_into_map += 1;

        // get current swath map data and log it every loop
        let cell_map_m = map_generator.get_cell_map_m();
        logger_cell_map_m.log(t, cell_map_m);

        // Increment how many pings we have calculated so far
        // Trigger calculate_map every N pings
        // Or
        // Trigger calculate_map if time gap is too large
        ping_counter += 1;

        let mut buffer_full = false;
        if ping_counter >= MAP_UPDATE_EVERY_N_SWATHS {
            buffer_full = true;
        } 
        else if let Some(t_prev) = prev_t {
            let t_dt = t - t_prev;
            if t_dt > MAP_UPDATE_MAX_TIME_GAP {
                buffer_full = true;
            }
        }

        if buffer_full {
            let t_start = Instant::now();
            let map = map_generator.calculate_map(&pose);
            total_calculate_map_s += t_start.elapsed().as_secs_f64();
            count_calculate_map += 1;

            logger_map.log(t, &map);

            let chunk_map = map_generator.get_chunk_map();
            logger_chunk_map.log(t, chunk_map);
            
            ping_counter = 0;
        }

        prev_t = Some(t);
    }

    // Performance light analysis
    let avg_buffer_processed_swath_into_map_s =
        if count_buffer_processed_swath_into_map > 0 {
            total_buffer_processed_swath_into_map_s
                / count_buffer_processed_swath_into_map as f64
        } else {
            0.0
        };

    let avg_calculate_map_s =
        if count_calculate_map > 0 {
            total_calculate_map_s / count_calculate_map as f64
        } else {
            0.0
        };

    println!(
        "Average buffer_processed_swath_into_map runtime: {:.6} s ({:.3} ms) over {} calls",
        avg_buffer_processed_swath_into_map_s,
        avg_buffer_processed_swath_into_map_s * 1e3,
        count_buffer_processed_swath_into_map
    );

    println!(
        "Average calculate_map runtime: {:.6} s ({:.3} ms) over {} calls",
        avg_calculate_map_s,
        avg_calculate_map_s * 1e3,
        count_calculate_map
    );
}