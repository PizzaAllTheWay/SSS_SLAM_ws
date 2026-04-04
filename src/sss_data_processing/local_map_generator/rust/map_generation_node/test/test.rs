// For simple test run:
// $ cargo run --bin test
// For profiling run:
// $ cargo flamegraph --release --bin test



use std::fs::File;
use std::io::{BufRead, BufReader};

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
                &["t", "pose_x", "pose_y", "pose_yaw", "width", "height", "map"],
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

    // ! TODO Might have to remake Params when done so we dont have useless dead code like params that we dont use here, not right now though
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
            theta: 25.0_f64.to_radians(),
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
            theta: 25.0_f64.to_radians(),
            is_reversed: false, // starboard is stored 0 -> 1000
            blind_zone_scale: 0.35, // empirically tuned from test data
        },
    };

    // ! TODO: trigger calculate_map every N pings
    const CALC_EVERY_N: usize = 100;

    // ! TODO: trigger calculate_map if time gap between pings gets too large
    const CALC_DT_THRESHOLD: f64 = 5.0;

    // ! TODO: init map generator before loop
    // This should ideally match the sonar sample spacing in range.
    // RECOMMENDED: sonar max range / number of swath samples.
    //
    // If map_resolution is made smaller than the sonar resolution:
    // - the map gets denser
    // - more pixels must be simulated / interpolated / stored
    // - runtime and memory cost increase
    // - but no real new information is created, since the sonar did not measure that finely
    //
    // If map_resolution is made larger than the sonar resolution:
    // - the map gets coarser
    // - runtime and memory cost decrease
    // - but fine details may be merged together or lost
    //
    // So in practice the sonar resolution is the best default tradeoff.
    let map_resolution = 30.0/1000.0;

    // ! TODO: init map generator before loop
    // Chunk size is the number of pixels per chunk in each direction.
    // Example: 64 means each chunk stores a 64x64 pixel region of the map.
    //
    // Larger chunk size:
    // - fewer chunks overall
    // - less hashmap / chunk-management overhead
    // - but each chunk holds more memory
    // - and updating / aging / clearing one chunk becomes heavier
    //
    // Smaller chunk size:
    // - more chunks overall
    // - more flexible local allocation
    // - better if only small map regions are active at a time
    // - but more chunk-management overhead
    //
    // 64x64 is a reasonable default starting point.
    let chunk_size = 64;

    // ! TODO: Chunk age
    // If a chunk exeeds this age
    // Ie it havsen been visited in that aunt of full buffer cycles CALC_EVERY_N
    // Then that chunk will get deleted
    let chunk_max_age = 1;

    // ! TODO: init map generator before loop
    // THsi is a very importnat thesholding that is ecsential to making this real time
    // Even after a lot of pruning we will be left with 10 000 000+ pixels per swath
    // In order to make this managable we need to check if it makes sense to furtehr inculde all tehse pixels
    // OFc not all pixels are important, most of them will have very little to do with the final result
    // So we can discrad these very small contribution and focus on pixels of teh map that contribute a lot
    // Thsi will enable real time perfoamnce as we decrease number of pixels by magnitude of factors down

    // *Testing: 0.3000 => 266 000 samples
    // *Testing: 1.0000 => 266 000 samples
    // *Testing: 1.5000 => 167 000 samples
    // *Testing: 1.7500 =>  79 000 samples
    // *Testing: 1.8000 =>  76 000 samples
    // *Testing: 1.8100 =>  60 000 samples
    // *Testing: 1.8200 =>  40 000 samples
    // *Testing: 1.8250 =>  26 000 samples
    // *Testing: 1.8280 =>  10 000 samples
    // *Testing: 1.8285 =>   4 000 samples
    //let probabilistic_map_threshold = 1.8285;

    // *Testing: 0.00200 =>  5 500 samples
    // *Testing: 0.00205 =>  5 200 samples
    // *Testing: 0.00210 =>  5 000 samples
    // *Testing: 0.00215 =>  4 700 samples
    // *Testing: 0.00220 =>  4 500 samples
    // *Testing: 0.00225 =>  4 300 samples
    // *Testing: 0.00250 =>  3 500 samples
    // *Testing: 0.00275 =>  2 800 samples
    // *Testing: 0.00300 =>  2 400 samples
    // *Testing: 0.00500 =>    700 samples
    // *Testing: 0.00600 =>    500 samples
    let probabilistic_map_threshold = 0.002000;

    // ! TODO: Very imprtant early pruning
    // This is an even earlier prunning parameter
    // Very imprtant
    // *Testing: 0.9900 =>  27 000 samples
    // *Testing: 0.9990 =>  10 000 samples
    // *Testing: 0.9993 =>   8 000 samples
    // *Testing: 0.9995 =>   6 000 samples
    let beam_weight_threshold = 0.9993;


    // ! TODO: Fill In
    // Fill in variables
    // How many neghbous with values to actualy fill the empty pixel
    // If fill_inn_min_neighbors high, the criteria fo rpixel to fill in is very strict, to big and no fill in wil happen (ie > 8 or more)
    // If fill_inn_min_neighbors is to low it will fill in way to early or miht even fit things worng (ie < 1 or less)
    // also how many lpasse we do because 1 pass will not be enough, 
    // After 1 pass we will fill in most of the gaps but not all so need a second round
    // The 3rd round is just to make sure there is no deadspace at all left
    // Note that having  fill_inn_passes to high will mean preformance if real time will tank as you go over million sof pixels over and over again
    // SO recomend to have no more than 5 fill inn passes to keep it real time
    let fill_inn_min_neighbors = 3;
    let fill_inn_passes = 3;


    let mut map_generator = MapGenerator::new(
        sonar,
        map_resolution,
        chunk_size,
        chunk_max_age,
        beam_weight_threshold,
        probabilistic_map_threshold,
        fill_inn_min_neighbors,
        fill_inn_passes,
    );

    // init logger before loop
    let mut logger_cell_map_m = LoggerCellMapM::new();
    let mut logger_chunk_map = LoggerChunkMap::new();
    let mut logger_map = LoggerMap::new();

    // Mapping logic
    let mut prev_t: Option<f64> = None;
    let mut ping_counter: usize = 0;

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
        map_generator.buffer_processed_swath_into_map(
            &pose,
            geometric_correction,
            swath_processed,
        );

        // get current swath map data and log it every loop
        let cell_map_m = map_generator.get_cell_map_m();
        logger_cell_map_m.log(t, cell_map_m);

        // Increment how many pings we have calculated so far
        // Trigger calculate_map every N pings
        // Or
        // Trigger calculate_map if time gap is too large
        ping_counter += 1;

        let mut buffer_full = false;
        if ping_counter >= CALC_EVERY_N {
            buffer_full = true;
        } 
        else if let Some(t_prev) = prev_t {
            let t_dt = t - t_prev;
            if t_dt > CALC_DT_THRESHOLD {
                buffer_full = true;
            }
        }

        if buffer_full {
            let map = map_generator.calculate_map(&pose);
            logger_map.log(t, &map);

            let chunk_map = map_generator.get_chunk_map();
            logger_chunk_map.log(t, chunk_map);
            
            ping_counter = 0;
        }

        prev_t = Some(t);
    }
}