// cargo run --bin test



use std::fs::File;
use std::io::{BufRead, BufReader};

use swath_processing_node::swath_processing_lib::types::*;
use swath_processing_node::swath_processing_lib::processor::process_swath;







// LOGGER 
use std::{
    fs::{create_dir_all},
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
// ---------- SWATH PROCESSED ----------
pub struct LoggerSwathProcessed {
    logger: Logger,
}

impl LoggerSwathProcessed {
    pub fn new() -> Self {
        Self {
            logger: Logger::new("swath_processed", &["t","port","starboard"])
        }
    }

    pub fn log<T: ToString>(&mut self, t: f64, port: &[T], starboard: &[T]) {
        let port = port.iter().map(|v| v.to_string()).collect::<Vec<_>>().join(" ");
        let stb  = starboard.iter().map(|v| v.to_string()).collect::<Vec<_>>().join(" ");
        self.logger.writer.write_record(&[
            t.to_string(),
            port,
            stb
        ]).unwrap();
    }
}










fn parse_vec_u8(s: &str) -> Vec<u8> {
    s.split_whitespace().map(|v| v.parse().unwrap()).collect()
}



fn main() {
    let n = 10_000;
    //let n = 5_000;
    //let n = 1_000;

    let pose_file = BufReader::new(File::open("test/data/pose.csv").unwrap());
    let alt_file  = BufReader::new(File::open("test/data/altitude.csv").unwrap());
    let swath_file= BufReader::new(File::open("test/data/swath_raw.csv").unwrap());

    let mut pose_iter = pose_file.lines().skip(1);
    let mut alt_iter  = alt_file.lines().skip(1);
    let mut swath_iter= swath_file.lines().skip(1);

    let mut logger = LoggerSwathProcessed::new();

    for _ in 0..n {
        let pose_line = pose_iter.next().unwrap().unwrap();
        let alt_line  = alt_iter.next().unwrap().unwrap();
        let swath_line= swath_iter.next().unwrap().unwrap();

        let p: Vec<f64> = pose_line.split(',').map(|v| v.parse().unwrap()).collect();
        let a: Vec<f64> = alt_line.split(',').map(|v| v.parse().unwrap()).collect();
        let s: Vec<&str> = swath_line.split(',').collect();

        let t = p[0];

        let pose = Pose3D {
            position: Position { x: p[1], y: p[2], z: p[3] },
            orientation: Orientation { roll: p[4], pitch: p[5], yaw: p[6] },
        };

        let altitude = AltitudeMeasurement { value: a[1] };

        let swath_raw = SwathRaw {
            timestamp: t,
            port: parse_vec_u8(s[1]),
            starboard: parse_vec_u8(s[2]),
            samples_per_beam: 1000,
        };

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
                alpha: 60.0_f64.to_radians(),
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
                alpha: 60.0_f64.to_radians(),
                is_reversed: false, // starboard is stored 0 -> 1000
                blind_zone_scale: 0.35, // empirically tuned from test data
            },
        };

        // empirically tuned from test data
        let illumination_ema_period = 200;
        let out = process_swath(&swath_raw, &pose, &altitude, &sonar, illumination_ema_period);

        logger.log(t, &out.port, &out.starboard);
    }
}