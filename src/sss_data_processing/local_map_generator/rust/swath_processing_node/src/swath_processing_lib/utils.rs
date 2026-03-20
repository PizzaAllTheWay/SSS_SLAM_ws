use std::{
    fs::{File, create_dir_all},
    time::{SystemTime, UNIX_EPOCH},
};
use serde::Serialize;

use super::types::*;



// ---------- BASE (Skeleton for all loggers) ----------
pub struct Logger {
    writer: csv::Writer<File>,
}

impl Logger {
    pub fn new(name: &str, header: &[&str]) -> Self {
        let dir = "src/sss_data_processing/local_map_generator/logs/data";
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

// ---------- POSE ----------
pub struct LoggerPose {
    logger: Logger,
}

impl LoggerPose {
    pub fn new() -> Self {
        Self {
            logger: Logger::new("pose_interpolated",
                &["t","x","y","z","roll","pitch","yaw"]
            )
        }
    }

    pub fn log(&mut self, t: f64, p: &Pose3D) {
        self.logger.log((
            t,
            p.position.x, p.position.y, p.position.z,
            p.orientation.roll, p.orientation.pitch, p.orientation.yaw
        ));
    }
}

// ---------- ALTITUDE ----------
pub struct LoggerAltitude {
    logger: Logger,
}

impl LoggerAltitude {
    pub fn new() -> Self {
        Self {
            logger: Logger::new("altitude_interpolated",
                &["t","altitude"]
            )
        }
    }

    pub fn log(&mut self, t: f64, alt: f64) {
        self.logger.log((t, alt));
    }
}

// ---------- SWATH RAW ----------
pub struct LoggerSwathRaw {
    logger: Logger,
}

impl LoggerSwathRaw {
    pub fn new() -> Self {
        Self {
            logger: Logger::new("swath_raw", &["t","port","starboard"])
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