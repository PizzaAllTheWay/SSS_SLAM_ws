use std::{
    fs::{File, create_dir_all},
    time::{SystemTime, UNIX_EPOCH},
};
use serde::Serialize;
use std::time::Instant;
use sysinfo::{System, get_current_pid};

use super::types::*;



pub struct Logger {
    writer: csv::Writer<File>,
}
impl Logger {
    pub fn new(name: &str, header: &[&str]) -> Self {
        let dir = "src/sss_data_processing/local_map_generator/logs/data/map";
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
        self.writer.flush().unwrap();
    }
}

// ---------- PERFORMANCE ----------
pub struct LoggerPerformance {
    logger: Logger,
    system: System,
    pid: sysinfo::Pid,
    t0: Option<Instant>,
}

impl LoggerPerformance {
    pub fn new() -> Self {
        let mut system = System::new_all();
        system.refresh_all();
        let pid = get_current_pid().unwrap();

        Self {
            logger: Logger::new(
                "performance",
                &["t", "runtime_s", "cpu_percent", "ram_mb"],
            ),
            system,
            pid,
            t0: None,
        }
    }

    pub fn start(&mut self) {
        self.t0 = Some(Instant::now());
    }

    pub fn stop(&mut self, t: f64) {
        let Some(t0) = self.t0.take() else {
            return;
        };

        let runtime_s = t0.elapsed().as_secs_f64();

        self.system.refresh_processes(sysinfo::ProcessesToUpdate::Some(&[self.pid]), true);

        let mut cpu_percent = 0.0;
        let mut ram_mb = 0.0;

        if let Some(proc_) = self.system.process(self.pid) {
            cpu_percent = proc_.cpu_usage() as f64;
            ram_mb = proc_.memory() as f64 / (1024.0 * 1024.0);
        }

        self.logger.log((t, runtime_s, cpu_percent, ram_mb));
    }
}

// ---------- MAP ALTITUDE ----------
pub struct LoggerMapAltitude {
    logger: Logger,
}

impl LoggerMapAltitude {
    pub fn new() -> Self {
        Self {
            logger: Logger::new(
                "altitude",
                &["t", "altitude"],
            )
        }
    }

    pub fn log(&mut self, t: f64, altitude: f64) {
        self.logger.log((t, altitude));
    }
}

// ---------- MAP POSE ----------
pub struct LoggerMapPose {
    logger: Logger,
}
impl LoggerMapPose {
    pub fn new() -> Self {
        Self {
            logger: Logger::new(
                "map_pose",
                &["t", "x", "y", "z", "roll", "pitch", "yaw"],
            ),
        }
    }

    pub fn log(&mut self, t: f64, pose: &Pose3D) {
        self.logger.log((
            t,
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.roll,
            pose.orientation.pitch,
            pose.orientation.yaw,
        ));
    }
}

// ---------- MAP ORIGIN ----------
pub struct LoggerMapOrigin {
    logger: Logger,
}
impl LoggerMapOrigin {
    pub fn new() -> Self {
        Self {
            logger: Logger::new(
                "map_origin",
                &["t", "x", "y", "yaw"],
            ),
        }
    }

    pub fn log(&mut self, t: f64, origin: &Pose2DMap) {
        self.logger.log((
            t,
            origin.x,
            origin.y,
            origin.yaw,
        ));
    }
}

// ---------- MAP ----------
pub struct LoggerMap {
    logger: Logger,
}
impl LoggerMap {
    pub fn new() -> Self {
        Self {
            logger: Logger::new(
                "map",
                &["t", "resolution", "width", "height", "map"],
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
            map.resolution.to_string(),
            map.width.to_string(),
            map.height.to_string(),
            map_str,
        ]).unwrap();
        self.logger.writer.flush().unwrap();
    }
}
