use std::{
    fs::{File, create_dir_all},
    time::{SystemTime, UNIX_EPOCH},
};
use serde::Serialize;
use std::time::Instant;
use sysinfo::{System, get_current_pid};

use super::types::*;



// ---------- BASE (Skeleton for all loggers) ----------
pub struct Logger {
    writer: csv::Writer<File>,
}

impl Logger {
    pub fn new(name: &str, header: &[&str]) -> Self {
        let dir = "src/sss_data_processing/local_map_generator/logs/data/swath";
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

// ---------- ALTITUDE DVL ----------
pub struct LoggerAltitudeDvl {
    logger: Logger,
}

impl LoggerAltitudeDvl {
    pub fn new() -> Self {
        Self {
            logger: Logger::new(
                "altitude_dvl",
                &["t", "h_dvl"],
            )
        }
    }

    pub fn log(&mut self, t: f64, h_dvl: f64) {
        self.logger.log((t, h_dvl));
    }
}

// ---------- GEOMETRIC CORRECTION ----------
pub struct LoggerGeometricCorrection {
    logger: Logger,
}

impl LoggerGeometricCorrection {
    pub fn new() -> Self {
        Self {
            logger: Logger::new(
                "geometric_correction",
                &["t", "h_port", "h_stb", "r_fbr_port", "r_fbr_stb"],
            )
        }
    }

    pub fn log(&mut self, t: f64, g: &GeometricCorrection) {
        self.logger.log((
            t,
            g.h_port,
            g.h_stb,
            g.r_fbr_port,
            g.r_fbr_stb,
        ));
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
