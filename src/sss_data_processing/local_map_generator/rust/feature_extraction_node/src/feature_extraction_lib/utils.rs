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
        let dir = "src/sss_data_processing/local_map_generator/logs/data/feature";
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

// ---------- LANDMARKS ----------
pub struct LoggerLandmarks {
    logger: Logger,
}

impl LoggerLandmarks {
    pub fn new() -> Self {
        Self {
            logger: Logger::new(
                "landmarks",
                &[
                    "t",
                    "label_id",
                    "cx",
                    "cy",
                    "bbox_x",
                    "bbox_y",
                    "bbox_width",
                    "bbox_height",
                    "z_r",
                    "z_theta",
                    "R_z_rr",
                    "R_z_rtheta",
                    "R_z_thetar",
                    "R_z_thetatheta",
                    "estimated_height",
                    "estimated_height_std",
                    "mean_intensity",
                    "std",
                    "contrast",
                    "entropy",
                    "area",
                    "weak_polar_r",
                    "weak_polar_theta",
                    "height_value",
                    "height_std",
                    "radial_intensity_gradient",
                ],
            ),
        }
    }

    pub fn log(&mut self, t: f64, landmark_set: &LandmarkSet) {
        for (label_id, landmark) in &landmark_set.landmarks {
            self.logger.writer.write_record(&[
                t.to_string(),
                label_id.to_string(),
                landmark.centroid.cx.to_string(),
                landmark.centroid.cy.to_string(),
                landmark.bounding_box.x.to_string(),
                landmark.bounding_box.y.to_string(),
                landmark.bounding_box.width.to_string(),
                landmark.bounding_box.height.to_string(),
                landmark.z.r.to_string(),
                landmark.z.theta.to_string(),
                landmark.R_z[(0, 0)].to_string(),
                landmark.R_z[(0, 1)].to_string(),
                landmark.R_z[(1, 0)].to_string(),
                landmark.R_z[(1, 1)].to_string(),
                landmark.estimated_height.value.to_string(),
                landmark.estimated_height.std.to_string(),
                landmark.d.strong.mean_intensity.to_string(),
                landmark.d.strong.std.to_string(),
                landmark.d.strong.contrast.to_string(),
                landmark.d.strong.entropy.to_string(),
                landmark.d.weak.area.to_string(),
                landmark.d.weak.polar_coordinates.r.to_string(),
                landmark.d.weak.polar_coordinates.theta.to_string(),
                landmark.d.weak.height_value.to_string(),
                landmark.d.weak.height_std.to_string(),
                landmark.d.weak.radial_intensity_gradient.to_string(),
            ]).unwrap();

            self.logger.writer.flush().unwrap();
        }
    }
}
