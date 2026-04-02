// cargo run --bin test



use std::fs::File;
use std::io::{BufRead, BufReader};

use map_generation_node::map_generation_lib::types::*;



fn parse_vec_u8(s: &str) -> Vec<u8> {
    s.split_whitespace().map(|v| v.parse().unwrap()).collect()
}



fn main() {
    //let n = 10_000;
    //let n = 5_000;
    let n = 1_000;

    let pose_interpolated_file = BufReader::new(File::open("test/data/pose_interpolated.csv").unwrap());
    let geometric_correction_file  = BufReader::new(File::open("test/data/geometric_correction.csv").unwrap());
    let swath_processed_file= BufReader::new(File::open("test/data/swath_processed.csv").unwrap());

    let mut pose_interpolated_iter = pose_interpolated_file.lines().skip(1);
    let mut geometric_correction_iter  = geometric_correction_file.lines().skip(1);
    let mut swath_processed_iter= swath_processed_file.lines().skip(1);

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

        // TODOD: RUn the command


        // TODOD LOG DATA

        
    }
}