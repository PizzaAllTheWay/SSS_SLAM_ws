// For simple test run:
// $ cargo run --bin test
//
// For profiling run:
// $ cargo flamegraph --release --bin test



use std::fs::File;
use std::io::{BufRead, BufReader};
use std::time::Instant;

use feature_extraction_node::feature_extraction_lib::extractor::FeatureExtractor;
use feature_extraction_node::feature_extraction_lib::types::*;

use opencv::prelude::*;

// LOGGER
use std::{
    fs::create_dir_all,
    time::{SystemTime, UNIX_EPOCH},
};



// ======================================================
// LOGGER BASE
// ======================================================

pub struct Logger {
    writer: csv::Writer<File>,
}

impl Logger {
    pub fn new(name: &str, header: &[&str]) -> Self {
        let dir = "test/logs/data";
        create_dir_all(dir).unwrap();

        let ts = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs();

        let path = format!("{}/{}_{}.csv", dir, name, ts);

        let file = File::create(path).unwrap();
        let mut writer = csv::Writer::from_writer(file);
        writer.write_record(header).unwrap();

        Self { writer }
    }

    pub fn flush(&mut self) {
        self.writer.flush().unwrap();
    }
}

// ======================================================
// PROCESSED IMAGE LOGGERS
// ======================================================

pub struct LoggerFilteredImage {
    logger: Logger,
}

impl LoggerFilteredImage {
    pub fn new() -> Self {
        Self {
            logger: Logger::new(
                "filtered_image",
                &[
                    "t",
                    "pose_x",
                    "pose_y",
                    "pose_yaw",
                    "resolution",
                    "width",
                    "height",
                    "image",
                ],
            ),
        }
    }

    pub fn log(&mut self, t: f64, map: &Map, image: &Mat) -> opencv::Result<()> {
        let width = image.cols() as usize;
        let height = image.rows() as usize;

        let image_bytes = image.data_typed::<u8>()?;
        let image_str = image_bytes
            .iter()
            .map(|v| v.to_string())
            .collect::<Vec<_>>()
            .join(" ");

        self.logger.writer.write_record(&[
            t.to_string(),
            map.pose.x.to_string(),
            map.pose.y.to_string(),
            map.pose.yaw.to_string(),
            map.resolution.to_string(),
            width.to_string(),
            height.to_string(),
            image_str,
        ]).unwrap();

        Ok(())
    }

    pub fn flush(&mut self) {
        self.logger.flush();
    }
}

pub struct LoggerSegmentedImage {
    logger: Logger,
}

impl LoggerSegmentedImage {
    pub fn new() -> Self {
        Self {
            logger: Logger::new(
                "segmented_image",
                &[
                    "t",
                    "pose_x",
                    "pose_y",
                    "pose_yaw",
                    "resolution",
                    "width",
                    "height",
                    "image",
                ],
            ),
        }
    }

    pub fn log(&mut self, t: f64, map: &Map, image: &Mat) -> opencv::Result<()> {
        let width = image.cols() as usize;
        let height = image.rows() as usize;

        let image_bytes = image.data_typed::<u8>()?;
        let image_str = image_bytes
            .iter()
            .map(|v| v.to_string())
            .collect::<Vec<_>>()
            .join(" ");

        self.logger.writer.write_record(&[
            t.to_string(),
            map.pose.x.to_string(),
            map.pose.y.to_string(),
            map.pose.yaw.to_string(),
            map.resolution.to_string(),
            width.to_string(),
            height.to_string(),
            image_str,
        ]).unwrap();

        Ok(())
    }

    pub fn flush(&mut self) {
        self.logger.flush();
    }
}

pub struct LoggerLandmarkSet {
    logger: Logger,
}

impl LoggerLandmarkSet {
    pub fn new() -> Self {
        Self {
            logger: Logger::new(
                "landmark_set",
                &[
                    "t",
                    "label_id",
                    "cx",
                    "cy",
                    "bbox_x",
                    "bbox_y",
                    "bbox_width",
                    "bbox_height",
                    "area",
                    "z_r",
                    "z_theta",
                    "mask_width",
                    "mask_height",
                    "mask",
                ],
            ),
        }
    }

    pub fn log(&mut self, t: f64, landmark_set: &LandmarkSet) -> opencv::Result<()> {
        for (label_id, landmark) in &landmark_set.landmarks {
            let mask = &landmark.bounding_box.mask;
            let mask_width = mask.cols() as usize;
            let mask_height = mask.rows() as usize;

            let mask_bytes = mask.data_typed::<u8>()?;
            let mask_str = mask_bytes
                .iter()
                .map(|v| v.to_string())
                .collect::<Vec<_>>()
                .join(" ");

            self.logger.writer.write_record(&[
                t.to_string(),
                label_id.to_string(),
                landmark.centroid.cx.to_string(),
                landmark.centroid.cy.to_string(),
                landmark.bounding_box.x.to_string(),
                landmark.bounding_box.y.to_string(),
                landmark.bounding_box.width.to_string(),
                landmark.bounding_box.height.to_string(),
                landmark.d.weak.area.to_string(),
                landmark.z.r.to_string(),
                landmark.z.theta.to_string(),
                mask_width.to_string(),
                mask_height.to_string(),
                mask_str,
            ]).unwrap();
        }

        Ok(())
    }

    pub fn flush(&mut self) {
        self.logger.flush();
    }
}



// ======================================================
// CSV HELPERS
// ======================================================

fn parse_vec_u8(s: &str) -> Vec<u8> {
    s.split_whitespace()
        .map(|v| v.parse::<u8>().unwrap())
        .collect()
}

fn reshape_flat_u8_to_2d(data: Vec<u8>, width: usize, height: usize) -> Vec<Vec<u8>> {
    assert_eq!(data.len(), width * height);

    data.chunks(width)
        .take(height)
        .map(|row| row.to_vec())
        .collect()
}



// ======================================================
// MAIN
// ======================================================

fn main() -> opencv::Result<()> {
    let start_n = 38;
    let n = 9;

    let map_file = BufReader::new(File::open("test/data/map.csv").unwrap());
    let origin_file = BufReader::new(File::open("test/data/map_origin.csv").unwrap());

    let mut map_iter = map_file.lines().skip(start_n);
    let mut origin_iter = origin_file.lines().skip(start_n);

    let mut logger_filtered_image = LoggerFilteredImage::new();
    let mut logger_segmented_image = LoggerSegmentedImage::new();
    let mut logger_landmark_set = LoggerLandmarkSet::new();

    // Bilateral filter settings.
    // These parameters control how strongly the image is smoothed while still trying to preserve edges.
    // The bilateral filter does not only look at how close pixels are in space, but also at how similar they are in intensity. 
    // This makes it useful when reducing speckle or small noise while keeping sharper boundaries, transitions, and object edges more visible.
    //
    // `filter_d` is the diameter of the local neighborhood in pixels.
    // Larger values use a bigger neighborhood, which gives stronger smoothing but costs more runtime.
    // Smaller values keep the filtering more local and usually preserve more fine detail.
    //
    // `filter_sigma_color` controls how much pixels with different intensity values are allowed to mix.
    // Larger values allow more blending across intensity differences, giving stronger smoothing but weaker edge preservation.
    // Smaller values preserve edges more strongly, but also reduce the denoising effect.
    //
    // `filter_sigma_space` controls how far away neighboring pixels can still influence the result.
    // Larger values spread the smoothing over a wider area.
    // Smaller values keep the smoothing tighter and more local.
    //
    // In short:
    // larger values -> stronger smoothing, less noise, more detail loss, weaker edge preservation
    // smaller values -> weaker smoothing, more noise left, better detail and edge preservation
    let filter_d: i32 = 31; // [pixel]
    let filter_sigma_color: f64 = 80.0;
    let filter_sigma_space: f64 = 100.0;

    // Semantic segmentation settings.
    // These parameters control how local bright/shadow candidates are found,
    // and how strict the later semantic support check is.
    //
    // `local_window_size` is the size of the local area used to compute the mean intensity around each pixel.
    // Larger values make the local mean more global and smoother, which reduces noise sensitivity
    // but can miss small local structures. Smaller values make the method more sensitive to local changes,
    // but also more sensitive to noise and speckles.
    //
    // `local_offset` is how much brighter or darker a pixel must be than its local mean
    // to be marked as a bright or shadow candidate.
    // Larger values make the threshold stricter, so fewer pixels are accepted.
    // Smaller values make it easier for pixels to become candidates, which increases sensitivity
    // but can also create more false positives.
    //
    // `search_radius` is the radius of the neighborhood used when checking whether
    // bright and shadow candidates support each other.
    // Larger values allow support from farther away pixels, which can preserve bigger object-shadow patterns
    // but may also connect unrelated regions. Smaller values make the support check more local and strict.
    //
    // `min_support` is the minimum number of nearby opposite-class pixels required
    // for a candidate pixel to survive the semantic filtering.
    // Larger values give stricter pruning and remove more isolated noise,
    // while smaller values keep more candidates but also let more weak or noisy regions survive.
    //
    // In short:
    // larger `local_window_size` -> more global behavior
    // larger `local_offset` -> stricter candidate detection
    // larger `search_radius` -> wider support search
    // larger `min_support` -> stricter semantic pruning
    let local_window_size = 51; // [pixel]
    let local_offset = 3.0;
    let search_radius = 21; // [pixel]
    let min_support = 100; // [pixel]

    // Minimum landmark area in pixels.
    // This is a final size-based pruning step applied after segmentation and labeling.
    //
    // Even if the segmentation is mostly good, some small leftover blobs, speckles,
    // or broken fragments can still survive and appear as separate labeled regions.
    // This threshold removes those by requiring a landmark to contain at least this many pixels.
    //
    // Larger values make the pruning stricter and keep only bigger landmark regions.
    // Smaller values keep more landmarks, but also allow more small noisy segments through.
    let landmark_area_min = 5_000; // [pixel]

    let mut extractor = FeatureExtractor::new(
        filter_d,
        filter_sigma_color,
        filter_sigma_space,

        local_window_size,
        local_offset,
        search_radius,
        min_support,

        landmark_area_min,
    );

    // Timing stats
    let mut total_extract_features_s: f64 = 0.0;
    let mut peak_extract_features_s: f64 = 0.0;
    let mut count_extract_features: usize = 0;

    for _ in 0..n {
        // Read, format an prep data for feature extraction
        let map_line = match map_iter.next() {
            Some(Ok(line)) => line,
            Some(Err(e)) => panic!("Failed to read map line: {}", e),
            None => break,
        };
        let origin_line = match origin_iter.next() {
            Some(Ok(line)) => line,
            Some(Err(e)) => panic!("Failed to read map_origin line: {}", e),
            None => break,
        };

        let map_parts: Vec<&str> = map_line.split(',').collect();
        let origin_parts: Vec<&str> = origin_line.split(',').collect();

        // map.csv: t, resolution, width, height, map
        let t = map_parts[0].parse::<f64>().unwrap();
        let resolution = map_parts[1].parse::<f64>().unwrap();
        let width = map_parts[2].parse::<usize>().unwrap();
        let height = map_parts[3].parse::<usize>().unwrap();
        let flat_map = parse_vec_u8(map_parts[4]);

        // map_origin.csv: t, x, y, yaw
        let origin_t = origin_parts[0].parse::<f64>().unwrap();
        let origin_x = origin_parts[1].parse::<i64>().unwrap();
        let origin_y = origin_parts[2].parse::<i64>().unwrap();
        let origin_yaw = origin_parts[3].parse::<f64>().unwrap();

        assert!((t - origin_t).abs() < 1e-9);

        let map_data = reshape_flat_u8_to_2d(flat_map, width, height);

        let map = Map {
            pose: Pose2DMap {
                x: origin_x,
                y: origin_y,
                yaw: origin_yaw,
            },
            resolution,
            width,
            height,
            data: map_data,
        };

        // Actual extraction, meat n taters
        let t_start = Instant::now();
        let landmark_set = extractor.extract_features_from_map(&map)?;
        let dt_extract_features_s = t_start.elapsed().as_secs_f64();

        total_extract_features_s += dt_extract_features_s;
        peak_extract_features_s = peak_extract_features_s.max(dt_extract_features_s);
        count_extract_features += 1;

        // Log data
        let filtered_image = extractor.get_filtered_image();
        let segmented_image = extractor.get_segmented_image();

        logger_filtered_image.log(t, &map, filtered_image)?;
        logger_segmented_image.log(t, &map, segmented_image)?;
        logger_landmark_set.log(t, &landmark_set)?;
    }

    logger_filtered_image.flush();
    logger_segmented_image.flush();
    logger_landmark_set.flush();

    let avg_extract_features_s =
        if count_extract_features > 0 {
            total_extract_features_s / count_extract_features as f64
        } else {
            0.0
        };

    println!(
        "extract_features_from_map runtime:\n\
         calls:   {}\n\
         average: {:.6} s ({:.3} ms)\n\
         peak:    {:.6} s ({:.3} ms)",
        count_extract_features,
        avg_extract_features_s,
        avg_extract_features_s * 1e3,
        peak_extract_features_s,
        peak_extract_features_s * 1e3,
    );

    Ok(())
}