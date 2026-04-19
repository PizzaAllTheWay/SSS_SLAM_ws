// TODO: Fix these importas later not now man
use opencv::imgproc;
use opencv::prelude::*;
use opencv::core::{
    self,
    Mat,
    Size,
    Point,
    Scalar,
    Rect,
};
use opencv::boxed_ref::BoxedRef;
use nalgebra::Matrix2;

use super::types::*;



pub struct FeatureExtractor {
    // TODO: Rewrite later
    // Image filter parameters
    // Bilateral filter parameters used for edge preserving smoothing of the map image.
    // These control the filtering configuration used to reduce local noise and speckle
    // while keeping sharper intensity transitions more visible than standard blur.
    filter_d: i32,
    filter_sigma_color: f64,
    filter_sigma_space: f64,

    // Segmentation parameters
    // local_window_size is the size of the local area used to compute the mean intensity.
    // local_offset is how much brighter or darker a pixel must be than the local mean
    // to be marked as a bright or shadow candidate.
    // search_radius is how far around each candidate pixel we look for support from the opposite class.
    // min_support is the minimum amount of nearby opposite-class pixels needed
    // for a candidate pixel to survive the semantic filtering.
    local_window_size: i32,
    local_offset: f64,
    search_radius: i32,
    min_support: i32,

    // Landmark labelling parameters 
    // Minimum allowed landmark area in pixels.
    // Smaller landmarks are discarded as likely noise or tiny fragments.
    landmark_area_min: i32,

    // Landmark measurement uncertainty parameters.
    // `sigma_r` and `sigma_theta` are the base standard deviations for range and bearing.
    // `alpha_r` and `alpha_theta` control how much that uncertainty grows with distance.
    sigma_r: f64,
    sigma_theta: f64,
    alpha_r: f64,
    alpha_theta: f64,

    // ? NOTE: `map_offset_yaw` is currently applied as a fixed alignment correction
    // ? between the pose yaw convention and the map/sonar ground-plane convention.
    // ? This works for the current setup, but ideally the underlying frame definition
    // ? should be made fully consistent so this extra offset is no longer needed.
    map_offset_yaw: f64,

    // Image after applying filter
    filtered_image: Mat,
    // Image after segmenting background from objects
    segmented_image: Mat,
}
impl FeatureExtractor {
    pub fn new(
        filter_d: i32,
        filter_sigma_color: f64,
        filter_sigma_space: f64,

        local_window_size: i32,
        local_offset: f64,
        search_radius: i32,
        min_support: i32,

        landmark_area_min: i32,

        sigma_r: f64,
        sigma_theta: f64,
        alpha_r: f64,
        alpha_theta: f64,

        map_offset_yaw: f64,
    ) -> Self {
        // Diameter must be at least 1 for bilateral filtering.
        let filter_d = filter_d.max(1);

        Self {
            filter_d,
            filter_sigma_color,
            filter_sigma_space,

            local_window_size,
            local_offset,
            search_radius,
            min_support,

            landmark_area_min,

            sigma_r,
            sigma_theta,
            alpha_r,
            alpha_theta,

            map_offset_yaw,

            filtered_image: Mat::default(),
            segmented_image: Mat::default(),
        }
    }

    // TODO: Explain what the function does
    pub fn extract_features_from_map(
        &mut self,
        map: &Map,
        altitude: f64,
    ) -> opencv::Result<LandmarkSet> {
        // Convert Map into image ----------
        // Convert the project map into an OpenCV image
        // so the later image processing steps can operate on it.
        let input_image = _map_to_image(map)?;

        // Filter image ----------
        // Smooth the image before segmentation.
        // This reduces speckle and tiny intensity fluctuations
        // while still preserving stronger local edges.
        self.filtered_image = _filter_image(
            &input_image,
            self.filter_d,
            self.filter_sigma_color,
            self.filter_sigma_space,
        )?;

        // Segment image ----------
        // Removes map-edge background, 
        // finds local bright/shadow semantic candidates,
        // and refines them into a cleaner final object mask.
        self.segmented_image = _segment_objects_from_background(
            &self.filtered_image,
            self.local_window_size,
            self.local_offset,
            self.search_radius,
            self.min_support,
        )?;

        // Label objects ----------
        // Label the segmented connected components and convert them into the projects LandmarkSet structure.
        let mut landmark_set = _label_segmented_objects(
            &self.segmented_image,
            self.landmark_area_min,
        )?;

        // Landmark Measurements ----------
        // Compute each landmarks range/bearing relative to the current map pose.
        // In addition, fill its measurement uncertainty matrix.
        _update_landmark_measurements(
            &map,
            &mut landmark_set,
            self.sigma_r,
            self.sigma_theta,
            self.alpha_r,
            self.alpha_theta,
            self.map_offset_yaw,
        );

        // Estimate Landmark Height ----------
        // TODO: Short explanation
        _estimate_landmark_height(
            &map,
            &self.filtered_image,
            &mut landmark_set,
            altitude,
        );

        // Landmark Descriptor ----------
        // TODO: Short explanation
        _update_landmark_descriptors(
            &input_image,
            &mut landmark_set,
        );

        // Return extracted features ----------
        return Ok(landmark_set);
    }

    // Functions for debugging
    pub fn get_filtered_image(&self) -> &Mat {
        return &self.filtered_image;
    }

    pub fn get_segmented_image(&self) -> &Mat {
        return &self.segmented_image;
    }
}



// Conversion Functions (START) --------------------------------------------------
// Converts the internal map representation into an OpenCV matrix.
//
// Why this exists:
// The mapping pipeline stores data in the projects own `Map` type,
// while OpenCV image processing functions operate on `Mat` (Custom matrix data structure).
// This function is the bridge between those two representations.
fn _map_to_image(
    map: &Map
) -> opencv::Result<Mat> {
    let rows: Vec<&[u8]> = map.data.iter().map(|row| row.as_slice()).collect();
    Mat::from_slice_2d(&rows)
}
// Conversion Functions (STOP) --------------------------------------------------



// Image Filter Functions (START) --------------------------------------------------
// Applies edge-preserving smoothing to the input map image using a bilateral filter.
//
// This step is used to reduce local speckle noise and small intensity fluctuations
// before later thresholding and feature extraction. Unlike a normal Gaussian blur,
// the bilateral filter does not smooth only by spatial distance, but also considers
// intensity similarity between nearby pixels.
//
// Because of that, noisy regions are smoothed while stronger object boundaries,
// echoes, and shadow edges are better preserved.
//
// In short:
// reduce small local noise, while keeping important image edges sharper.
fn _filter_image(
    input_image: &Mat,
    filter_d: i32,
    filter_sigma_color: f64,
    filter_sigma_space: f64,
) -> opencv::Result<Mat> {
    // Output image for this filtering stage.
    let mut filtered_image = Mat::default();

    // Apply bilateral filtering with OpenCV default border handling.
    // `BORDER_DEFAULT` uses the standard reflected edge rule, 
    // which helps avoid artificial dark/bright border artifacts during filtering near the image edges.
    imgproc::bilateral_filter(
        input_image,
        &mut filtered_image,
        filter_d,
        filter_sigma_color,
        filter_sigma_space,
        core::BORDER_DEFAULT,
    )?;

    return Ok(filtered_image);
}
// Image Filter Functions (STOP) --------------------------------------------------



// Image Segmentation Functions (START) --------------------------------------------------
// This section performs the full image segmentation pipeline after filtering.
//
// The goal is to separate likely object regions from the background in a way that fits
// side scan sonar better than a single global threshold. Instead of relying only on
// raw intensity, the method also uses local bright and shadow semantics, since objects
// often appear as local bright-shadow patterns rather than one uniform region.
//
// The pipeline first segments the dark map-edge/background region and removes it from
// further processing. It then extracts local bright and shadow candidates from the
// remaining image, and keeps only candidates that have nearby support from the opposite
// class. This removes isolated noise while preserving more meaningful object-like structure.
//
// Finally, morphology is applied to clean the result, reconnect useful fragments,
// and produce a more usable segmentation mask for later feature extraction.
fn _segment_objects_from_background(
    filtered_image: &Mat,
    local_window_size: i32,
    local_offset: f64,
    search_radius: i32,
    min_support: i32,
) -> opencv::Result<Mat> {
    // First find the dark map-edge/background region.
    // This mask is later used to suppress semantic detections
    // that belong to the unwanted background edge instead of real objects.
    let segmented_image_map_edge = _segmented_image_map_edge(filtered_image)?;

    // Build local semantic candidate masks from the filtered image.
    // One mask keeps likely shadows, the other keeps likely bright returns.
    let semantic_image_shadow_with_map_edge = _semantic_threshold_candidates_local(
        filtered_image,
        local_window_size,
        local_offset,
        LocalThresholdOp::Shadow,
    )?;
    let semantic_image_bright_with_map_edge = _semantic_threshold_candidates_local(
        filtered_image,
        local_window_size,
        local_offset,
        LocalThresholdOp::Bright,
    )?;

    // Remove anything that overlaps the map-edge/background mask.
    // This leaves only semantic candidates in the useful interior image region.
    let semantic_image_shadow = _apply_mask_remove_pixels(
        &semantic_image_shadow_with_map_edge,
        &segmented_image_map_edge,
    )?;
    let semantic_image_bright = _apply_mask_remove_pixels(
        &semantic_image_bright_with_map_edge,
        &segmented_image_map_edge,
    )?;

    // Keep only semantic pixels that have enough nearby support
    // from the opposite semantic class.
    // This removes isolated red/blue speckles and keeps more meaningful paired structure.
    let semantic_image_filtered = _filter_semantics(
        &semantic_image_shadow,
        &semantic_image_bright,
        search_radius,
        min_support,
    )?;

    // Final morphology cleanup and shaping.
    // This sequence was selected because it gives fairly nice coverage
    // of object-like regions while still removing some smaller noise.
    let morph_sequence = [
        MorphOp::Open(1),
        MorphOp::Dilate(1),
        MorphOp::Erode(1),
        MorphOp::Dilate(1),
        MorphOp::Open(2),
    ];

    let mut morphed_image = semantic_image_filtered;
    for morph_op in &morph_sequence {
        morphed_image = _morph_op(&morphed_image, *morph_op)?;
    }

    return Ok(morphed_image);
}

// Segments the dark outer map-edge/background region from the filtered image.
//
// The purpose of this step is not to detect objects.
// It only finds the darker background edge area so it can be removed
// before semantic bright/shadow analysis is done on the useful image content.
fn _segmented_image_map_edge(
    filtered_image: &Mat,
) -> opencv::Result<Mat> {
    let mut segmented_image_map_edge = Mat::default();

    // Otsu automatically finds a threshold that separates the darker
    // background edge region from the brighter foreground content.
    // THRESH_BINARY_INV keeps the darker map-edge/background as white.
    imgproc::threshold(
        filtered_image,
        &mut segmented_image_map_edge,
        0.0,
        255.0,
        imgproc::THRESH_BINARY_INV | imgproc::THRESH_OTSU,
    )?;

    // Refine the raw map-edge mask with morphology.
    // Open removes tiny speckles, close fills small gaps,
    // and a strong dilation grows the edge mask so the later removal is conservative.
    let morph_sequence = [
        MorphOp::Open(1),
        MorphOp::Close(1),
        MorphOp::Dilate(10),
    ];

    let mut morphed_image = segmented_image_map_edge;

    for morph_op in &morph_sequence {
        morphed_image = _morph_op(&morphed_image, *morph_op)?;
    }

    return Ok(morphed_image);
}

// Applies one selected morphology operation to the input binary image.
//
// This is a shared helper for all morphology stages so the code stays compact.
// The enum decides which operation to run, how large kernel to use,
// and how many iterations to apply.
fn _morph_op(
    segmented_image: &Mat,
    morph_op: MorphOp,
) -> opencv::Result<Mat> {
    let mut morphed_image = Mat::default();

    // Pick OpenCV mode, kernel size, and iteration count
    // from the enum variant the caller requested.
    let (mode, kernel_size, iterations) = match morph_op {
        MorphOp::Open(iterations) => (
            imgproc::MORPH_OPEN,
            5,
            iterations,
        ),
        MorphOp::Close(iterations) => (
            imgproc::MORPH_CLOSE,
            15, // Note Bigger than normal to ensure proper closure
            iterations,
        ),
        MorphOp::Dilate(iterations) => (
            imgproc::MORPH_DILATE,
            15, // Note Bigger than normal to ensure proper growth
            iterations,
        ),
        MorphOp::Erode(iterations) => (
            imgproc::MORPH_ERODE,
            5,
            iterations,
        ),
    };

    // Build the structuring kernel for the chosen operation.
    // Smaller kernels act more gently, larger ones act more aggressively.
    let kernel = imgproc::get_structuring_element(
        imgproc::MORPH_RECT,
        Size::new(kernel_size, kernel_size),
        Point::new(-1, -1),
    )?;

    // Run the selected morphology pass.
    // Open removes small blobs, close fills gaps,
    // dilate expands regions, and erode shrinks them.
    imgproc::morphology_ex(
        segmented_image,
        &mut morphed_image,
        mode,
        &kernel,
        Point::new(-1, -1),
        iterations,
        core::BORDER_CONSTANT,
        imgproc::morphology_default_border_value()?,
    )?;

    return Ok(morphed_image);
}

// Creates a local semantic candidate mask from the filtered image.
//
// The idea is simple:
// compute a local mean image, shift it by an offset,
// then compare the filtered image against that shifted local mean.
//
// Bright mode keeps pixels above local mean + offset.
// Shadow mode keeps pixels below local mean - offset.
fn _semantic_threshold_candidates_local(
    filtered_image: &Mat,
    window_size: i32,
    offset: f64,
    threshold_op: LocalThresholdOp,
) -> opencv::Result<Mat> {
    let mut local_mean = Mat::default();

    // Compute a local mean around every pixel.
    // This gives a simple local reference intensity
    // instead of relying on one single global threshold.
    imgproc::blur(
        filtered_image,
        &mut local_mean,
        Size::new(window_size, window_size),
        Point::new(-1, -1),
        core::BORDER_REPLICATE,
    )?;

    // Build the local threshold image.
    // Bright mode uses mean + offset,
    // while shadow mode uses mean - offset.
    let mut local_threshold = Mat::default();

    match threshold_op {
        LocalThresholdOp::Bright => {
            core::add(
                &local_mean,
                &Scalar::all(offset),
                &mut local_threshold,
                &core::no_array(),
                -1,
            )?;
        }
        LocalThresholdOp::Shadow => {
            core::subtract(
                &local_mean,
                &Scalar::all(offset),
                &mut local_threshold,
                &core::no_array(),
                -1,
            )?;
        }
    }

    // Compare the filtered image against the local threshold image.
    // This produces a binary candidate mask for either bright or shadow semantics.
    let compare_mode = match threshold_op {
        LocalThresholdOp::Bright => core::CMP_GE,
        LocalThresholdOp::Shadow => core::CMP_LE,
    };

    let mut candidate_mask = Mat::default();
    core::compare(
        filtered_image,
        &local_threshold,
        &mut candidate_mask,
        compare_mode,
    )?;

    return Ok(candidate_mask);
}

// Removes pixels from an image wherever the given mask is active.
//
// In practice this is used to erase semantic candidates
// that fall inside the already-detected map-edge/background region.
fn _apply_mask_remove_pixels(
    image: &Mat,
    mask: &Mat,
) -> opencv::Result<Mat> {
    let mut masked_image = image.clone();

    // Wherever mask == 255, set the image pixel to 0.
    // This keeps the rest of the image unchanged.
    masked_image.set_to(&core::Scalar::all(0.0), mask)?;

    return Ok(masked_image);
}

// Filters semantic bright/shadow masks by requiring local support from the opposite class.
//
// Big picture:
// a bright candidate should have enough nearby shadow support,
// and a shadow candidate should have enough nearby bright support.
// This removes many isolated outliers and keeps more meaningful semantic structure.
fn _filter_semantics(
    semantic_image_shadow: &Mat,
    semantic_image_bright: &Mat,
    search_radius: i32,
    min_support: i32,
) -> opencv::Result<Mat> {
    let mut shadow_bin = Mat::default();
    let mut bright_bin = Mat::default();

    // Convert the incoming masks into simple binary form.
    // We only care whether a semantic pixel exists or not at each location.
    imgproc::threshold(
        semantic_image_shadow,
        &mut shadow_bin,
        0.0,
        1.0,
        imgproc::THRESH_BINARY,
    )?;
    imgproc::threshold(
        semantic_image_bright,
        &mut bright_bin,
        0.0,
        1.0,
        imgproc::THRESH_BINARY,
    )?;

    // Build a square support kernel from the requested search radius.
    // This neighborhood defines how far around each pixel
    // we look for support from the opposite semantic class.
    let kernel_size = 2 * search_radius + 1;
    let kernel = Mat::ones(kernel_size, kernel_size, core::CV_32F)?.to_mat()?;

    let mut shadow_bin_f = Mat::default();
    let mut bright_bin_f = Mat::default();
    shadow_bin.convert_to(&mut shadow_bin_f, core::CV_32F, 1.0, 0.0)?;
    bright_bin.convert_to(&mut bright_bin_f, core::CV_32F, 1.0, 0.0)?;

    let mut shadow_support = Mat::default();
    let mut bright_support = Mat::default();

    // Count nearby semantic support using a local convolution.
    // The resulting support images store how much shadow/bright evidence
    // exists around each pixel inside the chosen neighborhood.
    imgproc::filter_2d(
        &shadow_bin_f,
        &mut shadow_support,
        core::CV_32F,
        &kernel,
        Point::new(-1, -1),
        0.0,
        core::BORDER_CONSTANT,
    )?;
    imgproc::filter_2d(
        &bright_bin_f,
        &mut bright_support,
        core::CV_32F,
        &kernel,
        Point::new(-1, -1),
        0.0,
        core::BORDER_CONSTANT,
    )?;

    let mut bright_keep = Mat::default();
    let mut shadow_keep = Mat::default();

    // Keep only pixels that have enough opposite-class support nearby.
    // Bright survives if shadow support is high enough,
    // and shadow survives if bright support is high enough.
    core::compare(
        &shadow_support,
        &Scalar::all(min_support as f64),
        &mut bright_keep,
        core::CMP_GE,
    )?;
    core::compare(
        &bright_support,
        &Scalar::all(min_support as f64),
        &mut shadow_keep,
        core::CMP_GE,
    )?;

    let mut bright_paired = Mat::default();
    let mut shadow_paired = Mat::default();

    // Apply the keep masks back onto the original semantic masks.
    // This prunes away unsupported semantic pixels while preserving survivors.
    core::bitwise_and(
        semantic_image_bright,
        &bright_keep,
        &mut bright_paired,
        &core::no_array(),
    )?;
    core::bitwise_and(
        semantic_image_shadow,
        &shadow_keep,
        &mut shadow_paired,
        &core::no_array(),
    )?;

    let mut semantic_image_filtered = Mat::default();

    // Merge the surviving bright and shadow semantics into one final mask.
    // This combined semantic image is what the later morphology operates on.
    core::bitwise_or(
        &bright_paired,
        &shadow_paired,
        &mut semantic_image_filtered,
        &core::no_array(),
    )?;

    Ok(semantic_image_filtered)
}
// Image Segmentation Functions (STOP) --------------------------------------------------



// Labelling Functions (START) --------------------------------------------------
// Converts the final segmented binary image into a pruned `LandmarkSet`.
//
// Big picture:
// OpenCV first labels all connected foreground regions in the segmented image.
// From that raw labeling output, we build one `Landmark` per connected component,
// including its centroid, bounding box, area, and exact local mask.
//
// After that, a final area-based pruning step removes landmarks that are too small.
// This helps reject tiny leftover blobs and fragments that survived segmentation,
// And remove small artifacts generated by segmentation step, 
// so the returned set contains only the larger most probable candidate landmark regions.
fn _label_segmented_objects(
    segmented_image: &Mat,
    landmark_area_min: i32,
) -> opencv::Result<LandmarkSet> {
    let mut labels = Mat::default();
    let mut stats = Mat::default();
    let mut centroids = Mat::default();

    // Run connected-components labeling on the segmented image.
    // This assigns one integer label to each connected foreground region
    // and also computes per-label geometry tables.
    // Label based on 8-connectivity: 
    // pixels are connected if they share either an edge or a corner 
    // (includes diagonals)
    let connectivity = 8; 
    let num_labels = imgproc::connected_components_with_stats(
        segmented_image,
        &mut labels,
        &mut stats,
        &mut centroids,
        connectivity,
        core::CV_32S,
    )?;

    // Convert the raw OpenCV label tables into the projects LandmarkSet structure.
    // This fills per-landmark geometry such as centroid, area, bounding box, and mask.
    let mut landmark_set = _build_landmark_set_from_labels(
        &labels,
        &stats,
        &centroids,
        num_labels,
    )?;

    // Remove landmarks that are smaller than the requested minimum area.
    // This acts as one final cleanup step after segmentation and labeling.
    _prune_landmarks_by_area(
        &mut landmark_set,
        landmark_area_min,
    );

    return Ok(landmark_set);
}

// Builds a LandmarkSet from the raw connected-components output returned by OpenCV.
//
// Big picture:
// We loop over every labeled foreground component, read its geometry from `stats`,
// read its center from `centroids`, extract its exact local mask from the `labels` image,
// and then store all of that inside a `Landmark` in the final `LandmarkSet`.
//
// Important:
// label 0 is always the background, so we skip it.
fn _build_landmark_set_from_labels(
    labels: &Mat,
    stats: &Mat,
    centroids: &Mat,
    num_labels: i32,
) -> opencv::Result<LandmarkSet> {
    let mut landmark_set = LandmarkSet::default();

    // Skip label 0 because OpenCV reserves it for background.
    // Every other label corresponds to one connected foreground object.
    for label_id in 1..num_labels {
        // Read the bounding box and area for the current connected component.
        // OpenCV stores these in the stats table, one row per label.
        let x = *stats.at_2d::<i32>(label_id, imgproc::CC_STAT_LEFT)?;
        let y = *stats.at_2d::<i32>(label_id, imgproc::CC_STAT_TOP)?;
        let width = *stats.at_2d::<i32>(label_id, imgproc::CC_STAT_WIDTH)?;
        let height = *stats.at_2d::<i32>(label_id, imgproc::CC_STAT_HEIGHT)?;
        let area = *stats.at_2d::<i32>(label_id, imgproc::CC_STAT_AREA)?;

        // Read the centroid of the current label.
        // This is the geometric center of the labeled region in image coordinates.
        let cx = *centroids.at_2d::<f64>(label_id, 0)?;
        let cy = *centroids.at_2d::<f64>(label_id, 1)?;

        // Extract the local ROI (Region Of Interest) from the full labels image using the bounding box.
        // Then create an exact binary mask for only this label inside that ROI.
        // Pixels belonging to this label become 255, everything else becomes 0.
        let roi = Rect::new(x, y, width, height);
        let labels_roi = labels.roi(roi)?;

        let mut mask = Mat::default();
        core::compare(
            &labels_roi,
            &Scalar::all(label_id as f64),
            &mut mask,
            core::CMP_EQ,
        )?;

        // Start with an empty landmark and then fill only what we already know.
        // The remaining fields stay default/placeholder for now and can be filled later.
        let mut landmark = Landmark::new();

        landmark.d.weak.area = area;

        landmark.centroid = Centroid {
            cx,
            cy,
        };

        landmark.bounding_box = BoundingBox {
            x,
            y,
            width,
            height,
            mask,
        };

        // Save the finished landmark using the OpenCV label ID as the key.
        landmark_set.landmarks.insert(label_id, landmark);
    }

    return Ok(landmark_set);
}

// Removes landmarks whose labeled area is smaller than the requested minimum.
//
// Big picture:
// Very small connected components are often just leftover noise or tiny fragments.
// This step keeps only landmarks large enough to be worth treating as real candidates.
fn _prune_landmarks_by_area(
    landmark_set: &mut LandmarkSet,
    min_area: i32,
) {
    // Keep only landmarks whose connected-component area passes the threshold.
    landmark_set
        .landmarks
        .retain(|_, landmark| landmark.d.weak.area >= min_area);
}
// Labelling Functions (STOP) --------------------------------------------------



// Landmark Measurement Functions (START) --------------------------------------------------
// Computes the measurement for every detected landmark relative to the current map pose,
// and also assigns a simple uncertainty matrix to that measurement.
//
// After segmentation and labeling, each landmark already has image-space geometry such as
// centroid, area, bounding box, and mask. This stage turns that into a SLAM-style observation:
// a polar measurement `z = [r, theta]` and its corresponding covariance `R_z`
//
// In short, the landmark centroid is measured relative to the current pose, converted from
// pixels into meters using the map resolution, and expressed as range and bearing.
// A simple distance dependent covariance is then added so farther landmarks are treated
// as more uncertain in later matching and SLAM update steps.
#[allow(non_snake_case)]
fn _update_landmark_measurements(
    map: &Map,
    landmark_set: &mut LandmarkSet,
    sigma_r: f64,
    sigma_theta: f64,
    alpha_r: f64,
    alpha_theta: f64,
    map_offset_yaw: f64,
) {
    // Build the 2D vehicle/map yaw once before the loop.
    // We only work in the horizontal plane here, so a 2D yaw rotation is enough.
    // ? NOTE: `map_offset_yaw` is currently applied as a fixed alignment correction
    // ? between the pose yaw convention and the map/sonar ground-plane convention.
    // ? This works for the current setup, but ideally the underlying frame definition
    // ? should be made fully consistent so this extra offset is no longer needed.
    let pose2D = Pose2DMap {
        x: map.pose.x,
        y: map.pose.y,
        yaw: map.pose.yaw + map_offset_yaw,
    };

    // Loop through every landmark and update its measurement data.
    // `z` stores the range/bearing measurement from the current map pose to the landmark,
    // while `R_z` stores the uncertainty of that measurement for later weighting in SLAM.
    for landmark in landmark_set.landmarks.values_mut() {
        landmark.z = _calculate_landmark_measurement(
            &landmark.centroid,
            &pose2D,
            map.resolution,
        );

        landmark.R_z = _calculate_landmark_measurement_uncertainty(
            &landmark.z,
            sigma_r,
            sigma_theta,
            alpha_r,
            alpha_theta,
        );
    }
}

// Calculates one landmark measurement relative to the current map/vehicle pose.
//
// Big picture:
// this helper converts the landmark centroid from map pixel coordinates into a relative
// displacement from the current pose, converts that displacement into meters using the
// map resolution, and then expresses it in polar form as range `r` and bearing `theta`.
fn _calculate_landmark_measurement(
    centroid: &Centroid,
    pose: &Pose2DMap,
    map_resolution: f64,
) -> LandmarkMeasurement {
    // Relative landmark position in map pixel coordinates.
    let dx_pixel = centroid.cx - pose.x as f64;
    let dy_pixel = centroid.cy - pose.y as f64;

    // Convert pixel displacement into metric displacement.
    let dx_m = dx_pixel * map_resolution;
    let dy_m = dy_pixel * map_resolution;

    // Convert Cartesian displacement into a polar landmark measurement.
    // `r` is the range to the landmark, and `theta` is the bearing relative
    // to the robot heading, not the absolute angle in the map/world frame.
    let r = (dx_m * dx_m + dy_m * dy_m).sqrt();
    let theta = dy_m.atan2(dx_m) - pose.yaw;

    let z = LandmarkMeasurement {
        r,
        theta,
    };

    return z;
}

// Calculates the landmark measurement uncertainty matrix `R_z`.
//
// Big picture:
// the landmark measurement is stored in polar form as range `r` and bearing `theta`.
// This function builds a simple diagonal 2x2 covariance matrix where both range and
// bearing uncertainty increase with distance. Close landmarks keep the base noise,
// while farther landmarks get larger uncertainty through the scaling factors.
#[allow(non_snake_case)]
fn _calculate_landmark_measurement_uncertainty(
    landmark_measurement: &LandmarkMeasurement,
    sigma_r: f64,
    sigma_theta: f64,
    alpha_r: f64,
    alpha_theta: f64,
) -> Matrix2<f64> {
    // Use the measured landmark range as the distance-dependent scaling variable.
    let r = landmark_measurement.r;

    // Range variance grows with distance.
    let var_r = sigma_r.powi(2) * (1.0 + alpha_r * r.powi(2));

    // Bearing variance also grows with distance.
    let var_theta = sigma_theta.powi(2) * (1.0 + alpha_theta * r.powi(2));

    let R_z: Matrix2<f64> = Matrix2::new(
        var_r,     0.0,
        0.0,       var_theta,
    );

    return R_z;
}
// Landmark Measurement Functions (STOP) --------------------------------------------------



// Height Estimation Functions (START) --------------------------------------------------
// TODO:
// !!! Also clean up functions here



// TODO: Explain functions jesjes
fn _estimate_landmark_height(
    map: &Map,
    filtered_image: &Mat,
    landmark_set: &mut LandmarkSet,
    altitude: f64,
) {
    for landmark in landmark_set.landmarks.values_mut() {
        let estimated_shadow_length = _estimate_landmark_shadow_length(
            filtered_image,
            landmark,
            map.resolution,
        );

        // TODO:
        // let estimated_height = _calculate_landmark_height(
        //     estimated_shadow_length,
        //     altitude,
        // );

        //landmark.estimated_height = estimated_height;
        // TODO
        // ! TESTING REMOVE LATER
        //landmark.estimated_height = estimated_shadow_length;
        landmark.estimated_height = estimated_shadow_length;
    }
}

fn _estimate_landmark_shadow_length(
    filtered_image: &Mat,
    landmark: &mut Landmark,
    resolution: f64,
) -> f64 {
    // TODO: Explain later no time now
    let roi = Rect::new(
        landmark.bounding_box.x,
        landmark.bounding_box.y,
        landmark.bounding_box.width,
        landmark.bounding_box.height,
    );
    let image_roi = match filtered_image.roi(roi) {
        Ok(image_roi) => image_roi,
        Err(_) => return 0.0
    };
    
    let mut I_min = 0.0;
    let mut I_max = 0.0;

    if core::min_max_loc(
        &image_roi,
        Some(&mut I_min),
        Some(&mut I_max),
        None,
        None,
        &landmark.bounding_box.mask,
    ).is_err() {
        return 0.0;
    }

    if I_max <= I_min {
        return 0.0;
    }

    // TODO TUne thsi boi
    let threshold_bias = -5.0; // tune: 3.0, 5.0, 8.0, 10.0
    let otsu_threshold = (0.5 * (I_min + I_max) + threshold_bias).max(0.0);


    // !!!! DEbugg
    // TODO: REMOVE
    println!("threshold: {:.3}", otsu_threshold);

    // We now apply that locally estimated threshold to the whole filtered image.
    // This gives a global dark-region mask, but tuned using the landmark local ROI.
    // Dark pixels become foreground so possible shadow blobs are easy to label.
    let mut shadow_mask = Mat::default();
    if imgproc::threshold(
        filtered_image,
        &mut shadow_mask,
        otsu_threshold,
        255.0,
        imgproc::THRESH_BINARY_INV,
    ).is_err() {
        return 0.0;
    }

    // Next we label all connected dark regions in the thresholded full image.
    // Each labeled component becomes a shadow candidate that we can compare
    // against the current landmark using area and centroid distance.
    // Label based on 8-connectivity: 
    // pixels are connected if they share either an edge or a corner 
    // (includes diagonals)
    let mut labels = Mat::default();
    let mut stats = Mat::default();
    let mut centroids = Mat::default();
    let connectivity = 8; 
    
    let n_labels = match imgproc::connected_components_with_stats(
        &shadow_mask,
        &mut labels,
        &mut stats,
        &mut centroids,
        connectivity,
        core::CV_32S,
    ) {
        Ok(n) => n,
        Err(_) => return 0.0,
    };

    // We prune away tiny dark blobs because those are usually just noise,
    // speckle, or weak texture dips instead of a meaningful landmark shadow.
    // Here we require the shadow candidate to be at least some fraction
    // of the landmark segmented area. In addition if the shadow exceeds a certain
    // area proportion of landmark, that means it cant be landmarks shadow and is also pruned
    // TODO: TUNE THIS 
    // 0.33
    // 0.03 works fine
    let shadow_area_min_ratio = 0.03; // !!!! TUNE
    let shadow_area_max_ratio = 1.50; // !!!! TUNE
    let landmark_area = landmark.d.weak.area as f64;
    let shadow_area_min = shadow_area_min_ratio * landmark_area;
    let shadow_area_max = shadow_area_max_ratio * landmark_area;

    let landmark_cx = landmark.centroid.cx;
    let landmark_cy = landmark.centroid.cy;

    // Among all valid dark regions, choose the one whose centroid is closest
    // to the landmark centroid. This is a simple and usually robust heuristic
    // for picking the shadow that belongs to the landmark.
    let mut best_label = -1;
    let mut best_dist_sq = f64::INFINITY;

    for label in 1..n_labels {
        let area = match stats.at_2d::<i32>(label, imgproc::CC_STAT_AREA) {
            Ok(v) => *v as f64,
            Err(_) => continue,
        };

        if area < shadow_area_min {
            continue;
        }
        if area > shadow_area_max {
            continue;
        }

        let shadow_cx = match centroids.at_2d::<f64>(label, 0) {
            Ok(v) => *v,
            Err(_) => continue,
        };
        let shadow_cy = match centroids.at_2d::<f64>(label, 1) {
            Ok(v) => *v,
            Err(_) => continue,
        };

        let dx = shadow_cx - landmark_cx;
        let dy = shadow_cy - landmark_cy;
        let dist_sq = dx*dx + dy*dy;

        if dist_sq < best_dist_sq {
            best_dist_sq = dist_sq;
            best_label = label;
        }
    }

    if best_label < 0 {
        return 0.0;
    }

    // After selecting the most likely shadow component, we scan all pixels
    // that belong to that component and measure their distance from the
    // landmark centroid. The nearest and farthest shadow points define
    // the shadow extent along the image plane.
    let mut min_dist = f64::INFINITY;
    let mut max_dist = 0.0;

    for y in 0..labels.rows() {
        for x in 0..labels.cols() {
            let label = match labels.at_2d::<i32>(y, x) {
                Ok(v) => *v,
                Err(_) => continue,
            };

            if label != best_label {
                continue;
            }

            let dx = x as f64 - landmark_cx;
            let dy = y as f64 - landmark_cy;
            let dist = (dx * dx + dy * dy).sqrt();

            if dist < min_dist {
                min_dist = dist;
            }

            if dist > max_dist {
                max_dist = dist;
            }
        }
    }

    // The estimated shadow length is taken as the spread of the chosen shadow
    // region measured from the landmark centroid directionally outward.
    // If the component degenerates or becomes invalid, return zero safely.
    if !min_dist.is_finite() || max_dist <= min_dist {
        return 0.0;
    }

    let estimated_shadow_length_pixels = max_dist - min_dist;
    let estimated_shadow_length = estimated_shadow_length_pixels * resolution;

    return estimated_shadow_length;
}

// TODO: Explain that thsi is a extremely iffy shit
// Liek teh stimate here asumes flat gorund AND that flat-ground similar-triangles approximation
// WIthsc in reality its not but it is an estimate at least
fn _calculate_landmark_height(
    estimated_shadow_length: f64,
    altitude: f64,
) -> f64 {
    if altitude <= 0.0 || estimated_shadow_length <= 0.0 {
        return 0.0;
    }

    let estimated_height = altitude * estimated_shadow_length
        / (altitude + estimated_shadow_length);

    return estimated_height;
}
// Height Estimation Functions (STOP) --------------------------------------------------



// Landmark Descriptor Functions (START) --------------------------------------------------
// TODO: Explain functions
fn _update_landmark_descriptors(
    image: &Mat,
    landmark_set: &mut LandmarkSet,
) {
    for landmark in landmark_set.landmarks.values_mut() {
        // Extract the landmark ROI once from the full image.
        // If the bounding box is invalid or falls outside the image,
        // just reset the descriptors for this landmark and continue.
        let roi = Rect::new(
            landmark.bounding_box.x,
            landmark.bounding_box.y,
            landmark.bounding_box.width,
            landmark.bounding_box.height,
        );

        let image_roi = match image.roi(roi) {
            Ok(image_roi) => image_roi,
            Err(_) => {
                landmark.d = LandmarkDescriptors::default();
                continue;
            }
        };

        // Compute descriptors from the same local ROI so all descriptor functions
        // operate on the exact same landmark image patch.
        landmark.d.strong = _get_landmark_descriptors_strong(
            &image_roi,
            landmark,
        );

        landmark.d.weak = _get_landmark_descriptors_weak(
            &image_roi,
            landmark,
        );
    }
}

fn _get_landmark_descriptors_strong(
    image_roi: &BoxedRef<'_, Mat>,
    landmark: &mut Landmark,
) -> LandmarkDescriptorsStrong {
    // Calculate descriptors
    let mean_intensity = _get_landmark_descriptor_mean(
        &image_roi,
        &landmark.bounding_box,
    );
    let std = _get_landmark_descriptor_std(
        &image_roi,
        &landmark.bounding_box,
        mean_intensity,
        landmark.d.weak.area as f64,
    );
    let contrast = _get_landmark_descriptor_contrast(
        &image_roi,
        &landmark.bounding_box,
    );
    let entropy = _get_landmark_descriptor_entropy(
        &image_roi,
        &landmark.bounding_box,
        landmark.d.weak.area as f64,
    );

    // Build and return descriptor
    let d = LandmarkDescriptorsStrong {
        mean_intensity,
        std,
        contrast,
        entropy,
    };

    return d;
}

fn _get_landmark_descriptor_mean(
    image_roi: &BoxedRef<'_, Mat>,
    bounding_box: &BoundingBox,
) -> f64 {
    // Compute the mean only over pixels where the landmark mask is active.
    let mean_scalar = match core::mean(&image_roi, &bounding_box.mask) {
        Ok(mean) => mean,
        Err(_) => return 0.0,
    };

    return mean_scalar[0];
}

fn _get_landmark_descriptor_std(
    image_roi: &BoxedRef<'_, Mat>,
    bounding_box: &BoundingBox,
    mean: f64,
    area: f64,
) -> f64 {
    // Sum squared deviation from the mean over only the masked landmark pixels.
    let mut sum_sq = 0.0;

    for y in 0..bounding_box.height {
        for x in 0..bounding_box.width {
            // Get the pixel mask value (background = 0, object = 255)
            let mask_value = *bounding_box.mask.at_2d::<u8>(y, x).unwrap();

            // Calculate sum square of the pixel intensity if its a valid object pixel
            if mask_value > 0 {
                let pixel = *image_roi.at_2d::<u8>(y, x).unwrap() as f64;
                let diff = pixel - mean;
                sum_sq += diff * diff;
            }
        }
    }

    // Standard deviation = sqrt(average squared deviation from mean)
    let variance = if area > 0.0 {
        sum_sq/area
    } else {
        0.0
    };

    let std = variance.sqrt();

    return std;
}

#[allow(non_snake_case)]
fn _get_landmark_descriptor_contrast(
    image_roi: &BoxedRef<'_, Mat>,
    bounding_box: &BoundingBox,
) -> f64 {
    let mut I_min = 0.0;
    let mut I_max = 0.0;

    if core::min_max_loc(
        image_roi,
        Some(&mut I_min),
        Some(&mut I_max),
        None,
        None,
        &bounding_box.mask,
    ).is_err() {
        return 0.0;
    }

    // Small epsilon to ensure we don't divide by 0
    let eps = 1e-9;
    let contrast = (I_max - I_min)/(I_max + I_min + eps);

    return contrast;
}

#[allow(non_snake_case)]
fn _get_landmark_descriptor_entropy(
    image_roi: &BoxedRef<'_, Mat>,
    bounding_box: &BoundingBox,
    area: f64,
) -> f64 {
    // Compute the intensity histogram only for pixels that belong to the masked landmark.
    // Instead of using all 256 grayscale levels directly, we group them into `K` coarser bins.
    // This gives a compact intensity distribution for just the current landmark region.
    let K = 32;
    let hist_size = core::Vector::<i32>::from(vec![K]);
    let channels = core::Vector::<i32>::from(vec![0]);
    let ranges = core::Vector::<f32>::from(vec![0.0, 256.0]);
    
    let mut images = core::Vector::<Mat>::new();
    images.push(image_roi.clone_pointee());

    let mut hist = Mat::default();
    if imgproc::calc_hist(
        &images,
        &channels,
        &bounding_box.mask,
        &mut hist,
        &hist_size,
        &ranges,
        false,
    ).is_err() {
        return 0.0;
    }

    // Convert each histogram count `h_i` into a probability `p_i = h_i/area`,
    // then compute Shannon entropy over the landmark intensity distribution.
    // Higher entropy means the landmark contains a more varied spread of intensities,
    // while lower entropy means the intensities are more concentrated/similar.
    // Note that eps is here in order for log to never be 0
    let N = area; // Number of pixels in area is the same as area so just use that
    let eps = 1e-12;
    let mut entropy = 0.0;

    for i in 0..K {
        let h_i = match hist.at_2d::<f32>(i, 0) {
            Ok(v) => *v as f64,
            Err(_) => 0.0,
        };

        let p_i = h_i/N;
        if p_i > 0.0 {
            entropy -= p_i * (p_i + eps).log2();
        }
    }

    return entropy;
}

fn _get_landmark_descriptors_weak(
    image_roi: &BoxedRef<'_, Mat>,
    landmark: &mut Landmark,
) -> LandmarkDescriptorsWeak {
    // Calculate descriptors
    // area = Already calculated from labelling step, so we good :)
    // polar_coordinates = Also already calculated from measurement step, noice 0o0
    // height = Already have from estimate landmark height step, so it is very nice \(*-*)/
    let radial_intensity_gradient = _get_landmark_descriptor_gradient(
        &image_roi,
        &landmark.bounding_box,
        landmark.d.weak.area as f64,
    );

    // Build and return descriptor
    let d = LandmarkDescriptorsWeak {
        area: landmark.d.weak.area,
        polar_coordinates: landmark.z,
        height: landmark.estimated_height,
        radial_intensity_gradient,
    };

    return d;
}

#[allow(non_snake_case)]
fn _get_landmark_descriptor_gradient(
    image_roi: &BoxedRef<'_, Mat>,
    bounding_box: &BoundingBox,
    area: f64,
) -> f64 {
    if area <= 0.0 {
        return 0.0;
    }

    // Sobel settings.
    // We compute the spatial image derivative separately in x and y direction.
    // `sobel_ddepth` sets the output type for the gradient image.
    // We use CV_64F so negative gradients and larger derivative values are preserved safely.
    //
    // `sobel_kernel_size` controls how large derivative stencil is used.
    // A value of 3 is the common standard choice and gives a reasonable balance
    // between sensitivity and smoothing.
    //
    // `sobel_scale` scales the derivative result.
    // `sobel_delta` adds a constant offset after filtering, which we keep at 0 here.
    //
    // `sobel_border_type` controls how image borders are handled during filtering.
    let sobel_ddepth = core::CV_64F;
    let sobel_kernel_size = 3;
    let sobel_scale = 1.0;
    let sobel_delta = 0.0;
    let sobel_border_type = core::BORDER_DEFAULT;

    // Compute the horizontal and vertical image gradients inside the landmark ROI.
    // `grad_x` measures how intensity changes left-right,
    // while `grad_y` measures how intensity changes up-down.
    let mut grad_x = Mat::default();
    let mut grad_y = Mat::default();

    if imgproc::sobel(
        image_roi,
        &mut grad_x,
        sobel_ddepth,
        1,
        0,
        sobel_kernel_size,
        sobel_scale,
        sobel_delta,
        sobel_border_type,
    ).is_err() {
        return 0.0;
    }

    if imgproc::sobel(
        image_roi,
        &mut grad_y,
        sobel_ddepth,
        0,
        1,
        sobel_kernel_size,
        sobel_scale,
        sobel_delta,
        sobel_border_type,
    ).is_err() {
        return 0.0;
    }

    // For each valid landmark pixel, combine x and y gradients into one gradient magnitude:
    // sqrt(gx^2 + gy^2)
    //
    // This gives one local edge-strength value per landmark pixel.
    // We then average those values over the masked landmark area,
    // so the final descriptor becomes a simple measure of how strong
    // the intensity transitions are inside that landmark region.
    let mut sum_grad = 0.0;

    for y in 0..bounding_box.height {
        for x in 0..bounding_box.width {
            let mask_value = match bounding_box.mask.at_2d::<u8>(y, x) {
                Ok(v) => *v,
                Err(_) => 0,
            };

            if mask_value > 0 {
                let gx = match grad_x.at_2d::<f64>(y, x) {
                    Ok(v) => *v,
                    Err(_) => 0.0,
                };

                let gy = match grad_y.at_2d::<f64>(y, x) {
                    Ok(v) => *v,
                    Err(_) => 0.0,
                };

                sum_grad += (gx*gx + gy*gy).sqrt();
            }
        }
    }

    // Return the average gradient magnitude over the landmark pixels only.
    let gradient = sum_grad/area;

    return gradient;
}
// Landmark Descriptor Functions (STOP) --------------------------------------------------
