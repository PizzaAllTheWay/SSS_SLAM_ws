// TODO: Fix these importas later not now man
use opencv::imgproc;
use opencv::prelude::*;
use opencv::core::{
    self,
    Mat,
    Size,
    Point,
    Scalar,
};

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

    // Image after applying filter.
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

            filtered_image: Mat::default(),
            segmented_image: Mat::default(),
        }
    }

    // TODO: Explain what the function does
    pub fn extract_features_from_map(
        &mut self,
        map: &Map,
    ) -> opencv::Result<()> {
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
        // TODO: Make function for labeling, openCV should have stuff already for it

        // TODO: Return features jesjes
        Ok(())
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

