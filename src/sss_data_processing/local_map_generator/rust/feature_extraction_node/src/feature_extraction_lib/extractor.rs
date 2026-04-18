// TODO: Fix these importas later not now man
use opencv::{
    core::{self, Mat, Size, Point},
    imgproc,
};

use opencv::prelude::*;

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
    ) -> Self {
        // Diameter must be at least 1 for bilateral filtering.
        let filter_d = filter_d.max(1);

        Self {
            filter_d,
            filter_sigma_color,
            filter_sigma_space,

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
        let input_image = _map_to_image(map)?;

        // Filter image ----------
        self.filtered_image = _filter_image(
            &input_image,
            self.filter_d,
            self.filter_sigma_color,
            self.filter_sigma_space,
        )?;

        // Segment image ----------
        self.segmented_image = _segment_objects_from_background(
            &self.filtered_image,
        )?;

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
// !!!
// TODO: IMPORTANT TO MENTION WE USE boundary-based methods HERE!!!!!!!! VERY IMPORTNAT TO MENTION MULTIPLE TIMES AND WHY WE USE THSOE ISNTEAD OF region-based methods like thresholding and region growing
// !!!


// ? NOTE: To remove teh map edges that are anoying just segment with super high thersholds and ten later subtract those from the image jesjes


// TODO: Describe function later not now though





// fn _segment_objects_from_background(
//     input_image: &Mat,
// ) -> opencv::Result<Mat> {
//     let mut segmented_image = Mat::default();

//     // TODO: Redo later
//     // Adaptive threshold settings.
//     //
//     // `adaptive_block_size` is the size of the local neighborhood used when OpenCV
//     // computes the threshold for each pixel.
//     // It must be an odd number greater than 1.
//     // Larger block size makes the threshold depend on a wider surrounding region,
//     // which gives smoother and more global behavior.
//     // Smaller block size makes the threshold react more locally to nearby intensity changes.
//     //
//     // `adaptive_c` is a constant subtracted from the locally computed threshold.
//     // Smaller positive `c` makes the threshold lower, so more pixels become white.
//     // Larger `c` makes the threshold higher, so fewer pixels become white.
//     //
//     // In short:
//     // larger block size -> smoother / less local thresholding
//     // smaller block size -> more local / more sensitive thresholding
//     //
//     // larger c -> more white foreground
//     // smaller c -> less white foreground
//     let adaptive_block_size: i32 = 51;
//     let adaptive_c: f64 = 3.0;

//     // TODO: Rewrite expanation of teh two
//     // Threshold mode used below:
//     // `THRESH_BINARY` means:
//     // - pixel > local threshold  -> 255
//     // - pixel <= local threshold -> 0
//     //
//     // Adaptive method used below:
//     // `ADAPTIVE_THRESH_GAUSSIAN_C` means the local threshold is computed from a
//     // Gaussian-weighted neighborhood around each pixel, so nearby pixels closer to
//     // the center affect the threshold more than pixels farther away.
//     //
//     // So together this gives:
//     // - a locally computed Gaussian-based threshold for each pixel
//     // - binary output image with only black/background and white/foreground
//     imgproc::adaptive_threshold(
//         input_image,
//         &mut segmented_image,
//         255.0,
//         imgproc::ADAPTIVE_THRESH_GAUSSIAN_C,
//         imgproc::THRESH_BINARY,
//         adaptive_block_size,
//         adaptive_c,
//     )?;

//     return Ok(segmented_image);
// }




// fn _segment_objects_from_background(
//     filtered_image: &Mat,
// ) -> opencv::Result<Mat> {
//     let mut segmented_image = Mat::default();

//     // Lower and upper hysteresis thresholds for edge acceptance.
//     // Lower threshold keeps weaker edge candidates if they connect to strong edges.
//     // Upper threshold marks strong edge pixels directly.
//     let canny_threshold_low = 800.0;
//     let canny_threshold_high = 1550.0;

//     // Sobel kernel size used internally for gradient estimation.
//     // Common values are 3, 5, or 7. Smaller is sharper/noisier, larger is smoother.
//     let canny_aperture_size = 7;

//     // If true, uses the more accurate L2 gradient magnitude.
//     // If false, uses a slightly cheaper approximation.
//     let canny_l2_gradient = true;

//     imgproc::canny(
//         filtered_image,
//         &mut segmented_image,
//         canny_threshold_low,
//         canny_threshold_high,
//         canny_aperture_size,
//         canny_l2_gradient,
//     )?;

//     // !!! DIALation testing
//     // Dilation grows edge pixels outward so nearby weak/broken edges connect into larger contours.
//     // Larger kernel or more iterations gives more merging, but can also over-thicken edges.
//     let dilate_kernel_size = 3;
//     let dilate_iterations = 5;

//     let dilate_kernel = imgproc::get_structuring_element(
//         imgproc::MORPH_RECT,
//         Size::new(dilate_kernel_size, dilate_kernel_size),
//         Point::new(-1, -1),
//     )?;

//     let mut dilated_image = Mat::default();

//     imgproc::dilate(
//         &segmented_image,
//         &mut dilated_image,
//         &dilate_kernel,
//         Point::new(-1, -1),
//         dilate_iterations,
//         core::BORDER_CONSTANT,
//         imgproc::morphology_default_border_value()?,
//     )?;

//     return Ok(dilated_image);
// }













fn _segment_objects_from_background(
    filtered_image: &Mat,
) -> opencv::Result<Mat> {
    // Lower and upper hysteresis thresholds for edge acceptance.
    // Lower threshold keeps weaker edge candidates if they connect to strong edges.
    // Upper threshold marks strong edge pixels directly.

    // ? 13 Actually better than 11
    // let canny_threshold_low = 100.0;
    // let canny_threshold_high = 1300.0;


    // ? 22 ehhhhh good enough I guess for now
    // let canny_threshold_low = 570.0;
    // let canny_threshold_high = 1100.0;

    // ? 23 Actually good
    let canny_threshold_low = 100.0;
    let canny_threshold_high = 1200.0;



    // Morphology execution sequence applied after Canny.
    // Each entry says which operation to run and how many iterations to use.
    // let morph_sequence = [
    //     MorphOp::Close(3),
    //     MorphOp::Open(2),
    // ];

    let morph_sequence = [
        MorphOp::Close(5),
        MorphOp::Open(2)
    ].repeat(20);








    

    let segmented_image_map_edge_and_objects = _get_segmented_map(
        filtered_image,
        canny_threshold_low,
        canny_threshold_high,
        &morph_sequence,
    )?;

    return Ok(segmented_image_map_edge_and_objects);
}

    


// TODO: Better explamnation later not now though
fn _get_segmented_map(
    filtered_image: &Mat,
    canny_threshold_low: f64,
    canny_threshold_high: f64,
    morph_sequence: &[MorphOp],
) -> opencv::Result<Mat> {
    let mut segmented_image = Mat::default();

    // Sobel kernel size used internally for gradient estimation.
    // Common values are 3, 5, or 7. Smaller is sharper/noisier, larger is smoother.
    let canny_aperture_size = 7;

    // If true, uses the more accurate L2 gradient magnitude.
    // If false, uses a slightly cheaper approximation.
    let canny_l2_gradient = true;

    imgproc::canny(
        filtered_image,
        &mut segmented_image,
        canny_threshold_low,
        canny_threshold_high,
        canny_aperture_size,
        canny_l2_gradient,
    )?;

    // Morphology sequence:
    // TODO: explain later jesjes
    let mut morphed_image = segmented_image;

    for morph_op in morph_sequence {
        morphed_image = match *morph_op {
            MorphOp::Open(iterations) => _morph_op_opening(&morphed_image, iterations)?,
            MorphOp::Close(iterations) => _morph_op_closing(&morphed_image, iterations)?,
        };
    }

    return Ok(morphed_image);
}

// Morphological opening = erosion followed by dilation.
// It is used first to remove small isolated foreground noise, tiny speckles,
// and thin unwanted blobs while keeping larger candidate regions.
fn _morph_op_opening(
    segmented_image: &Mat,
    opening_iterations: i32,
) -> opencv::Result<Mat> {
    let mut morphed_image = Mat::default();

    // Small kernel removes isolated noise and tiny speckle blobs.
    // More iterations gives stronger cleanup, but can also remove thin real objects.
    let opening_kernel_size = 3;

    let opening_kernel = imgproc::get_structuring_element(
        imgproc::MORPH_RECT,
        Size::new(opening_kernel_size, opening_kernel_size),
        Point::new(-1, -1),
    )?;

    imgproc::morphology_ex(
        segmented_image,
        &mut morphed_image,
        imgproc::MORPH_OPEN,
        &opening_kernel,
        Point::new(-1, -1),
        opening_iterations,
        core::BORDER_CONSTANT,
        imgproc::morphology_default_border_value()?,
    )?;

    return Ok(morphed_image);
}

// Morphological closing = dilation followed by erosion.
// It is used after opening to fill small holes inside candidate objects
// and reconnect weak nearby fragments into more solid landmark regions.
fn _morph_op_closing(
    segmented_image: &Mat,
    closing_iterations: i32,
) -> opencv::Result<Mat> {
    let mut morphed_image = Mat::default();

    // Slightly larger kernel helps fill small holes and reconnect weak broken fragments.
    // More iterations gives stronger merging, but can also merge nearby separate objects.
    let closing_kernel_size = 5;

    let closing_kernel = imgproc::get_structuring_element(
        imgproc::MORPH_RECT,
        Size::new(closing_kernel_size, closing_kernel_size),
        Point::new(-1, -1),
    )?;

    imgproc::morphology_ex(
        segmented_image,
        &mut morphed_image,
        imgproc::MORPH_CLOSE,
        &closing_kernel,
        Point::new(-1, -1),
        closing_iterations,
        core::BORDER_CONSTANT,
        imgproc::morphology_default_border_value()?,
    )?;

    return Ok(morphed_image);
}










// // Iterative threshold segmentation.
// // Each pass raises the background floor, so the next firefly search is forced
// // to look only at stronger remaining intensities above the previous threshold.
// fn _segment_objects_from_background(
//     filtered_image: &Mat,
// ) -> opencv::Result<Mat> {
//     let first_threshold = _firefly_best_threshold(filtered_image, 1, 254)?;
//     let second_threshold = _firefly_best_threshold(filtered_image, first_threshold - 1, 254)?;

//     let mut segmented_image = Mat::new_rows_cols_with_default(
//         filtered_image.rows(),
//         filtered_image.cols(),
//         core::CV_8UC1,
//         core::Scalar::all(0.0),
//     )?;

//     let input_data = filtered_image.data_bytes()?;
//     let output_data = segmented_image.data_bytes_mut()?;

//     for i in 0..input_data.len() {
//         let v = input_data[i];
//         output_data[i] = if v >= second_threshold && v != 255 { 255 } else { 0 };
//     }

//     Ok(segmented_image)
// }

// // Firefly search for one best threshold inside a limited intensity range.
// // `min_level` is the current background floor, so lower values are ignored.
// fn _firefly_best_threshold(
//     filtered_image: &Mat,
//     min_level: u8,
//     max_level: u8,
// ) -> opencv::Result<u8> {
//     let firefly_count: usize = 500;
//     let firefly_iterations: usize = 50;

//     let beta0: f64 = 0.50;
//     let gamma: f64 = 1.01;
//     let alpha: f64 = 10.0;

//     let histogram = _compute_histogram_u8_range(filtered_image, min_level, max_level)?;

//     let mut seed: u64 = 123456789;
//     let mut fireflies: Vec<f64> = (0..firefly_count)
//         .map(|_| _rand_uniform(&mut seed, min_level as f64, max_level as f64))
//         .collect();

//     for _ in 0..firefly_iterations {
//         let scores: Vec<f64> = fireflies
//             .iter()
//             .map(|&t| _threshold_objective_otsu_like_range(&histogram, t, min_level as usize, max_level as usize))
//             .collect();

//         for i in 0..firefly_count {
//             for j in 0..firefly_count {
//                 if scores[j] > scores[i] {
//                     let distance = fireflies[j] - fireflies[i];
//                     let attractiveness = beta0 * (-gamma * distance * distance).exp();
//                     let random_step = alpha * _rand_uniform(&mut seed, -0.5, 0.5);

//                     fireflies[i] += attractiveness * distance + random_step;
//                     fireflies[i] = fireflies[i].clamp(min_level as f64, max_level as f64);
//                 }
//             }
//         }
//     }

//     let mut best_threshold = fireflies[0];
//     let mut best_score = _threshold_objective_otsu_like_range(
//         &histogram,
//         best_threshold,
//         min_level as usize,
//         max_level as usize,
//     );

//     for &t in &fireflies {
//         let score = _threshold_objective_otsu_like_range(
//             &histogram,
//             t,
//             min_level as usize,
//             max_level as usize,
//         );
//         if score > best_score {
//             best_score = score;
//             best_threshold = t;
//         }
//     }

//     Ok(best_threshold.round().clamp(min_level as f64, max_level as f64) as u8)
// }

// // Histogram only over the currently allowed intensity range.
// fn _compute_histogram_u8_range(
//     image: &Mat,
//     min_level: u8,
//     max_level: u8,
// ) -> opencv::Result<[u32; 256]> {
//     let data = image.data_bytes()?;
//     let mut hist = [0u32; 256];

//     for &v in data {
//         if v >= min_level && v <= max_level && v != 255 {
//             hist[v as usize] += 1;
//         }
//     }

//     Ok(hist)
// }

// // Otsu-like score, but only inside the active search window.
// fn _threshold_objective_otsu_like_range(
//     hist: &[u32; 256],
//     threshold: f64,
//     min_level: usize,
//     max_level: usize,
// ) -> f64 {
//     let t = threshold.round().clamp(min_level as f64, max_level as f64) as usize;

//     let total: f64 = (min_level..=max_level).map(|i| hist[i] as f64).sum();
//     if total <= 0.0 {
//         return 0.0;
//     }

//     let mut w0 = 0.0;
//     let mut sum0 = 0.0;
//     let mut w1 = 0.0;
//     let mut sum1 = 0.0;

//     for i in min_level..=t {
//         let h = hist[i] as f64;
//         w0 += h;
//         sum0 += h * i as f64;
//     }

//     for i in (t + 1)..=max_level {
//         let h = hist[i] as f64;
//         w1 += h;
//         sum1 += h * i as f64;
//     }

//     if w0 <= 0.0 || w1 <= 0.0 {
//         return 0.0;
//     }

//     let mu0 = sum0 / w0;
//     let mu1 = sum1 / w1;

//     let p0 = w0 / total;
//     let p1 = w1 / total;

//     p0 * p1 * (mu0 - mu1) * (mu0 - mu1)
// }

// fn _rand_uniform(seed: &mut u64, min: f64, max: f64) -> f64 {
//     *seed = seed
//         .wrapping_mul(6364136223846793005)
//         .wrapping_add(1);

//     let u = ((*seed >> 33) as f64) / ((1u64 << 31) as f64);
//     min + (max - min) * u
// }

// fn _second_round_segment(
//     filtered_image: &Mat,
// ) -> opencv::Result<Mat> {
//     let mut segmented_image = Mat::default();

//     // Lower and upper hysteresis thresholds for edge acceptance.
//     // Lower threshold keeps weaker edge candidates if they connect to strong edges.
//     // Upper threshold marks strong edge pixels directly.
//     let canny_threshold_low = 800.0;
//     let canny_threshold_high = 1550.0;

//     // Sobel kernel size used internally for gradient estimation.
//     // Common values are 3, 5, or 7. Smaller is sharper/noisier, larger is smoother.
//     let canny_aperture_size = 7;

//     // If true, uses the more accurate L2 gradient magnitude.
//     // If false, uses a slightly cheaper approximation.
//     let canny_l2_gradient = true;

//     imgproc::canny(
//         filtered_image,
//         &mut segmented_image,
//         canny_threshold_low,
//         canny_threshold_high,
//         canny_aperture_size,
//         canny_l2_gradient,
//     )?;

//     // !!! DIALation testing
//     // Dilation grows edge pixels outward so nearby weak/broken edges connect into larger contours.
//     // Larger kernel or more iterations gives more merging, but can also over-thicken edges.
//     let dilate_kernel_size = 3;
//     let dilate_iterations = 5;

//     let dilate_kernel = imgproc::get_structuring_element(
//         imgproc::MORPH_RECT,
//         Size::new(dilate_kernel_size, dilate_kernel_size),
//         Point::new(-1, -1),
//     )?;

//     let mut dilated_image = Mat::default();

//     imgproc::dilate(
//         &segmented_image,
//         &mut dilated_image,
//         &dilate_kernel,
//         Point::new(-1, -1),
//         dilate_iterations,
//         core::BORDER_CONSTANT,
//         imgproc::morphology_default_border_value()?,
//     )?;

//     return Ok(dilated_image);
// }






// Image Segmentation Functions (STOP) --------------------------------------------------

