use nalgebra::{
    Rotation3,
    Vector3,
};
use ta::{
    Next,
    indicators::ExponentialMovingAverage,
};

use super::types::*;



pub fn process_swath(
    swath_raw: &SwathRaw, 
    pose: &Pose3D, 
    altitude: &AltitudeMeasurement,
    sonar: &SonarParams,
    illumination_ema_period: usize,
) -> SwathProcessed {
    // Interpolate ----------
    let pose_interpolated = _interpolate_pose(pose);
    let altitude_interpolated = _interpolate_altitude(altitude);

    // Geometric Correction ----------
    let geometric_correction_data = _apply_geometric_correction(
        &pose_interpolated,
        &altitude_interpolated,
        sonar,
    );

    // Blind Zone Removal ----------
    let (port, starboard) = _remove_blind_zone(
        swath_raw,
        sonar,
        &geometric_correction_data,
    );

    // Intensity Normalization ----------
    let (port, starboard) = _intensity_normalization(
        &port,
        &starboard,
        swath_raw,
        sonar,
        illumination_ema_period,
        &geometric_correction_data,
    );

    // Output ----------
    SwathProcessed {
        pose_interpolated: pose_interpolated,
        geometric_correction_data: geometric_correction_data,
        port: port,
        starboard: starboard,
    }
}



// Interpolate Functions (START) --------------------------------------------------
// NOTE: interpolation intentionally NOT used
// Reason:
// - State estimator runs over 50 Hz, sonar less than 1 Hz, so its already well aligned
// - Drone moves slow, interpolation error is under cm level (pure noise at that point)
// - SLERP + linear interpolation adds compute + uncertainty with no gain
// - Experiments showed no improvement, only noise increase
// => we treat incoming pose as already "interpolated enough"
// If using a fast maneuvering vehicle (e.g. aggressive drone/AUV),
// interpolation (linear + SLERP) should be implemented as it can significantly improve spatial alignment of swaths.
fn _interpolate_pose(pose: &Pose3D) -> Pose3D {
    pose.clone()
}
// Same logic for altitude
// If fast dynamics / high vertical motion → consider interpolation
fn _interpolate_altitude(altitude: &AltitudeMeasurement) -> AltitudeMeasurement {
    altitude.clone()
}
// Interpolate Functions (STOP) --------------------------------------------------



// Geometric Correction Functions (START) --------------------------------------------------
// Computes the geometric correction terms needed later for blind-zone removal
// and slant-range correction. This function does not touch the swath samples.
// It only computes, for each transducer separately:
// 1. corrected transducer height above the seabed
// 2. estimated first-bottom-return slant range
//
// Why this is needed:
// The measured altitude belongs to the vehicle/body reference point, not to the
// sonar transducer itself. Since each transducer is mounted at a fixed offset
// from the body frame, roll and pitch change its true vertical position relative
// to the seabed. Port and starboard must therefore be treated independently.
//
// The kinematic idea is standard rigid-body geometry:
// - take the fixed body->transducer offset
// - rotate it using the current vehicle attitude
// - use the rotated vertical component to correct the measured altitude
// - then use the corrected height together with the transducer beam geometry
//   to estimate the first-bottom-return slant range
//
// Yaw is ignored because it rotates around the vertical axis and does not change
// transducer height above the seabed.
fn _apply_geometric_correction(
    pose: &Pose3D,
    altitude: &AltitudeMeasurement,
    sonar: &SonarParams,
) -> GeometricCorrection {
    let h_t_port = _transducer_height(
        pose,
        altitude,
        &sonar.transducer_port,
    );

    let h_t_stb = _transducer_height(
        pose,
        altitude,
        &sonar.transducer_stb,
    );

    let r_fbr_port = _first_bottom_return_slant_range(
        pose,
        h_t_port,
        &sonar.transducer_port,
    );

    let r_fbr_stb = _first_bottom_return_slant_range(
        pose,
        h_t_stb,
        &sonar.transducer_stb,
    );

    let geometric_correction_data = GeometricCorrection {
        h_port: h_t_port,
        h_stb: h_t_stb,
        r_fbr_port: r_fbr_port,
        r_fbr_stb: r_fbr_stb,
    };

    return geometric_correction_data;
}

// Computes corrected height of one transducer above the seabed.
//
// The transducer offset is defined in the body frame. Using the current roll and
// pitch, that offset is rotated into the world/level frame. The rotated z-part
// tells how much the transducer is vertically shifted relative to the body origin.
// That shift is then applied to the measured vehicle altitude.
fn _transducer_height(
    pose: &Pose3D,
    altitude: &AltitudeMeasurement,
    transducer: &TransducerParams,
) -> f64 {
    let roll = pose.orientation.roll;
    let pitch = pose.orientation.pitch;

    let r_world_body = Rotation3::from_euler_angles(roll, pitch, 0.0);

    let p_body_transducer = Vector3::new(
        transducer.offset.position.x,
        transducer.offset.position.y,
        transducer.offset.position.z,
    );

    let p_world_transducer_offset = r_world_body * p_body_transducer;

    let h_t_transducer = altitude.value - p_world_transducer_offset.z;

    return h_t_transducer;
}

// Computes the first-bottom-return slant range for one transducer.
//
// Why this is needed:
// After correcting the altitude to the actual transducer height above the seabed,
// we still need to estimate where the first valid seabed return can physically
// happen along the beam. That first return comes from the lower edge of the beam,
// not from the beam center, so we compute that direction and use it to recover
// the slant range.
fn _first_bottom_return_slant_range(
    pose: &Pose3D,
    h_t: f64,
    transducer: &TransducerParams,
) -> f64 {
    // Build the rigid-body rotations needed to express the transducer beam in
    // world coordinates:
    // - `R_world_body` applies the current vehicle attitude
    // - `R_body_transducer` applies the fixed mounting orientation of the transducer
    // - `R_world_transducer` is the combined world-space orientation of the transducer
    #[allow(non_snake_case)]
    let R_world_body = Rotation3::from_euler_angles(
        pose.orientation.roll,
        pose.orientation.pitch,
        pose.orientation.yaw,
    );

    #[allow(non_snake_case)]
    let R_body_transducer = Rotation3::from_euler_angles(
        transducer.offset.orientation.roll,
        transducer.offset.orientation.pitch,
        transducer.offset.orientation.yaw,
    );

    #[allow(non_snake_case)]
    let R_world_transducer = R_world_body * R_body_transducer;

    // The beam center is defined in the transducer local frame.
    // Here it is hardcoded as +Y because side-scan sonar transducers transmit
    // sideways relative to their own housing/frame. Since port/starboard facing
    // direction is already encoded in the transducer mounting orientation, this
    // beam-center vector can stay fixed for both sides.
    //
    // `R_beam_center_to_lower_edge` rotates from the beam centerline to the lower
    // beam edge. We use `alpha / 2` because `alpha` is the full vertical beamwidth,
    // so half of it moves from the center to one edge. That lower edge is what
    // gives the first possible seabed hit, which is exactly what we need here.
    let beam_center_transducer = Vector3::new(
        0.0,
        1.0,
        0.0
    );

    #[allow(non_snake_case)]
    let R_beam_center_to_lower_edge = Rotation3::from_euler_angles(
        transducer.alpha/2.0,
        0.0,
        0.0
    );

    let beam_lower_edge_transducer = R_beam_center_to_lower_edge * beam_center_transducer;

    // Rotate the lower-edge beam direction into the world frame.
    // `beam_world` is therefore the actual lower-edge beam direction after both
    // vehicle attitude and transducer mounting have been applied.
    //
    // The z component gives the vertical fraction of that beam direction.
    // Taking `abs()` removes sign dependence since we only care about downward
    // magnitude, and `max(1e-9)` avoids division by zero if the beam ever becomes
    // almost horizontal.
    //
    // The final division is just basic triangle/projection math:
    // if vertical_drop = slant_range * vertical_fraction,
    // then h_t = r_fbr * down_component  =>  r_fbr = h_t / down_component.
    let beam_world = R_world_transducer * beam_lower_edge_transducer;

    let down_component = beam_world.z.abs().max(1e-9);

    let r_fbr_transducer = h_t/down_component;

    return r_fbr_transducer;
}
// Geometric Correction Functions (STOP) --------------------------------------------------



// Blind Zone Removal Functions (START) --------------------------------------------------
// Removes the blind zone (water column before the first seabed return)
// from each channel using the already computed first-bottom-return range.
//
// Why this is needed:
// Side scan sonar samples near the transducer do not yet correspond to valid
// seabed reflections. They mainly contain water-column / no-bottom-return data.
// These samples should be removed or masked before later steps such as
// slant-range correction and intensity normalization.
//
// How it works:
// 1. The corrected first-bottom-return slant range is converted from meters
//    into a sample/bin index.
// 2. That index marks where valid seabed returns begin.
// 3. Everything before that point is masked out.
//
// Important detail:
// The function does not reorder the sonar data.
// Each channel is masked directly in its native storage order.
// This is why `is_reversed` is needed:
// - if samples are stored near -> far, blind zone is at the front
// - if samples are stored far  -> near, blind zone is at the back
//
// This keeps the processing lightweight and avoids unnecessary array flips.
fn _remove_blind_zone(
    swath: &SwathRaw,
    sonar: &SonarParams,
    geometric_correction_data: &GeometricCorrection,
) -> (Vec<u8>, Vec<u8>) {
    // Convert corrected first-bottom-return ranges from meters to bin indices.
    // These indices tell us where the first valid seabed return appears
    // in each channel.
    // NOTE:
    // The geometric first-bottom-return estimate tends to overpredict the blind-zone
    // boundary in practice. This issue was also observed in the original thesis work
    // that this implementation is based on, where the exact cause of the mismatch
    // was not fully resolved. Most likely it comes from a difference between the
    // first physically possible seabed return given by geometry and the first strong
    // measured backscatter seen in the real sonar data. For now, a per-channel
    // scaling factor is applied as a practical correction.
    let bin_fbr_port = _range_to_bin(
        swath,
        sonar.transducer_port.max_range,
        sonar.transducer_port.blind_zone_scale,
        geometric_correction_data.r_fbr_port,
    );
    let bin_fbr_stb = _range_to_bin(
        swath,
        sonar.transducer_stb.max_range,
        sonar.transducer_stb.blind_zone_scale,
        geometric_correction_data.r_fbr_stb,
    );

    // Work on local copies so the raw input swath remains unchanged.
    let mut port = swath.port.clone();
    let mut starboard = swath.starboard.clone();

    // Mask blind zone in each channel according to its native sample direction.
    _mask_blind_zone(
        &mut port,
        bin_fbr_port,
        sonar.transducer_port.is_reversed,
    );

    _mask_blind_zone(
        &mut starboard,
        bin_fbr_stb,
        sonar.transducer_stb.is_reversed,
    );

    return (port, starboard);
}

// Converts first-bottom-return slant range from meters into a sample/bin index.
//
// The conversion uses the slant-range resolution of the swath:
//     slant_resolution = max_range / samples_per_beam
//
// The result is clamped so it always stays inside the valid array bounds.
fn _range_to_bin(
    swath: &SwathRaw,
    max_range: f64,
    scale: f64,
    r: f64,
) -> usize {
    let slant_resolution = max_range/swath.samples_per_beam as f64;
    let r_scaled = r * scale;
    let bin = (r_scaled/slant_resolution).floor();
    let bin_fbr = bin.clamp(0.0, swath.samples_per_beam as f64) as usize;

    return bin_fbr;
}

// Masks the blind-zone portion of one channel in-place.
//
// `bin_fbr` is the first-bottom-return index.
// All samples before that return are considered invalid and are set to zero.
//
// The `is_reversed` flag tells which side of the array corresponds to
// near-range samples:
// - false: near samples are at the front  -> mask from the front
// - true:  near samples are at the back   -> mask from the back
fn _mask_blind_zone(
    channel: &mut [u8],
    bin_fbr: usize,
    is_reversed: bool,
) {
    let n = channel.len();
    let k = bin_fbr.min(n);

    if is_reversed {
        for v in channel.iter_mut().skip(n.saturating_sub(k)) {
            *v = 0;
        }
    } else {
        for v in channel.iter_mut().take(k) {
            *v = 0;
        }
    }
}
// Blind Zone Removal Functions (STOP) --------------------------------------------------



// Intensity Normalization Functions (START) --------------------------------------------------
// Normalizes intensity for both sonar channels by estimating and removing the
// large-scale illumination trend from each swath independently.
//
// Why this is needed:
// Even after blind-zone removal, the measured sonar intensity is still affected
// by range-dependent acoustic effects such as spreading loss, attenuation,
// beam pattern variation, and gain changes. These effects create a slow-varying
// brightness envelope across the swath that is not caused by actual seabed
// texture.
//
// The goal of intensity normalization is therefore:
//     R_hat(i) = I(i) / L_hat(i)
// where:
// - `I(i)`     is the measured swath intensity
// - `L_hat(i)` is the estimated illumination envelope
// - `R_hat(i)` is the normalized reflectivity estimate
//
// High-level algorithm:
// 1. Estimate the illumination map `L_hat` separately for port and starboard.
// 2. Compute normalized reflectivity by dividing measured intensity by `L_hat`.
// 3. Rescale the normalized values back into a practical `u8` image range for
//    visualization and later processing.
//
// Notes:
// - Normalization is done per swath, not globally over the whole image.
// - Blind-zone samples remain excluded from the estimation.
// - Port and starboard are treated independently because their geometry and
//   illumination profiles may differ.
fn _intensity_normalization(
    port: &Vec<u8>,
    starboard: &Vec<u8>,
    swath: &SwathRaw,
    sonar: &SonarParams,
    illumination_ema_period: usize,
    geometric_correction_data: &GeometricCorrection,
) -> (Vec<u8>, Vec<u8>) {
    let port_est_illum_map = _estimate_illumination_map(
        swath,
        port,
        sonar.transducer_port.max_range,
        sonar.transducer_port.is_reversed,
        sonar.transducer_port.blind_zone_scale,
        illumination_ema_period,
        geometric_correction_data.r_fbr_port,
    );
    let starboard_est_illum_map = _estimate_illumination_map(
        swath,
        starboard,
        sonar.transducer_stb.max_range,
        sonar.transducer_stb.is_reversed,
        sonar.transducer_stb.blind_zone_scale,
        illumination_ema_period,
        geometric_correction_data.r_fbr_stb,
    );

    let port_est_ref_map = _estimate_reflectivity_map(
        &port_est_illum_map,
        swath,
        port,
        sonar.transducer_port.max_range,
        sonar.transducer_port.is_reversed,
        sonar.transducer_port.blind_zone_scale,
        geometric_correction_data.r_fbr_port,
    );
    let starboard_est_ref_map = _estimate_reflectivity_map(
        &starboard_est_illum_map,
        swath,
        starboard,
        sonar.transducer_stb.max_range,
        sonar.transducer_stb.is_reversed,
        sonar.transducer_stb.blind_zone_scale,
        geometric_correction_data.r_fbr_stb,
    );

    return (port_est_ref_map, starboard_est_ref_map);
}

// Estimates the slowly varying illumination envelope `L_hat(i)` for one sonar channel.
//
// Why this is needed:
// The measured swath contains both:
// - high-frequency local seabed texture / reflectivity
// - low-frequency illumination trends caused by sonar physics
//
// For normalization, we only want the slow illumination component.
// This function therefore extracts the valid seabed part of the channel and
// applies a lightweight low-pass smoothing operation to estimate `L_hat`.
//
// Algorithm:
// 1. Compute the first-bottom-return bin so only valid seabed samples are used.
// 2. Read the valid part of the channel in logical near->far order.
// 3. Treat that valid intensity vector as `I(i)` from the normalization model.
// 4. Apply a forward EMA pass.
// 5. Apply a reverse EMA pass to reduce directional lag.
// 6. Return the resulting smoothed envelope as `L_hat`.
//
// Why EMA is used here:
// The original spline / cost-function style smoothing is mathematically nice,
// but was too computationally expensive for real-time use in this pipeline.
// A first-order IIR low-pass filter based on EMA gives a very similar slow
// illumination estimate at a much lower cost.
//
// Forward + reverse EMA:
// A single EMA introduces directional lag. Running EMA once forward and once
// backward produces a more symmetric smoothing result, which is better suited
// for envelope estimation.
//
// Tuning:
// `illumination_ema_period` controls how slowly `L_hat` is allowed to vary:
// - larger value  -> smoother, slower envelope
// - smaller value -> more responsive, more jagged envelope
fn _estimate_illumination_map(
    swath: &SwathRaw,
    channel: &Vec<u8>,
    max_range: f64,
    is_reversed: bool,
    blind_zone_scale: f64,
    illumination_ema_period: usize,
    r_fbr: f64,
) -> Vec<f64> {
    let n_bins = channel.len();

    // Convert the corrected first-bottom-return range into a bin index.
    // This marks where valid seabed returns begin after applying the same
    // practical blind-zone scaling used elsewhere in the pipeline.
    let bin_fbr = _range_to_bin(
        swath,
        max_range,
        blind_zone_scale,
        r_fbr,
    );

    // Extract only the valid seabed samples and read them in logical near->far
    // order. Blind-zone samples are intentionally excluded from the illumination
    // estimate so they do not bias the low-pass envelope.
    //
    // This valid intensity vector corresponds to I(i) in the normalization model.
    #[allow(non_snake_case)]
    let I: Vec<f64> = if is_reversed {
        channel[..n_bins.saturating_sub(bin_fbr)]
            .iter()
            .rev()
            .map(|&v| v as f64)
            .collect()
    } else {
        channel[bin_fbr.min(n_bins)..]
            .iter()
            .map(|&v| v as f64)
            .collect()
    };

    // If there are too few valid samples, a meaningful envelope estimate is not
    // possible. In that case, return the valid signal itself as a fallback.
    if I.len() < 5 {
        return I;
    }

    // Clamp the requested EMA period so it always stays valid for the current
    // signal length. Larger period means a smoother and slower-varying envelope.
    let ema_period = illumination_ema_period.min(I.len().max(2));

    // Forward EMA pass:
    // produces a cheap first-order low-pass estimate of the illumination trend.
    let mut ema_fwd = ExponentialMovingAverage::new(ema_period).unwrap();
    let mut forward = Vec::<f64>::with_capacity(I.len());
    for &x in &I {
        forward.push(ema_fwd.next(x));
    }

    // Reverse EMA pass:
    // run the same low-pass smoothing in the opposite direction to reduce the
    // phase lag introduced by the forward-only EMA.
    let mut ema_rev = ExponentialMovingAverage::new(ema_period).unwrap();
    let mut backward_rev = Vec::<f64>::with_capacity(I.len());
    for &x in forward.iter().rev() {
        backward_rev.push(ema_rev.next(x));
    }

    // Flip back so the final illumination estimate is returned in the same
    // logical near->far ordering as the extracted valid signal.
    #[allow(non_snake_case)]
    let L_hat = backward_rev.into_iter().rev().collect();

    return L_hat;
}

// Estimates the normalized reflectivity map `R_hat` for one channel.
//
// Why this is needed:
// Once the illumination envelope `L_hat` has been estimated, the channel can
// be normalized by dividing the measured intensity by this envelope:
//     R_hat(i) = I(i) / L_hat(i)
//
// This suppresses the large-scale illumination trend and preserves relative
// local contrast caused more by actual seabed reflectivity than by sonar
// propagation effects.
//
// Algorithm:
// 1. Recompute the valid seabed region boundary.
// 2. Collect the valid indices in logical near->far order.
// 3. Compute raw normalized reflectivity values by dividing `I / L_hat`.
// 4. Find the min/max of the normalized valid values.
// 5. Rescale them back into a practical `u8` display range using the original
//    valid channel span.
// 6. Reinsert the normalized result into a full channel, leaving blind-zone
//    samples as zero.
//
// Why rescaling is needed:
// The physical normalized reflectivity values are floating-point ratios and are
// not directly suitable for storage in the same `u8` channel representation.
// To keep the pipeline simple and image-compatible, the normalized valid region
// is mapped back into a practical 8-bit intensity span.
//
// Important note:
// This rescaling is mainly for representation / visualization convenience.
// It is not a strict physical preservation of the raw reflectivity ratio.
fn _estimate_reflectivity_map(
    est_illum_map: &Vec<f64>,
    swath: &SwathRaw,
    channel: &Vec<u8>,
    max_range: f64,
    is_reversed: bool,
    blind_zone_scale: f64,
    r_fbr: f64,
) -> Vec<u8> {
    let n_bins = channel.len();

    // Use the same valid-boundary definition as the blind-zone removal and
    // illumination estimation steps so all normalization stages operate over
    // the exact same seabed sample region.
    let bin_fbr = _range_to_bin(
        swath,
        max_range,
        blind_zone_scale,
        r_fbr,
    );

    // Start from a zero-padded output so blind-zone samples remain invalid in
    // the final normalized channel as well.
    let mut r_hat_full = vec![0u8; n_bins];

    // Build the valid sample index list in logical near->far order, regardless
    // of how the channel is physically stored in memory.
    let valid_indices: Vec<usize> = if is_reversed {
        (0..n_bins.saturating_sub(bin_fbr)).rev().collect()
    } else {
        (bin_fbr.min(n_bins)..n_bins).collect()
    };

    // If there is no valid region or no illumination estimate, return the empty
    // zero-padded output.
    if valid_indices.is_empty() || est_illum_map.is_empty() {
        return r_hat_full;
    }

    // Measure the original valid intensity span of the channel.
    // This span is later reused as the output display range when mapping the
    // normalized reflectivity back into u8 image space.
    let mut valid_min = u8::MAX;
    let mut valid_max = u8::MIN;
    for &idx in &valid_indices {
        let v = channel[idx];
        valid_min = valid_min.min(v);
        valid_max = valid_max.max(v);
    }

    let out_min = valid_min as f64;
    let out_max = valid_max as f64;

    // Compute the raw reflectivity estimate over the valid region.
    // A very small floor is applied to L_hat to avoid division by zero.
    let mut r_hat_valid = Vec::<f64>::new();
    for (i, &idx) in valid_indices.iter().enumerate().take(est_illum_map.len()) {
        let l_hat = est_illum_map[i].max(1e-9);
        let r_hat = (channel[idx] as f64) / l_hat;
        r_hat_valid.push(r_hat);
    }

    // If no valid reflectivity values were produced, keep the zero-padded output.
    if r_hat_valid.is_empty() {
        return r_hat_full;
    }

    // Normalize the reflectivity values into [0, 1] using the valid-region min/max.
    // This removes the arbitrary floating-point ratio range before converting the
    // result back into the original u8-like image span.
    let r_min = r_hat_valid.iter().cloned().fold(f64::INFINITY, f64::min);
    let r_max = r_hat_valid.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let r_span = (r_max - r_min).max(1e-9);

    // Write the final normalized reflectivity back into the full output channel.
    // Only the valid seabed region is updated; blind-zone samples remain zero.
    for (i, &idx) in valid_indices.iter().enumerate().take(r_hat_valid.len()) {
        let r_norm = (r_hat_valid[i] - r_min) / r_span;
        let v_out = out_min + r_norm * (out_max - out_min);
        r_hat_full[idx] = v_out.clamp(0.0, 255.0) as u8;
    }

    r_hat_full
}
// Intensity Normalization Functions (STOP) --------------------------------------------------
