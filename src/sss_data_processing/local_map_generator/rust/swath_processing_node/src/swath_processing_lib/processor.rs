use nalgebra::{
    Rotation3,
    Vector3,
};
use interp::{
    InterpMode,
    interp,
};

use super::types::*;



pub fn process_swath(
    swath_raw: &SwathRaw, 
    pose: &Pose3D, 
    altitude: &AltitudeMeasurement, 
    sound_speed: &SoundSpeed,
    sonar: &SonarParams,
    blind_zone_scale: f64,
) -> SwathProcessed {
    // Interpolate ----------
    let pose_interpolated = _interpolate_pose(pose);
    let altitude_interpolated = _interpolate_altitude(altitude);

    // Geometric Correction ----------
    let geometric_correction_data = _apply_geometric_correction(
        pose,
        altitude,
        sonar,
    );

    // Blind Zone Removal ----------
    let (port, starboard) = _remove_blind_zone(
        swath_raw,
        sonar,
        blind_zone_scale,
        &geometric_correction_data,
    );

    // Slant Range Correction ----------
    let (port, starboard) = _slant_to_ground(
        &port,
        &starboard,
        swath_raw,
        sonar,
        blind_zone_scale,
        &geometric_correction_data,
    );

    // Intensity Normalization ----------
        // TODO: intensity_normalization()

    // Output ----------
    SwathProcessed {
        pose: pose_interpolated,
        altitude: altitude_interpolated,
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
/// Computes the geometric correction terms needed later for blind-zone removal
/// and slant-range correction. This function does not touch the swath samples.
/// It only computes, for each transducer separately:
/// 1. corrected transducer height above the seabed
/// 2. estimated first-bottom-return slant range
///
/// Why this is needed:
/// The measured altitude belongs to the vehicle/body reference point, not to the
/// sonar transducer itself. Since each transducer is mounted at a fixed offset
/// from the body frame, roll and pitch change its true vertical position relative
/// to the seabed. Port and starboard must therefore be treated independently.
///
/// The kinematic idea is standard rigid-body geometry:
/// - take the fixed body->transducer offset
/// - rotate it using the current vehicle attitude
/// - use the rotated vertical component to correct the measured altitude
/// - then use the corrected height together with the transducer beam geometry
///   to estimate the first-bottom-return slant range
///
/// Yaw is ignored because it rotates around the vertical axis and does not change
/// transducer height above the seabed.
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

    GeometricCorrection {
        h_port: h_t_port,
        h_stb: h_t_stb,
        r_fbr_port: r_fbr_port,
        r_fbr_stb: r_fbr_stb,
    }
}

/// Computes corrected height of one transducer above the seabed.
///
/// The transducer offset is defined in the body frame. Using the current roll and
/// pitch, that offset is rotated into the world/level frame. The rotated z-part
/// tells how much the transducer is vertically shifted relative to the body origin.
/// That shift is then applied to the measured vehicle altitude.
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

/// Computes first-bottom-return slant range for one transducer.
///
/// After the corrected transducer height is known, the first-bottom-return range
/// is obtained from simple right-triangle beam geometry:
///     r_fbr = h / sin(beta + alpha/2)
///
/// Port and starboard are already handled separately through their own transducer
/// parameters and their own corrected heights.
fn _first_bottom_return_slant_range(
    pose: &Pose3D,
    h_t: f64,
    transducer: &TransducerParams,
) -> f64 {
    let roll = pose.orientation.roll;

    let theta = transducer.beta;
    let alpha = transducer.alpha;

    // y > 0 => port  => use -roll
    // y < 0 => stb   => use +roll
    let roll_sign = -transducer.offset.position.y.signum();

    let angle_r_fbr = theta + alpha / 2.0 + roll_sign * roll;

    let r_fbr_transducer = h_t / angle_r_fbr.sin();

    return r_fbr_transducer;
}
// Geometric Correction Functions (STOP) --------------------------------------------------



// Blind Zone Removal Functions (START) --------------------------------------------------
/// Removes the blind zone (water column before the first seabed return)
/// from each channel using the already computed first-bottom-return range.
///
/// Why this is needed:
/// Side scan sonar samples near the transducer do not yet correspond to valid
/// seabed reflections. They mainly contain water-column / no-bottom-return data.
/// These samples should be removed or masked before later steps such as
/// slant-range correction and intensity normalization.
///
/// How it works:
/// 1. The corrected first-bottom-return slant range is converted from meters
///    into a sample/bin index.
/// 2. That index marks where valid seabed returns begin.
/// 3. Everything before that point is masked out.
///
/// Important detail:
/// The function does not reorder the sonar data.
/// Each channel is masked directly in its native storage order.
/// This is why `is_reversed` is needed:
/// - if samples are stored near -> far, blind zone is at the front
/// - if samples are stored far  -> near, blind zone is at the back
///
/// This keeps the processing lightweight and avoids unnecessary array flips.
fn _remove_blind_zone(
    swath: &SwathRaw,
    sonar: &SonarParams,
    blind_zone_scale: f64,
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
        geometric_correction_data.r_fbr_port * blind_zone_scale,
        swath,
    );
    let bin_fbr_stb = _range_to_bin(
        geometric_correction_data.r_fbr_stb * blind_zone_scale,
        swath
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

/// Masks the blind-zone portion of one channel in-place.
///
/// `bin_fbr` is the first-bottom-return index.
/// All samples before that return are considered invalid and are set to zero.
///
/// The `is_reversed` flag tells which side of the array corresponds to
/// near-range samples:
/// - false: near samples are at the front  -> mask from the front
/// - true:  near samples are at the back   -> mask from the back
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

/// Converts first-bottom-return slant range from meters into a sample/bin index.
///
/// The conversion uses the slant-range resolution of the swath:
///     slant_resolution = max_range / samples_per_beam
///
/// The result is clamped so it always stays inside the valid array bounds.
fn _range_to_bin(
    r_fbr: f64,
    swath: &SwathRaw,
) -> usize {
    let slant_resolution = swath.max_range/swath.samples_per_beam as f64;
    let bin = (r_fbr/slant_resolution).floor();
    let bin_fbr = bin.clamp(0.0, swath.samples_per_beam as f64) as usize;

    return bin_fbr;
}
// Blind Zone Removal Functions (STOP) --------------------------------------------------



// Slant Range Correction Functions (START) --------------------------------------------------
/// Converts both sonar channels from slant range to ground range.
///
/// Why this is needed:
/// After blind-zone removal, the remaining samples are still indexed by distance
/// along the acoustic beam path, not by true horizontal distance on the seabed.
/// This causes across-track geometric distortion in the swath. Slant-range
/// correction removes that distortion by projecting each valid sample onto the
/// seabed plane using the corrected transducer height estimated earlier.
///
/// High-level algorithm:
/// 1. For each channel separately, read valid samples in logical near->far order.
/// 2. Convert each valid slant-range bin to horizontal ground range.
/// 3. Store the result as irregular `(ground_range, intensity)` samples.
/// 4. Interpolate those irregular samples onto a uniform ground-range grid.
/// 5. Write the final result back in the original native storage order.
///
/// Port and starboard are processed independently, but with the exact same logic.
fn _slant_to_ground(
    port: &[u8],
    starboard: &[u8],
    swath: &SwathRaw,
    sonar: &SonarParams,
    blind_zone_scale: f64,
    geometric_correction_data: &GeometricCorrection,
) -> (Vec<u8>, Vec<u8>) {
    let port = _collect_ground_samples(
        port,
        geometric_correction_data.h_port,
        sonar.transducer_port.is_reversed,
        swath,
        blind_zone_scale,
    );
    let starboard = _collect_ground_samples(
        starboard,
        geometric_correction_data.h_stb,
        sonar.transducer_stb.is_reversed,
        swath,
        blind_zone_scale,
    );

    return (port, starboard);
}

/// Stage 1:
/// Collect valid projected samples from one channel.
///
/// Why this is needed:
/// The original channel is sampled uniformly in slant-range space, but after
/// projection onto the seabed the samples are no longer uniformly spaced.
/// Therefore, this first stage does not try to build the final output row yet.
/// Instead, it creates an intermediate sparse representation made of projected
/// `(ground_bin, intensity)` samples.
///
/// Algorithm:
/// 1. Walk through the channel in logical near->far order.
/// 2. Ignore blind-zone samples that were already masked to zero.
/// 3. Convert each bin index to physical slant range.
/// 4. Project slant range to ground range using flat-seabed geometry.
/// 5. Convert ground range to a target ground bin.
/// 6. Store `(ground_bin, intensity)` for later accumulation.
///
/// Intensities are kept as `u8` here because they are still just sample values.
/// No interpolation is done in this step.
fn _collect_ground_samples(
    channel: &[u8],
    h_t_transducer: f64,
    is_reversed: bool,
    swath: &SwathRaw,
    blind_zone_scale: f64,
) -> Vec<u8> {
    let n_bins: usize = channel.len();
    let slant_resolution = swath.max_range / swath.samples_per_beam as f64;

    let mut ground_channel = vec![0u8; n_bins];

    for slant_bin in 0..n_bins {
        let source_index = if is_reversed {
            n_bins - 1 - slant_bin
        } else {
            slant_bin
        };

        let intensity = channel[source_index];

        // Blind-zone samples were already masked in the previous step,
        // so only nonzero valid seabed samples are kept here.
        if intensity == 0 { continue; }

        // Convert the current bin to physical slant range and project it onto
        // the seabed plane. Invalid geometric cases are skipped. In practice,
        // these should already mostly be excluded by blind-zone removal.
        // ! let slant_range = slant_bin as f64 * slant_resolution;
        let slant_range = slant_bin as f64 * slant_resolution/blind_zone_scale;
        if slant_range <= h_t_transducer { continue; }

        let ground_range = (slant_range * slant_range - h_t_transducer * h_t_transducer).sqrt();

        // Convert projected ground range to the nearest target ground bin.
        // Samples outside the channel bounds are ignored.
        let mut ground_bin = (ground_range / slant_resolution).floor() as usize;
        ground_bin = ((ground_bin as f64) * blind_zone_scale).floor() as usize;
        if ground_bin >= n_bins { continue; }

        // keep strongest sample landing in this bin
        let target_index = if is_reversed {
            n_bins - 1 - ground_bin
            
        } else {
            ground_bin
        };

        ground_channel[target_index] = ground_channel[target_index].max(intensity);
    }

    return ground_channel;
}



// !!! STEP 2 


// Slant Range Correction Functions (STOP) --------------------------------------------------
