use super::types::*;

pub fn process_swath(
    swath_raw: &SwathRaw, 
    pose: &Pose3D, 
    altitude: &AltitudeMeasurement, 
    sound_speed: &SoundSpeed
) -> SwathProcessed {

    // TODO: validate input (sizes, NaNs, etc.)

    // --- Copy raw data → float (no processing yet) ---
    let port: Vec<f32> = swath_raw.port.iter().map(|&v| v as f32).collect();
    let starboard: Vec<f32> = swath_raw.starboard.iter().map(|&v| v as f32).collect();

    // --- Compute sample rate (if possible) ---
    // TODO: move max_range to config instead of hardcoded
    let max_range = swath_raw.max_range;
    let n = swath_raw.samples_per_beam as f32;

    let slant_resolution = max_range / n;

    let sample_rate = if sound_speed.value > 0.0 {
        (sound_speed.value as f32) / (2.0 * slant_resolution)
    } else {
        0.0 // unknown
    };

    // TODO: intensity correction (TVG / normalization)
    // TODO: beam pattern correction
    // TODO: bottom detection
    // TODO: blind zone removal
    // TODO: slant → ground range projection
    // TODO: filtering (bilateral / SRBF)
    // TODO: normalization across swaths

    // --- Output ---
    SwathProcessed {
        pose: pose.clone(),
        altitude: altitude.clone(),
        port,
        starboard,
        sample_rate,
    }
}