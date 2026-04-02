use super::types::*;



pub struct MapGenerator {
    // ? NOTE: if the vraible is NOT pub var, thne it is private even if class is publisk, very nice 
    // TODO: store V and P here
    // TODO: store chunk data here
    // TODO: store config / params here
}



impl MapGenerator {
    pub fn new() -> Self {
        Self {
            // TODO Add stuff here
        }
    }

    pub fn buffer_processed_swath_into_map(
        &mut self,
        pose: Pose3D,
        geometric_correction: GeometricCorrection,
        swath_processed: SwathProcessed,
        sonar: SonarParams,
    ) -> bool {
        // TODO Pruning stage =====
        // TODO Stage 1
        // r ground stuff

        // TODO Stage 2
        // ANgle theta stuff

        // TODO Stage 3
        // Find P_m for all q
        // Prune with emz on P_m

        // TODO V_m measured whatever map idk what to say interpolated swath to polar map idk what this V_m is called ======
        // DO calculations here

        // TODO V and P and Chunk management stuff ======
        // V + P + Chunk manage by updating age of revisited chunks here

        return true;
    }

    pub fn calculate_map(
        &mut self,
    ) // TODO -> return map type
    {
        // TODO Map generation =======
        // Chunk management prune old chunks
        // Calculate M
        // kNN fill
        // Chunk Management age chunks
    }

    pub fn get_q_m(&self) {
        // TODO
    }

    pub fn get_p_m(&self) {
        // TODO
    }

    pub fn get_v_m(&self) {
        // TODO
    }

    pub fn get_p(&self) {
        // TODO
    }

    pub fn get_v(&self) {
        // TODO
    }

    pub fn get_m(&self) {
        // TODO
    }
}



// Pruning Functions (START) --------------------------------------------------

// Pruning Functions (STOP) --------------------------------------------------



// Probabilistic Map Functions (START) --------------------------------------------------

// Probabilistic Map Functions (STOP) --------------------------------------------------



// Intensity Map Functions (START) --------------------------------------------------

// Intensity Map Functions (STOP) --------------------------------------------------



// Normalized Map Functions (START) --------------------------------------------------

// Normalized Map Functions (STOP) --------------------------------------------------



// Fill Inn Functions (START) --------------------------------------------------

// Fill Inn Functions (STOP) --------------------------------------------------



// Chunk Management Functions (START) --------------------------------------------------

// Chunk Management Functions (STOP) --------------------------------------------------













/*
! DELETE THIS LATER !
? For now this might come in useful
// Converts first-bottom-return slant range from meters into a sample/bin index.
//
// The conversion uses the slant-range resolution of the swath:
//     slant_resolution = max_range / samples_per_beam
//
// The result is clamped so it always stays inside the valid array bounds.
fn _range_to_bin(
    swath: &SwathProcessed,
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

// Slant Range Correction Functions (START) --------------------------------------------------
// Converts both sonar channels from slant range to ground range.
//
// Why this is needed:
// After blind-zone removal, the remaining samples still represent distance
// along the acoustic path of the sonar beam. That is useful for raw sensing,
// but not ideal for later mapping, since equal bin spacing in slant range does
// not correspond to equal horizontal spacing on the seabed.
//
// This step projects each valid sample onto the seabed plane using the
// corrected transducer height estimated earlier. In practice, this reduces the
// across-track geometric distortion that appears when the swath is kept purely
// in beam coordinates.
//
// High-level algorithm:
// 1. Process port and starboard independently.
// 2. Read each channel in logical near->far order.
// 3. Convert each valid slant-range sample to horizontal ground range.
// 4. Place the projected sample into its corresponding ground-range bin.
// 5. Write the result back in the channel's native storage order.
//
// Note:
// This implementation currently uses a direct bin remap without interpolation.
// That means some ground bins may remain empty, but it keeps the method simple,
// fast, and close to the raw measured structure.
// Next phases of the sonar processing pipeline will do probabilistic interpolation
// to fill these gaps fast and accurately
fn _slant_to_ground(
    port: &[u8],
    starboard: &[u8],
    swath: &SwathProcessed,
    sonar: &SonarParams,
    geometric_correction_data: &GeometricCorrection,
) -> (Vec<u8>, Vec<u8>) {
    let port = _collect_ground_samples(
        port,
        geometric_correction_data.h_port,
        sonar.transducer_port.max_range,
        sonar.transducer_port.is_reversed,
        sonar.transducer_port.blind_zone_scale,
        swath,
    );
    let starboard = _collect_ground_samples(
        starboard,
        geometric_correction_data.h_stb,
        sonar.transducer_stb.max_range,
        sonar.transducer_stb.is_reversed,
        sonar.transducer_stb.blind_zone_scale,
        swath,
    );

    return (port, starboard);
}

// Projects one channel from slant-range bins into ground-range bins.
//
// Why this is needed:
// A sonar channel is sampled uniformly in slant range, but after projection to
// the seabed the corresponding horizontal ground positions are no longer the
// same as the original beam positions. This function remaps each valid sample
// to the ground-range bin where it physically belongs.
//
// Algorithm:
// 1. Walk through the channel in logical near->far order.
// 2. Skip samples that were already masked out by blind-zone removal.
// 3. Convert the current bin index into physical slant range.
// 4. Project that slant range to horizontal ground range using flat-seabed geometry.
// 5. Convert the ground range back to a discrete output bin index.
// 6. Write the intensity into that projected output bin.
//
// Notes:
// - The output keeps the same number of bins as the input.
// - The channel is written back in its native storage order.
// - No interpolation is done here; empty gaps are therefore expected.
// - If multiple projected samples land in the same output bin, the strongest
//   intensity is kept.
fn _collect_ground_samples(
    channel: &[u8],
    h_t_transducer: f64,
    max_range: f64,
    is_reversed: bool,
    blind_zone_scale: f64,
    swath: &SwathProcessed,
) -> Vec<u8> {
    let n_bins: usize = channel.len();
    let slant_resolution = max_range/swath.samples_per_beam as f64;

    let mut ground_channel = vec![0u8; n_bins];

    for slant_bin in 0..n_bins {
        let source_index = if is_reversed {
            n_bins - 1 - slant_bin
        } else {
            slant_bin
        };

        let intensity = channel[source_index];

        // Samples already masked by the blind-zone step are invalid for seabed
        // projection, so only nonzero returns are processed further.
        if intensity == 0 { continue; }

        // Convert the current discrete bin into physical slant range and project
        // it onto the seabed plane. Samples that still fall below the corrected
        // transducer height would produce invalid geometry and are skipped.
        //
        // NOTE:
        // The blind-zone scale is also used here as an empirical correction.
        // In practice, using the pure geometric slant bin directly tended to
        // over-expand the nadir / blind-zone region. Applying the same tuning
        // factor here gave a better match to the observed sonar image.
        let slant_range = slant_bin as f64 * slant_resolution/blind_zone_scale;
        if slant_range <= h_t_transducer { continue; }

        let ground_range = (slant_range * slant_range - h_t_transducer * h_t_transducer).sqrt();

        // Convert projected ground range back to the nearest output ground bin.
        // Any sample that lands outside the valid channel bounds is ignored.
        //
        // NOTE:
        // The same empirical scale factor is applied here as well. This is not a
        // purely geometric correction, but a practical tuning that currently
        // gives the best visual and spatial agreement with the real data.
        let ground_bin = _range_to_bin(
            swath,
            max_range,
            blind_zone_scale,
            ground_range,
        );
        if ground_bin >= n_bins { continue; }

        // Convert the logical near->far ground bin back into the channel's native
        // storage order before writing the projected intensity.
        let target_index = if is_reversed {
            n_bins - 1 - ground_bin
        } else {
            ground_bin
        };

        ground_channel[target_index] = ground_channel[target_index].max(intensity);
    }

    return ground_channel;
}
// Slant Range Correction Functions (STOP) --------------------------------------------------
*/
