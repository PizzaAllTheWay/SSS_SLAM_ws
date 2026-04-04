use std::f64::consts::PI;
use std::collections::{
    HashMap,
};
use nalgebra::{
    Vector2,
    Matrix2,
    Rotation2,
};
use statrs::distribution::{
    ContinuousCDF, 
    Normal,
};

use super::types::*;



#[allow(non_snake_case)]
pub struct MapGenerator {
    // TODO: Explain each data briefly here like 1-2 sentences max and what its used for.

    sonar: SonarParams,

    cell_map_m: CellMapM,

    map_resolution: f64,
    chunk_size: i64,
    chunk_max_age: u32,
    chunk_map: ChunkMap,

    beam_weight_threshold: f64,
    probabilistic_map_threshold: f64,

    rotation2_lookup_table: Rotation2LookupTable,
}



impl MapGenerator {
    pub fn new(
        sonar: SonarParams,
        map_resolution: f64,
        chunk_size: i64,
        chunk_max_age: u32,
        beam_weight_threshold: f64,
        probabilistic_map_threshold: f64,
    ) -> Self {
        // Use the largest sonar range across both transducers when building the rotation lookup-table.
        // This guarantees that the lookup-table angular resolution is good enough for 
        // both sonar sides while only having to build it once.
        let max_range_port = sonar.transducer_port.max_range;
        let max_range_stb = sonar.transducer_stb.max_range;
        let max_range_max = max_range_port.max(max_range_stb);

        Self {
            // TODO Add stuff here, no need to explain a lot here
            sonar,

            cell_map_m: CellMapM::new(),

            map_resolution,
            chunk_size,
            chunk_max_age,
            chunk_map: ChunkMap::new(),

            beam_weight_threshold,
            probabilistic_map_threshold,

            // Precompute a shared 2D rotation lookup-table once during initialization.
            // This is purely an optimization to avoid rebuilding beam-angle rotations
            // with repeated sin/cos calls during the hot pruning loop.
            rotation2_lookup_table: Rotation2LookupTable::new(
                map_resolution,
                max_range_max,
            ),
        }
    }

    #[allow(non_snake_case)]
    pub fn buffer_processed_swath_into_map(
        &mut self,
        pose: &Pose3D,
        geometric_correction: GeometricCorrection,
        swath_processed: SwathProcessed,
    ) -> bool {
        // Pruning stage ---------- 
        // Build and Prunes Q_m
        // In addition caches P_m for later use
        let (mut cell_map_m_port, mut cell_map_m_stb) = _prune(
            &pose,
            &geometric_correction,
            &self.sonar,
            self.map_resolution,
            self.beam_weight_threshold,
            self.probabilistic_map_threshold,
            &self.rotation2_lookup_table,
        );

        // Intensity Calculation ----------
        _calculate_V_m(
            &mut cell_map_m_port,
            &mut cell_map_m_stb,
            &pose,
            &geometric_correction,
            &swath_processed,
            &self.sonar,
            self.map_resolution,
        );

        // Merge Data ----------
        self.cell_map_m = _merge_cell_map_m(
            &cell_map_m_port,
            &cell_map_m_stb,
        );

        // Manage Chunks ----------
        // Add cell_map_m data to Q, P, V
        // In addition manage Chunk Map
        _chunk_manager_add(
            &self.cell_map_m,
            &mut self.chunk_map,
            self.chunk_size,
        );

        return true;
    }

    pub fn calculate_map(
        &mut self,
        pose: &Pose3D,
    ) -> Map {
        // Map generation ----------
        // TODO Map generation =======
        _chunk_manager_prune(
            &mut self.chunk_map,
            self.chunk_max_age,
        );

        #[allow(non_snake_case)]
        let M = _calculate_M(
            pose,
            &mut self.chunk_map,
            self.map_resolution,
            self.chunk_size,
        );

        // TODO: kNN fill
        
        _chunk_manager_age(&mut self.chunk_map);

        return M;
    }

    pub fn get_cell_map_m(&self) -> &CellMapM {
        return &self.cell_map_m;
    }

    pub fn get_chunk_map(&self) -> &ChunkMap {
        return &self.chunk_map;
    }
}



// Look Up Tables (START) --------------------------------------------------
// Precomputed lookup table of 2D rotation matrices used to avoid rebuilding
// a fresh rotation from sin/cos for every sampled beam angle inside the
// pruning loop.
//
// Why this exists:
// The pruning stage may evaluate millions of beam-angle samples. Constructing
// a new 2D rotation matrix for every one of those samples is expensive because
// it repeatedly calls trigonometric functions. To reduce that cost, we build
// a dense table of rotation matrices once and then, during pruning, simply
// snap each requested beam angle to the nearest precomputed rotation.
//
// What it stores:
// - angle_min / angle_max: the angular range covered by the table
// - angle_step: angular spacing between neighboring entries
// - rotations: precomputed 2D rotation matrices over that angular range
//
// Tradeoff:
// - denser table  -> better angular accuracy, more memory
// - coarser table -> less memory, more angular quantization/aliasing error
//
// This is purely an optimization structure. It does not change the model
// itself, only how efficiently the rotations are evaluated.
pub struct Rotation2LookupTable {
    angle_min: f64,
    angle_max: f64,
    angle_step: f64,
    rotations: Vec<Matrix2<f64>>,
}

#[allow(non_snake_case)]
impl Rotation2LookupTable {
    // Builds a lookup table of 2D rotation matrices over a fixed angle range.
    // angle_step controls the angular resolution of the table.
    pub fn new(
        map_resolution: f64,
        max_range: f64,
    ) -> Self {
        let angle_min = -PI;
        let angle_max = PI;

        // Refines the angular lookup-table (LUT) resolution beyond the nominal beam sampling step.
        // A value < 1.0 makes the LUT denser, which reduces angular quantization/aliasing error
        // when snapping requested beam angles to the nearest precomputed rotation.
        // This is purely an accuracy-vs-memory/performance tuning factor for the LUT.
        // It has been chosen to be 10x factor just to be on a safe side
        let angle_lookup_refinement_factor = 0.1;

        let angle_step = angle_lookup_refinement_factor * map_resolution/max_range.max(map_resolution);

        let mut rotations = Vec::new();

        let mut angle = angle_min;
        while angle <= angle_max {
            let (sin_a, cos_a) = angle.sin_cos();

            rotations.push(
                Matrix2::new(
                    cos_a, -sin_a,
                    sin_a,  cos_a,
                )
            );

            angle += angle_step;
        }

        Self {
            angle_min,
            angle_max,
            angle_step,
            rotations,
        }
    }

    // Returns the precomputed rotation matrix closest to the requested angle.
    pub fn R(
        &self,
        angle: f64,
    ) -> &Matrix2<f64> {
        let clamped_angle = angle.clamp(self.angle_min, self.angle_max);

        let idx = ((clamped_angle - self.angle_min) / self.angle_step).round() as usize;
        let idx = idx.min(self.rotations.len().saturating_sub(1));

        &self.rotations[idx]
    }
}
// Look Up Tables (STOP) --------------------------------------------------



// Pruning Functions (START) --------------------------------------------------
// Builds the temporary measurement cell maps for the current ping,
// separately for port and starboard.
//
// This is the geometric + probabilistic pruning stage of the local map pipeline.
// Its job is to find which discrete map cells from the current measurement m
// are plausible candidates for illumination, while discarding unlikely samples
// as early as possible.
//
// Each sonar side is processed independently because port and starboard have
// different first-bottom-return ranges, mounting directions, and beam geometry.
// The result is one temporary sparse cell map per side, ready to be merged
// into a single measurement map for the current ping.
#[allow(non_snake_case)]
fn _prune(
    pose: &Pose3D,
    geometric_correction: &GeometricCorrection,
    sonar: &SonarParams,
    map_resolution: f64,
    beam_weight_threshold: f64,
    probabilistic_map_threshold: f64,
    rotation2_lookup_table: &Rotation2LookupTable,
) -> (CellMapM, CellMapM) {
    let cell_map_m_port = _prune_side(
        pose,
        geometric_correction.r_fbr_port,
        sonar.transducer_port.max_range,
        &sonar.transducer_port.offset,
        sonar.transducer_port.theta,
        sonar.transducer_port.blind_zone_scale,
        map_resolution,
        beam_weight_threshold,
        probabilistic_map_threshold,
        rotation2_lookup_table,
    );

    let cell_map_m_stb = _prune_side(
        pose,
        geometric_correction.r_fbr_stb,
        sonar.transducer_stb.max_range,
        &sonar.transducer_stb.offset,
        sonar.transducer_stb.theta,
        sonar.transducer_stb.blind_zone_scale,
        map_resolution,
        beam_weight_threshold,
        probabilistic_map_threshold,
        rotation2_lookup_table,
    );

    return (cell_map_m_port, cell_map_m_stb);
}

// Builds the temporary sparse measurement cell map for one sonar side
// (either port or starboard) for the current ping.
//
// The function sweeps through the side-scan sonar footprint from the corrected
// first-bottom-return range to the maximum range, and across the horizontal
// beam opening angle. Each sampled beam point is pruned in two stages:
// first by a cheap approximate beam-weight check, and then by the more exact
// integrated measurement probability. Surviving samples are transformed into
// world/map coordinates, discretized to map cells, and stored in CellMapM.
//
// If multiple beam samples land in the same discrete map cell, only one cell
// entry is kept and the strongest probability for that cell is preserved.
#[allow(non_snake_case)]
fn _prune_side(
    pose: &Pose3D,
    r_fbr: f64,
    r_max: f64,
    transducer_offset: &Pose3D,
    theta: f64,
    blind_zone_scale: f64,
    map_resolution: f64,
    beam_weight_threshold: f64,
    probabilistic_map_threshold: f64,
    rotation2_lookup_table: &Rotation2LookupTable,
) -> CellMapM {
    // Store discrete map cells together with their data.
    // Key = discrete map index
    // Value = cell data for that map pixel
    let mut cell_map_m = CellMapM::new();

    // The sonar beam spans sideways around its centerline.
    // We therefore sweep from -theta/2 to +theta/2 around the side-looking direction.
    let half_theta = theta/2.0;

    // Determine whether this transducer points to positive body-y or negative body-y.
    // Port and starboard are therefore handled with the same logic, only sign changes.
    let side_sign = if transducer_offset.position.y >= 0.0 {
        1.0
    } else {
        -1.0
    };

    // Precompute vehicle yaw rotation in 2D once for this ping.
    // Since Q_m is generated only in the horizontal ground plane, a 2D rotation
    // is sufficient and avoids the overhead of full 3D rotation objects.
    // ? NOTE: Had to offset by PI/2 because the Yaw angle mounting is weird, might have to redo this a bit or look into it later if time allows, not ideal :/
    let R_body_to_world = Rotation2::new(pose.orientation.yaw - PI/2.0);

    // Vehicle/body origin in world coordinates.
    // Each beam point is rotated into world frame and then translated by this.
    let pose_body_in_world = Vector2::new(
        pose.position.x,
        pose.position.y,
    );

    // Sweep outward in range from first-bottom-return to max range.
    // For each range shell, sweep across the beam width.
    // This naturally fills the curved illuminated sector.
    //
    // The first-bottom-return range is scaled here by blind_zone_scale for the
    // same practical reason as in the swath-processing step: the pure geometric
    // first-bottom-return estimate tends to place the blind-zone boundary too far
    // out in real data. Applying this empirical correction makes the inner edge
    // of the candidate illuminated region better match the actual usable sonar data.
    let mut range_m = r_fbr * blind_zone_scale;

    while range_m <= r_max {
        let mut local_beam_angle = -half_theta;

        // Beam center for the current range shell in body-ground coordinates.
        // This is the side-looking centerline before applying the local beam angle.
        let beam_center_body_ground = Vector2::new(
            0.0,
            side_sign * range_m,
        );

        while local_beam_angle <= half_theta {
            // ? NOTE: This is where optimization is VERY important
            // ? NOTE: Here depending on the situation we might have to prune through 10 000 000 + pixels
            // ? NOTE: Because of this, some code clarity is discarded for the sake of optimization magic

            // Rotate from the beam centerline to the current beam angle inside the sector.
            // A precomputed lookup-table rotation is used here instead of constructing a
            // fresh Rotation2 every time, to avoid repeated sin/cos calls in this hot loop.
            #[allow(non_snake_case)]
            let R_beam = rotation2_lookup_table.R(local_beam_angle);

            let beam_point_body_ground = R_beam * beam_center_body_ground;

            // ? NOTE: At this point we will have 10 000 000+ pixels still
            // ? NOTE: Because of this we will have to do pruning here before insertion
            // ? NOTE: We will calculate the probability this certain sample point is illuminated and we will drop those bellow a certain threshold
            // ? NOTE: This will drastically decrease amount of pixels inserted into the map

            // Angular beam pruning:
            // Compute a fast approximate beam weight for this sampled point.
            // If the point lies too far toward the beam edge, skip it immediately
            // before doing more expensive work.
            let p_theta_weight = _p_theta_approximation(
                local_beam_angle,
                half_theta,
            );

            if p_theta_weight < beam_weight_threshold {
                // Angular step:
                // use a range-dependent step so the arc is sampled roughly at map resolution.
                // This avoids huge oversampling near r_min and undersampling near r_max.
                local_beam_angle += map_resolution/range_m.max(map_resolution);
                continue;
            }

            // Exact probabilistic beam pruning:
            // The sample survived the cheap angular beam-weight gate, so now we
            // evaluate the integrated measurement probability over the small
            // angular interval represented by this sample. If that probability
            // is still too small, skip the sample before doing world transform
            // and map insertion.
            let P_m = _P_m(
                local_beam_angle,
                half_theta,
                map_resolution,
                range_m,
            );

            if P_m < probabilistic_map_threshold {
                // Angular step:
                // use a range-dependent step so the arc is sampled roughly at map resolution.
                // This avoids huge oversampling near r_min and undersampling near r_max.
                local_beam_angle += map_resolution/range_m.max(map_resolution);
                continue;
            }

            // Transform the beam point from body-ground frame into world frame.
            let beam_point_world_ground = R_body_to_world * beam_point_body_ground + pose_body_in_world;

            // Convert world position into discrete map cell index.
            // This is the actual q pixel index used in the set.
            let (qx, qy) = _discretize_to_q(
                beam_point_world_ground.x,
                beam_point_world_ground.y,
                map_resolution,
            );

            // If multiple beam samples land in the same discrete map cell,
            // keep only one cell entry for that location and preserve the
            // strongest measurement probability seen so far.
            let cell_coord_m = CellCoordM {
                x_m: qx,
                y_m: qy,
            };

            cell_map_m
                .map_m
                .entry(cell_coord_m)
                .and_modify(|cell| {
                    cell.p_m = cell.p_m.max(P_m);
                })
                .or_insert(
                    CellDataM {
                        q_m: QPixel {
                            x: qx as f64 * map_resolution,
                            y: qy as f64 * map_resolution,
                        },
                        v_m: 0.0,
                        p_m: P_m,
                    }
                );

            // Angular step:
            // use a range-dependent step so the arc is sampled roughly at map resolution.
            // This avoids huge oversampling near r_min and undersampling near r_max.
            local_beam_angle += map_resolution/range_m.max(map_resolution);
        }

        // Radial step:
        // move outward roughly one map pixel at a time.
        range_m += map_resolution;
    }

    return cell_map_m;
}

// Converts a continuous world coordinate (x, y) into a discrete map grid index (qx, qy).
// The map is assumed to be a uniform grid with cell size = map_resolution.
// Each returned (qx, qy) represents the index of the pixel/cell in that grid.
// This is used to ensure all points snap to consistent discrete locations.
fn _discretize_to_q(
    x: f64,
    y: f64,
    map_resolution: f64,
) -> (i64, i64) {
    let qx = (x/map_resolution).round() as i64;
    let qy = (y/map_resolution).round() as i64;

    return (qx, qy);
}
// Pruning Functions (STOP) --------------------------------------------------



// Probabilistic Map Functions (START) --------------------------------------------------
// Returns an approximate measurement probability for the current sampled point
// inside the sonar beam.
//
// This is NOT the full probabilistic map-cell model yet.
// The true P_m(q) should be computed as the integral of p(theta) over the
// angular extent of the map cell q:
//
//     P_m(q) = ∫ p(theta) dtheta
//
// However, doing that full integral for every sampled point/cell during pruning stage would be too expensive.
//
// So for now, we use a fast pointwise approximation:
// - treat the current sampled point as if it represents the cell
// - evaluate the beam-density at its local beam angle
// - use that value as an approximate P_m(q)
//
// This is good enough for aggressive early pruning.
#[allow(non_snake_case)]
fn _P_m_approximation(
    local_beam_angle: f64,
    beam_angle_horizontal_half: f64,
) -> f64 {
    return _p_theta_approximation(
        local_beam_angle,
        beam_angle_horizontal_half,
    );
}

// Returns a fast approximate angular beam-weight p_theta for a given local beam angle.
//
// Why this approximation is used:
// The original Gaussian beam model uses an exponential, which is too expensive
// when evaluated millions of times during pruning. Since this stage only needs
// a cheap center-vs-edge weighting for early rejection, we replace the full
// Gaussian with a much faster polynomial approximation.
//
// Method:
// 1. Normalize the local beam angle by the half horizontal beam width.
// 2. This gives a dimensionless value u where:
//      u = 0   -> beam center
//      |u| = 1 -> beam edge
// 3. Use the squared normalized angle to build a simple center-heavy weight.
//
// The approximation used is:
//     p_theta_approx = (1 - u^2)^2,   for |u| < 1
//     p_theta_approx = 0,             otherwise
//
// This keeps the same general behavior we want for pruning:
// - strongest at the beam center
// - smoothly decreasing toward the edges
// - zero outside the beam
//
// It is NOT a physically exact beam-density model.
// It is only a fast approximation for aggressive early pruning.
fn _p_theta_approximation(
    theta: f64,
    alpha_h_half: f64,
) -> f64 {
    let u = (theta / alpha_h_half).abs();

    if u >= 1.0 {
        return 0.0;
    }

    let v = 1.0 - u * u;
    let p_theta_approx = v * v;

    return p_theta_approx;
}

// Returns the integrated measurement probability P_m(q) for the current sampled point.
//
// This function is the main probabilistic beam model used after the cheap
// angular pruning stage. Unlike the fast approximation, it does not evaluate
// only a single point value. Instead, it approximates the current sampled point
// as representing a small angular interval and integrates the Gaussian beam
// model over that interval.
//
// What it does:
// 1. Estimate the angular width represented by the current sample using the
//    current map resolution and range:
//        delta_theta ≈ map_resolution / range_m
// 2. Build the angular interval around the sampled beam angle:
//        [theta_min, theta_max]
// 3. Model the horizontal sonar beam as a zero-mean Gaussian in angle space,
//    with standard deviation:
//        sigma = alpha_h / 2
// 4. Compute the integrated probability mass over that interval using the
//    Gaussian CDF:
//
//        P_m(q) = ∫ p(theta) dtheta
//               = CDF(theta_max) - CDF(theta_min)
//
// Why this is useful:
// - it is much more meaningful than a single pointwise beam value
// - it better matches the idea that each sampled point corresponds to a small
//   finite angular region, not an infinitely thin ray
//
// Note:
// This is still an approximation of the final cell probability, since the true
// map cell footprint may not match this angular interval exactly. But it is a
// good practical probabilistic estimate for the current pruning/mapping stage.
#[allow(non_snake_case)]
fn _P_m(
    local_beam_angle: f64,
    alpha_h_half: f64,
    map_resolution: f64,
    range_m: f64,
) -> f64 {
    let delta_theta = map_resolution / range_m.max(map_resolution);

    let theta_min = local_beam_angle - 0.5 * delta_theta;
    let theta_max = local_beam_angle + 0.5 * delta_theta;

    let sigma = alpha_h_half.max(1e-12);
    let normal = Normal::new(0.0, sigma).unwrap();

    let P_m = (normal.cdf(theta_max) - normal.cdf(theta_min)).max(0.0);

    return P_m;
}
// Probabilistic Map Functions (STOP) --------------------------------------------------



// Intensity Map Functions (START) --------------------------------------------------
// TODO: Explain what the function does
#[allow(non_snake_case)]
fn _calculate_V_m(
    cell_map_m_port: &mut CellMapM,
    cell_map_m_stb: &mut CellMapM,
    pose: &Pose3D,
    geometric_correction: &GeometricCorrection,
    swath_processed: &SwathProcessed,
    sonar: &SonarParams,
    map_resolution: f64,
) {
    _calculate_V_m_side(
        cell_map_m_port,
        pose,
        geometric_correction.h_port,
        &swath_processed.port,
        swath_processed.samples_per_beam,
        &sonar.transducer_port.offset,
        sonar.transducer_port.max_range,
        sonar.transducer_port.is_reversed,
        map_resolution,
    );

    _calculate_V_m_side(
        cell_map_m_stb,
        pose,
        geometric_correction.h_stb,
        &swath_processed.starboard,
        swath_processed.samples_per_beam,
        &sonar.transducer_stb.offset,
        sonar.transducer_stb.max_range,
        sonar.transducer_stb.is_reversed,
        map_resolution,
    );
}

// TODO: Reexlain what the function does
// Builds the temporary intensity map V_m for the current measurement m.
//
// Idea:
// For each candidate map cell q_m that survived pruning, estimate its intensity
// by projecting that cell back into sonar slant space and sampling the processed
// swath image. Instead of using only the cell center, use the 4 cell corners,
// interpolate each corner intensity in slant space, and average them to get
// one representative intensity value for the cell.
//
// High-level steps:
// 1. Loop through all surviving cells in cell_map_m.
// 2. For each cell, compute its 4 map-space corner coordinates.
// 3. For each corner, project that corner into the corresponding sonar slant range.
// 4. Convert each projected slant range into neighboring sonar sample/bin indices.
// 5. Interpolate between the lower/upper neighboring bins to estimate corner intensity.
// 6. Average the 4 interpolated corner intensities to get V_m for that cell.
// 7. Store the result back into the cell's v_m field.
//
// Notes:
// - This should be done separately for port and starboard before final merge,
//   because the projection into slant space depends on sonar side.
// - The actual projection helper should decide whether a map cell belongs to
//   the current sonar side and whether the projected slant range is valid.
// - For now this is only a skeleton / TODO guide.
#[allow(non_snake_case)]
fn _calculate_V_m_side(
    cell_map_m: &mut CellMapM,
    pose: &Pose3D,
    h_transducer: f64,
    swath_processed_channel: &[u8],
    swath_samples: u64,
    transducer_offset: &Pose3D,
    max_range: f64,
    is_reversed: bool,
    map_resolution: f64,
) {
    // Vehicle/body origin in world coordinates.
    // Each beam point is rotated into world frame and then translated by this.
    let pose_body_in_world = Vector2::new(
        pose.position.x,
        pose.position.y,
    );

    // Precompute vehicle yaw rotation in 2D once for this ping.
    // Since Q_m is generated only in the horizontal ground plane, a 2D rotation
    // is sufficient and avoids the overhead of full 3D rotation objects.
    // ? NOTE: Had to offset by PI/2 because the Yaw angle mounting is weird, might have to redo this a bit or look into it later if time allows, not ideal :/
    let R_body_to_world = Rotation2::new(pose.orientation.yaw - PI/2.0);
    let R_world_to_body = R_body_to_world.transpose();

    // Prep the half cell in advance so no extra calculations for optimized code
    let half_cell = map_resolution/2.0;

    for cell_data_m in cell_map_m.map_m.values_mut() {
        // Get the world coordinate
        let q_m = &cell_data_m.q_m;
        let range_world_m = Vector2::new(
            q_m.x,
            q_m.y,
        );

        // Build the 4 cell corners in world/map frame first, then rotate them into body frame.
        // This is the correct order because the map cell is axis-aligned in world space, not in body space.
        let range_corners_world = [
            Vector2::new(
                range_world_m.x - half_cell,
                range_world_m.y - half_cell,
            ),
            Vector2::new(
                range_world_m.x + half_cell,
                range_world_m.y - half_cell,
            ),
            Vector2::new(
                range_world_m.x + half_cell,
                range_world_m.y + half_cell,
            ),
            Vector2::new(
                range_world_m.x - half_cell,
                range_world_m.y + half_cell,
            ),
        ];

        let mut range_corners_body = [Vector2::zeros(); 4];
        for i in 0..4 {
            range_corners_body[i] =
                R_world_to_body * (range_corners_world[i] - pose_body_in_world);
        }

        // Compute slant range
        // Precompute body -> sonar transform
        let R_body_to_sonar = Rotation2::new(transducer_offset.orientation.yaw);
        let t_body_to_sonar = Vector2::new(
            transducer_offset.position.x,
            transducer_offset.position.y,
        );

        let mut slant_ranges = [0.0; 4];
        for i in 0..4 {
            // body -> sonar frame
            let corner_sonar = R_body_to_sonar * (range_corners_body[i] - t_body_to_sonar);

            // horizontal distance in sonar frame
            let ground_range = (corner_sonar.x * corner_sonar.x + corner_sonar.y * corner_sonar.y).sqrt();

            // slant range using height
            slant_ranges[i] = (ground_range * ground_range + h_transducer * h_transducer).sqrt();
        }

        // Compute intensities
        let slant_resolution = max_range/swath_samples as f64;

        let mut intensities = [0.0; 4];

        for i in 0..4 {
            // Get the bin indexes for upper and lower bin
            let bin_f = slant_ranges[i]/slant_resolution;

            let bin_low = bin_f.floor() as usize;
            let bin_high = bin_low + 1;

            // Calculate the weight for interpolation
            let w = bin_f - bin_low as f64;

            // Calculate index order depending on inversion
            let idx_low = if is_reversed {
                (swath_samples as usize - 1).saturating_sub(bin_low)
            } else {
                bin_low
            };

            let idx_high = if is_reversed {
                (swath_samples as usize - 1).saturating_sub(bin_high)
            } else {
                bin_high
            };

            // Safety clamp so no indexes are out of bound
            let idx_low = idx_low.min(swath_samples as usize - 1);
            let idx_high = idx_high.min(swath_samples as usize - 1);

            // Extract intensities
            let v0 = swath_processed_channel[idx_low] as f64;
            let v1 = swath_processed_channel[idx_high] as f64;

            intensities[i] = (1.0 - w) * v0 + w * v1;
        }

        // average the 4 corner intensities to get one V_m value for this cell
        let V_m = intensities.iter().sum::<f64>()/4.0;

        // write the averaged value into cell_data_m.v_m
        cell_data_m.v_m = V_m
    }
}
// Intensity Map Functions (STOP) --------------------------------------------------



// Merge Data Functions (START) --------------------------------------------------
// Merges the port and starboard temporary measurement cell maps into one
// combined measurement map for the current ping.
//
// If both sonar sides land in the same discrete map cell, keep one merged cell.
// For now:
// - q_m is the same map location
// - p_m keeps the strongest probability
// - v_m keeps the strongest intensity value
#[allow(non_snake_case)]
fn _merge_cell_map_m(
    cell_map_m_port: &CellMapM,
    cell_map_m_stb: &CellMapM,
) -> CellMapM {
    let mut cell_map_m = CellMapM {
        map_m: HashMap::with_capacity(
            cell_map_m_port.map_m.len() + cell_map_m_stb.map_m.len()
        ),
    };

    for (cell_coord_m, cell_data_m) in &cell_map_m_port.map_m {
        cell_map_m.map_m.insert(
            *cell_coord_m,
            CellDataM {
                q_m: QPixel {
                    x: cell_data_m.q_m.x,
                    y: cell_data_m.q_m.y,
                },
                v_m: cell_data_m.v_m,
                p_m: cell_data_m.p_m,
            },
        );
    }

    for (cell_coord_m, cell_data_m) in &cell_map_m_stb.map_m {
        cell_map_m
            .map_m
            .entry(*cell_coord_m)
            .and_modify(|cell| {
                cell.p_m = cell.p_m.max(cell_data_m.p_m);
                cell.v_m = cell.v_m.max(cell_data_m.v_m);
            })
            .or_insert(
                CellDataM {
                    q_m: QPixel {
                        x: cell_data_m.q_m.x,
                        y: cell_data_m.q_m.y,
                    },
                    v_m: cell_data_m.v_m,
                    p_m: cell_data_m.p_m,
                },
            );
    }

    return cell_map_m;
}
// Merge Data Functions (STOP) --------------------------------------------------



// Normalized Map Functions (START) --------------------------------------------------
// TODO: Rewrite a bit with Pose in mind as well
// Builds the dense final map image M from the sparse chunk map.
//
// The output is a dense 2D pixel grid of type u8 where each pixel is computed as:
//
//     M = V / P
//
// for cells that exist in the chunk map and have nonzero probability.
// Missing chunks / missing cells / zero-probability cells are written as 0.
//
// Output layout:
// - M[row][col]
// - row corresponds to y
// - col corresponds to x
#[allow(non_snake_case)]
pub fn _calculate_M(
    pose: &Pose3D,
    chunk_map: &ChunkMap,
    map_resolution: f64,
    chunk_size: i64,
) -> Map {
    if chunk_map.chunks.is_empty() {
        return Map::new(
            Pose2DMap { 
                x: 0,
                y: 0,
                yaw: pose.orientation.yaw
            },
            0,
            0,
        );
    }

    // Global chunk bounds
    let min_chunk_x = chunk_map.chunks.keys().map(|c| c.x).min().unwrap();
    let max_chunk_x = chunk_map.chunks.keys().map(|c| c.x).max().unwrap();
    let min_chunk_y = chunk_map.chunks.keys().map(|c| c.y).min().unwrap();
    let max_chunk_y = chunk_map.chunks.keys().map(|c| c.y).max().unwrap();

    // Convert chunk bounds to global cell bounds
    let min_cell_x = min_chunk_x * chunk_size;
    let max_cell_x = (max_chunk_x + 1) * chunk_size;
    let min_cell_y = min_chunk_y * chunk_size;
    let max_cell_y = (max_chunk_y + 1) * chunk_size;

    let width = (max_cell_x - min_cell_x) as usize;
    let height = (max_cell_y - min_cell_y) as usize;

    let pose_global_x = (pose.position.x/map_resolution).round() as i64;
    let pose_global_y = (pose.position.y/map_resolution).round() as i64;

    let pose_map = Pose2DMap {
        x: pose_global_x - min_cell_x,
        y: pose_global_y - min_cell_y,
        yaw: pose.orientation.yaw,
    };

    let mut M = Map::new(pose_map, width, height);

    for row in 0..height {
        for col in 0..width {
            let global_x = min_cell_x + col as i64;
            let global_y = min_cell_y + row as i64;

            let chunk_coord = ChunkCoord {
                x: global_x.div_euclid(chunk_size),
                y: global_y.div_euclid(chunk_size),
            };

            let local_cell_coord = CellCoord {
                x: global_x.rem_euclid(chunk_size),
                y: global_y.rem_euclid(chunk_size),
            };

            let value = if let Some(chunk) = chunk_map.chunks.get(&chunk_coord) {
                if let Some(cell) = chunk.data.get(&local_cell_coord) {
                    if cell.p > 0.0 {
                        let m = cell.v/cell.p;
                        m.clamp(0.0, 255.0) as u8
                    } else {
                        0 // Probability is ill defined, set value to 0 for this pixel
                    }
                } else {
                    0 // Cell didn't exist, set value to 0 for this pixel
                }
            } else {
                0 // Chunk didn't exist, set value to 0 for this pixel
            };

            M.set(col, row, value);
        }
    }

    M
}
// Normalized Map Functions (STOP) --------------------------------------------------



// Fill Inn Functions (START) --------------------------------------------------

// Fill Inn Functions (STOP) --------------------------------------------------



// Chunk Management Functions (START) --------------------------------------------------
// Adds the current measurement cell map into the persistent chunk map.
//
// For each cell from the current measurement m:
// 1. find which chunk it belongs to,
// 2. find the local cell coordinate inside that chunk,
// 3. allocate the chunk if needed,
// 4. reset chunk age to 0 because it was touched this round,
// 5. allocate the cell if needed,
// 6. accumulate probability and probability-weighted intensity:
//
//      P += P_m
//      V += P_m * V_m
//
#[allow(non_snake_case)]
pub fn _chunk_manager_add(
    cell_map_m: &CellMapM,
    chunk_map: &mut ChunkMap,
    chunk_size: i64,
) {
    for (cell_coord_m, cell_data_m) in &cell_map_m.map_m {
        let chunk_coord = ChunkCoord {
            x: cell_coord_m.x_m.div_euclid(chunk_size),
            y: cell_coord_m.y_m.div_euclid(chunk_size),
        };

        let local_cell_coord = CellCoord {
            x: cell_coord_m.x_m.rem_euclid(chunk_size),
            y: cell_coord_m.y_m.rem_euclid(chunk_size),
        };

        let chunk = chunk_map
            .chunks
            .entry(chunk_coord)
            .or_insert_with(|| Chunk {
                age: 0,
                data: HashMap::new(),
            });

        chunk.age = 0;

        let cell = chunk
            .data
            .entry(local_cell_coord)
            .or_insert(CellData {
                v: 0.0,
                p: 0.0,
            });

        cell.p += cell_data_m.p_m;
        cell.v += cell_data_m.p_m * cell_data_m.v_m;
    }
}

// Removes/deallocates old chunks whose age has exceeded the allowed limit.
//
// Any chunk with age > max_age is deleted completely from the chunk map,
// including all cell data stored inside that chunk.
#[allow(non_snake_case)]
pub fn _chunk_manager_prune(
    chunk_map: &mut ChunkMap,
    max_age: u32,
) {
    chunk_map
        .chunks
        .retain(|_, chunk| chunk.age <= max_age);
}

// Increments the age of every active chunk by 1.
//
// This should typically be called once per map-update cycle after all chunks
// that touched in the current round have been reset to age = 0.
#[allow(non_snake_case)]
pub fn _chunk_manager_age(
    chunk_map: &mut ChunkMap,
) {
    for chunk in chunk_map.chunks.values_mut() {
        chunk.age += 1;
    }
}
// Chunk Management Functions (STOP) --------------------------------------------------













/*
// !
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
