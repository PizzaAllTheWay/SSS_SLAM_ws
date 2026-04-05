// ? NOTE: Use FxHashMap here instead of the default std HashMap because this code is
// ? performance-critical and does a very large number of hashmap operations on internal,
// ? non-hostile numeric keys. FxHashMap uses a faster non-cryptographic hasher, which
// ? reduces hashing overhead and improves runtime in hot paths like map/chunk/cell access.
use rustc_hash::{
    FxHashMap,
};

use std::f64::consts::PI;
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
    // Fixed sonar configuration used throughout pruning, projection, and intensity lookup.
    sonar: SonarParams,

    // Temporary measurement cell map for the current swath after port/stb merge.
    cell_map_m: CellMapM,

    // Dense map resolution in meters per pixel.
    map_resolution: f64,

    // Persistent sparse chunked map storing accumulated probabilistic map data over time.
    chunk_map: ChunkMap,

    // Fast early pruning threshold using the cheap beam approximation.
    beam_weight_threshold: f64,
    // Main probabilistic pruning threshold using the more exact beam model.
    probabilistic_map_threshold: f64,

    // Precomputed 2D rotation lookup table used to speed up beam-angle rotations.
    rotation2_lookup_table: Rotation2LookupTable,

    // Minimum number of valid neighboring pixels required before filling an empty pixel.
    fill_inn_min_neighbors: u8,
    // Maximum number of iterative local fill rounds used for small gap filling.
    fill_inn_passes: u8,

    // ? NOTE: `map_offset_yaw` is currently applied as a fixed alignment correction
    // ? between the pose yaw convention and the map/sonar ground-plane convention.
    // ? This works for the current setup, but ideally the underlying frame definition
    // ? should be made fully consistent so this extra offset is no longer needed.
    map_offset_yaw: f64,
}
impl MapGenerator {
    pub fn new(
        sonar: SonarParams,
        map_resolution: f64,
        chunk_size: i64,
        chunk_max_age: u32,
        beam_weight_threshold: f64,
        probabilistic_map_threshold: f64,
        fill_inn_min_neighbors: u8,
        fill_inn_passes: u8,
        map_offset_yaw: f64,
    ) -> Self {
        // Use the largest sonar range across both transducers when building the rotation lookup-table.
        // This guarantees that the lookup-table angular resolution is good enough for 
        // both sonar sides while only having to build it once.
        let max_range_port = sonar.transducer_port.max_range;
        let max_range_stb = sonar.transducer_stb.max_range;
        let max_range_max = max_range_port.max(max_range_stb);

        Self {
            sonar,

            cell_map_m: CellMapM::new(),

            map_resolution,

            chunk_map: ChunkMap::new(
                chunk_size,
                chunk_max_age
            ),

            beam_weight_threshold,
            probabilistic_map_threshold,

            // Precompute a shared 2D rotation lookup-table once during initialization.
            // This is purely an optimization to avoid rebuilding beam-angle rotations
            // with repeated sin/cos calls during the hot pruning loop.
            rotation2_lookup_table: Rotation2LookupTable::new(
                map_resolution,
                max_range_max,
            ),

            fill_inn_min_neighbors,
            fill_inn_passes,

            map_offset_yaw,
        }
    }

    // Buffers one processed swath into the probabilistic map state.
    // The function prunes candidate map cells, estimates their measurement intensity,
    // merges port and starboard results, and then inserts the final measurement cells
    // into the persistent chunk map representation.
    #[allow(non_snake_case)]
    pub fn buffer_processed_swath_into_map(
        &mut self,
        pose: &Pose3D,
        geometric_correction: GeometricCorrection,
        swath_processed: SwathProcessed,
    ) {
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
            self.map_offset_yaw,
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
            self.map_offset_yaw,
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
        );
    }

    // Builds the current dense map from the persistent chunk map state.
    // Old chunks are pruned first, then the sparse chunk data is converted into a
    // dense normalized map image, small local gaps are filled, and finally the
    // remaining active chunks are aged for future chunk management.
    pub fn calculate_map(
        &mut self,
        pose: &Pose3D,
    ) -> Map {
        // Map generation ----------
        _chunk_manager_prune(&mut self.chunk_map);

        #[allow(non_snake_case)]
        let mut M = _calculate_M(
            pose,
            &self.chunk_map,
            self.map_resolution,
        );

        _fill_small_gaps(
            &mut M,
            &self.chunk_map,
            self.fill_inn_min_neighbors,
            self.fill_inn_passes,
        );
        
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
    inv_angle_step: f64,
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
        let inv_angle_step = 1.0/angle_step;

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
            inv_angle_step,
            rotations,
        }
    }

    // Returns the precomputed rotation matrix closest to the requested angle.
    // ? NOTE:
    // ? PERFORMANCE CRITICAL:
    // ? This function assumes `angle` is always in [angle_min, angle_max].
    // ? No clamp or bounds checks are done here.
    // ? If that contract is violated, behavior is invalid.
    // ? However this should never be violated as it is internal function for a very specific optiization case
    // ? In that specific case we already have angle under control and is already bounded where 
    #[inline(always)]
    pub fn R(&self, angle: f64) -> &Matrix2<f64> {
        let idx = ((angle - self.angle_min) * self.inv_angle_step + 0.5) as usize;
        unsafe { self.rotations.get_unchecked(idx) }
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
    map_offset_yaw: f64,
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
        map_offset_yaw,
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
        map_offset_yaw,
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
    map_offset_yaw: f64,
) -> CellMapM {
    // Store discrete map cells together with their data.
    // Key = discrete map index
    // Value = cell data for that map pixel
    let mut cell_map_m = CellMapM::new();

    // The sonar beam spans sideways around its centerline.
    // We therefore sweep from -theta/2 to +theta/2 around the side-looking direction.
    let half_theta = theta/2.0;
    let half_theta_inv = 1.0/half_theta; // Purely for optimization so we don't have to divide each time we want inverse value

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
    // ? NOTE: `map_offset_yaw` is currently applied as a fixed alignment correction
    // ? between the pose yaw convention and the map/sonar ground-plane convention.
    // ? This works for the current setup, but ideally the underlying frame definition
    // ? should be made fully consistent so this extra offset is no longer needed.
    let R_body_to_world = Rotation2::new(pose.orientation.yaw + map_offset_yaw);

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
            let P_m_approximation = _P_m_approximation(
                local_beam_angle,
                half_theta_inv,
            );

            if P_m_approximation < beam_weight_threshold {
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



// Probabilistic Map Functions (START) -------------------------------------------------
// Returns a very fast approximate measurement probability P_m for the current
// sampled point inside the sonar beam.
//
// This is used only for early pruning, where speed matters more than physical accuracy.
// Instead of evaluating the full probabilistic beam model, this function uses a cheap
// polynomial approximation that is strongest at the beam center, smoothly decreases
// toward the beam edges, and becomes zero outside the beam.
//
// Method:
// 1. Normalize the local beam angle by the half horizontal beam width.
// 2. Square the normalized value to avoid calling `abs()`.
// 3. Use the simple center-heavy approximation:
//
//     P_m_approx = (1 - u^2)^2,   for u^2 < 1
//     P_m_approx = 0,             otherwise
//
// This is not the final physical probability model.
// It is only a fast approximation used to reject unlikely samples early.
#[allow(non_snake_case)]
#[inline(always)]
fn _P_m_approximation(
    local_beam_angle: f64,
    beam_angle_horizontal_half_inv: f64,
) -> f64 {
    let u = local_beam_angle * beam_angle_horizontal_half_inv;
    let u2 = u * u;

    if u2 >= 1.0 {
        return 0.0;
    }

    let v = 1.0 - u2;
    v * v
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
// Computes the measurement intensity V_m for all surviving candidate cells.
// It does this separately for the port and starboard channels, because each
// side has its own transducer geometry, usable range, and processed swath data.
#[allow(non_snake_case)]
fn _calculate_V_m(
    cell_map_m_port: &mut CellMapM,
    cell_map_m_stb: &mut CellMapM,
    pose: &Pose3D,
    geometric_correction: &GeometricCorrection,
    swath_processed: &SwathProcessed,
    sonar: &SonarParams,
    map_resolution: f64,
    map_offset_yaw: f64,
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
        sonar.transducer_port.blind_zone_scale,
        map_resolution,
        map_offset_yaw,
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
        sonar.transducer_stb.blind_zone_scale,
        map_resolution,
        map_offset_yaw,
    );
}

// Estimates the measurement intensity V_m for each surviving candidate cell on one sonar side.
//
// For every candidate map cell, the function projects the cell footprint back into
// sonar slant-range space and samples the processed swath intensity there.
// Instead of using only the cell center, it uses all 4 cell corners, interpolates
// the corresponding sonar intensities, and averages them to get one representative
// intensity value for that map cell.
//
// This is done separately for port and starboard, because the projection depends on
// the transducer mounting, usable slant-range geometry, and channel storage direction.
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
    blind_zone_scale: f64,
    map_resolution: f64,
    map_offset_yaw: f64,
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
    // ? NOTE: `map_offset_yaw` is currently applied as a fixed alignment correction
    // ? between the pose yaw convention and the map/sonar ground-plane convention.
    // ? This works for the current setup, but ideally the underlying frame definition
    // ? should be made fully consistent so this extra offset is no longer needed.
    let R_body_to_world = Rotation2::new(pose.orientation.yaw + map_offset_yaw);
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
            range_corners_body[i] = R_world_to_body * (range_corners_world[i] - pose_body_in_world);
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

        // Compute the effective slant-range resolution used when projecting map cells
        // back into sonar sample space.
        //
        // ? NOTE:
        // ? The blind-zone scale is included here for the same practical reason as in the
        // ? swath-processing stage: the purely geometric first-bottom-return/slant-range
        // ? mapping tends to place the near-nadir boundary slightly too far out in real data.
        // ? Applying the same empirical blind-zone correction here keeps the map-to-swath
        // ? intensity lookup aligned with the corrected usable sonar region, so projected
        // ? cell intensities are sampled from the same effective slant-space geometry as
        // ? the processed swath data.
        let slant_resolution = (1.0 / blind_zone_scale) * (max_range/swath_samples as f64);

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
        map_m: FxHashMap::with_capacity_and_hasher(
            cell_map_m_port.map_m.len() + cell_map_m_stb.map_m.len(),
            Default::default(),
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
// Builds the dense normalized map image M from the current sparse chunk map.
//
// The sparse chunk map stores only cells that actually contain accumulated data.
// This function converts that sparse representation into a dense 2D image by
// first determining the global map bounds, allocating the required dense map,
// and then writing each valid sparse cell into its corresponding dense pixel.
//
// Each valid pixel is computed from the normalized map value
//
//     M = V / P
//
// while missing cells remain zero. The current vehicle pose is also converted
// into map cell coordinates so it can be stored consistently inside the map frame.
#[allow(non_snake_case)]
pub fn _calculate_M(
    pose: &Pose3D,
    chunk_map: &ChunkMap,
    map_resolution: f64,
) -> Map {
    if chunk_map.chunks.is_empty() {
        return Map::new(
            Pose2DMap {
                x: 0,
                y: 0,
                yaw: pose.orientation.yaw,
            },
            map_resolution,
            0,
            0,
        );
    }

    // Convert the sparse chunk map bounds into global cell-space bounds.
    // These bounds define the dense map rectangle that will be allocated and filled.
    let (min_cell_x, max_cell_x, min_cell_y, max_cell_y) = _map_cell_bounds(chunk_map);

    let width = (max_cell_x - min_cell_x) as usize;
    let height = (max_cell_y - min_cell_y) as usize;

    // Convert the vehicle pose from metric world coordinates into global cell coordinates,
    // so the pose can be placed correctly inside the dense map frame.
    let pose_global_x = (pose.position.x/map_resolution).round() as i64;
    let pose_global_y = (pose.position.y/map_resolution).round() as i64;

    let pose_map = Pose2DMap {
        x: pose_global_x - min_cell_x,
        y: pose_global_y - min_cell_y,
        yaw: pose.orientation.yaw,
    };

    let mut M = Map::new(
        pose_map,
        map_resolution,
        width,
        height,
    );

    // Sparse traversal:
    // iterate only chunks/cells that actually exist, then write into dense map
    for (chunk_coord, chunk) in &chunk_map.chunks {
        let chunk_base_x = chunk_coord.x * chunk_map.chunk_size;
        let chunk_base_y = chunk_coord.y * chunk_map.chunk_size;

        for (local_cell_coord, cell) in &chunk.data {
            if cell.p <= 0.0 {
                continue;
            }

            let global_x = chunk_base_x + local_cell_coord.x;
            let global_y = chunk_base_y + local_cell_coord.y;

            let col = (global_x - min_cell_x) as usize;
            let row = (global_y - min_cell_y) as usize;

            let m = (cell.v/cell.p).clamp(0.0, 255.0) as u8;
            M.set(col, row, m);
        }
    }

    return M;
}

// Returns the global dense-map cell bounds covered by the current sparse chunk map.
//
// The chunk map stores data in chunk coordinates, while later map generation
// and fill-in steps often need the corresponding bounds in global cell coordinates.
// This helper converts the minimum and maximum active chunk coordinates into the
// equivalent global cell-space rectangle:
//
//   min_cell_x .. max_cell_x
//   min_cell_y .. max_cell_y
//
// where the max bounds are exclusive, matching the usual width/height convention.
#[inline]
fn _map_cell_bounds(chunk_map: &ChunkMap) -> (i64, i64, i64, i64) {
    let min_chunk_x = chunk_map.chunks.keys().map(|c| c.x).min().unwrap();
    let max_chunk_x = chunk_map.chunks.keys().map(|c| c.x).max().unwrap();
    let min_chunk_y = chunk_map.chunks.keys().map(|c| c.y).min().unwrap();
    let max_chunk_y = chunk_map.chunks.keys().map(|c| c.y).max().unwrap();

    let min_cell_x = min_chunk_x * chunk_map.chunk_size;
    let max_cell_x = (max_chunk_x + 1) * chunk_map.chunk_size;
    let min_cell_y = min_chunk_y * chunk_map.chunk_size;
    let max_cell_y = (max_chunk_y + 1) * chunk_map.chunk_size;

    (min_cell_x, max_cell_x, min_cell_y, max_cell_y)
}
// Normalized Map Functions (STOP) --------------------------------------------------



// Fill Inn Functions (START) --------------------------------------------------
// Fills small gaps in the dense map by iteratively checking only empty pixels
// inside active chunk regions from the sparse chunk map.
//
// The algorithm first builds a compact list of empty candidate cells from the
// chunk footprints, instead of scanning the full dense map each round.
// It then runs several fill passes, where each pass reads from a frozen copy
// of the current map and fills only those candidates that have enough valid
// neighboring pixels in their local 3x3 neighborhood.
//
// Cells that get filled are removed from future work, while unresolved cells
// are kept for later passes. This makes the workload shrink over time and keeps
// the method much faster than repeatedly scanning the full map.
#[allow(non_snake_case)]
fn _fill_small_gaps(
    M: &mut Map,
    chunk_map: &ChunkMap,
    min_neighbors: u8,
    passes: u8,
) {
    if M.width == 0 || M.height == 0 || passes == 0 || chunk_map.chunks.is_empty() {
        return;
    }

    // Recover the global cell-space origin of the dense map so chunk-local coordinates
    // can be converted into pixel coordinates inside M.
    let (min_cell_x, _, min_cell_y, _) = _map_cell_bounds(chunk_map);

    let chunk_size_usize = chunk_map.chunk_size as usize;
    let chunk_area = chunk_size_usize * chunk_size_usize;

    // Build the initial list of fill candidates only once before the iterative passes.
    //
    // The goal here is to avoid scanning the full dense map during every fill round.
    // Instead, we use the sparse chunk map to identify which regions are actually active,
    // and inside those active chunks we collect only the cells that are currently empty.
    //
    // To make this fast, each chunk is first converted into a temporary local occupancy mask.
    // That mask tells us which local chunk cells are already occupied by real map data.
    // After that, we do one dense scan over the local chunk footprint and save only the
    // empty cells as candidate pixels that may later be filled by the gap-filling step.
    //
    // This is much faster than asking the hashmap for every cell in the chunk footprint,
    // because the occupancy mask turns repeated hashmap lookups into cheap array indexing.
    let mut empty_cells: Vec<(usize, usize)> = Vec::new();
    let mut occupied = vec![false; chunk_area];

    for (chunk_coord, chunk) in &chunk_map.chunks {
        let chunk_base_x = chunk_coord.x * chunk_map.chunk_size;
        let chunk_base_y = chunk_coord.y * chunk_map.chunk_size;

        // Reset the temporary local occupancy mask for this chunk.
        // The mask is reused for every chunk to avoid reallocating memory.
        occupied.fill(false);

        // Mark which local cells inside this chunk are already occupied
        // according to the sparse chunk data structure.
        //
        // After this step:
        // - occupied[...] = true  means the cell already contains valid map data
        // - occupied[...] = false means the cell is empty and may become a fill candidate
        for local_cell_coord in chunk.data.keys() {
            let lx = local_cell_coord.x as usize;
            let ly = local_cell_coord.y as usize;

            occupied[ly * chunk_size_usize + lx] = true;
        }

        // Scan the full local chunk footprint and collect only the empty cells.
        //
        // These empty cells are converted from local chunk coordinates into dense map
        // coordinates (x, y) inside M, and stored in `empty_cells`.
        // That gives us one compact work list of pixels that the iterative fill
        // algorithm should actually consider later.
        for local_y in 0..chunk_size_usize {
            for local_x in 0..chunk_size_usize {
                if occupied[local_y * chunk_size_usize + local_x] {
                    continue;
                }

                let global_x = chunk_base_x + local_x as i64;
                let global_y = chunk_base_y + local_y as i64;

                let x = (global_x - min_cell_x) as usize;
                let y = (global_y - min_cell_y) as usize;

                if x < M.width && y < M.height {
                    empty_cells.push((x, y));
                }
            }
        }
    }

    // Run the gap-filling step iteratively using only the candidate empty cells
    // collected earlier from the active chunk regions.
    //
    // The purpose of this stage is to gradually fill small holes in the dense map
    // without scanning the full image every time. Instead, each pass works only on
    // the current list of still-empty candidate pixels.
    //
    // At the start of each pass, the current map is copied into `src`.
    // This frozen copy is used for all neighborhood checks during that pass, so
    // newly filled pixels do not immediately influence other pixels in the same round.
    //
    // Any pixel that gets filled is removed from future work automatically.
    // Any pixel that still cannot be filled is kept in a new candidate list for the
    // next pass. This means the work list shrinks over time, making later passes cheaper.
    for _ in 0..passes {
        if empty_cells.is_empty() {
            break;
        }

        let src = M.data.clone();
        let mut next_empty_cells: Vec<(usize, usize)> = Vec::with_capacity(empty_cells.len());
        let mut any_filled = false;

        // Process each currently empty candidate pixel and check whether it now has
        // enough valid support in its local 3x3 neighborhood to be filled.
        //
        // Pixels that succeed are written into M.
        // Pixels that still lack enough support are carried forward into the next pass.
        for &(x, y) in &empty_cells {
            // Skip if already filled from an earlier pass.
            if src[y][x] != 0 {
                continue;
            }

            // Define the local 3x3 neighborhood around this empty cell.
            let y0 = y.saturating_sub(1);
            let y1 = (y + 1).min(M.height - 1);
            let x0 = x.saturating_sub(1);
            let x1 = (x + 1).min(M.width - 1);

            // Collect valid neighboring intensities from the frozen source map.
            // Only nonzero pixels count as real support for the fill.
            let mut sum = 0u32;
            let mut count = 0u8;

            for ny in y0..=y1 {
                for nx in x0..=x1 {
                    if nx == x && ny == y {
                        continue;
                    }

                    let v = src[ny][nx];
                    if v != 0 {
                        sum += v as u32;
                        count += 1;
                    }
                }
            }

            // Fill the current empty pixel only if enough valid neighbors exist nearby.
            // Otherwise, keep it in the candidate list so it can be retried in a later pass
            // after surrounding pixels may have been filled.
            if count >= min_neighbors {
                M.data[y][x] = (sum as f64 / count as f64).round() as u8;
                any_filled = true;
            } else {
                next_empty_cells.push((x, y));
            }
        }

        // Replace the old candidate list with only the cells that are still unresolved.
        // This makes the next pass cheaper because already-filled pixels are no longer tracked.
        empty_cells = next_empty_cells;

        // Stop early if this pass could not fill anything.
        // In that case, more passes would do the same useless work again.
        if !any_filled {
            break;
        }
    }
}
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
) {
    for (cell_coord_m, cell_data_m) in &cell_map_m.map_m {
        let chunk_coord = ChunkCoord {
            x: cell_coord_m.x_m.div_euclid(chunk_map.chunk_size),
            y: cell_coord_m.y_m.div_euclid(chunk_map.chunk_size),
        };

        let local_cell_coord = CellCoord {
            x: cell_coord_m.x_m.rem_euclid(chunk_map.chunk_size),
            y: cell_coord_m.y_m.rem_euclid(chunk_map.chunk_size),
        };

        let chunk = chunk_map
            .chunks
            .entry(chunk_coord)
            .or_insert_with(|| Chunk {
                age: 0,
                data: Default::default(),
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
) {
    let chunk_max_age = chunk_map.chunk_max_age;
    chunk_map
        .chunks
        .retain(|_, chunk| chunk.age <= chunk_max_age);
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
