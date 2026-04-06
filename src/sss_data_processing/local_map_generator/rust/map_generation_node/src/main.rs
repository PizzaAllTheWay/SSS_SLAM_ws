// Libraries for conversions ----------
use nalgebra::{
    Quaternion, 
    UnitQuaternion
};
// Libraries for realtime ----------
use std::sync::{
    Arc,
    RwLock
};
use futures::{
    future, 
    stream::StreamExt
};
use tokio::time::{
    interval,
    Duration,
};
// Libraries for ROS2 ----------
use r2r::{
    Context,
    Node,
    QosProfile,
    Publisher,
    std_msgs::msg::Header,
    geometry_msgs::msg::PoseStamped,
    geometry_msgs::msg::PolygonStamped,
    marine_acoustic_msgs::msg::RawSonarImage,
    sensor_msgs::msg::Image,
    
};
// Libraries for Map Generation  ----------
use map_generation_node::map_generation_lib::types::{
    GeometricCorrection,
    Orientation,
    Pose3D,
    Position,
    SwathProcessed,
    SonarParams,
    TransducerParams,
};
use map_generation_node::map_generation_lib::generator::MapGenerator;
use map_generation_node::map_generation_lib::utils::{
    LoggerPerformance,
    LoggerMapPose,
    LoggerMapOrigin,
    LoggerMap,
};



#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize ROS2 (START) --------------------------------------------------
    // Shared variables ----------
    // Subscribers
    let pose_shared = Arc::new(RwLock::new(
        Pose3D {
            position: Position {
                x: 0.0,
                y: 0.0,
                z: 0.0
            },
            orientation: Orientation {
                roll: 0.0,
                pitch: 0.0,
                yaw: 0.0
            },
        }
    ));
    let geometric_correction_shared = Arc::new(RwLock::new(
        GeometricCorrection {
            h_port: 0.0,
            h_stb: 0.0,
            r_fbr_port: 0.0,
            r_fbr_stb: 0.0,
        }
    ));
    // Map Generation Specific
    let last_map_header_shared: Arc<RwLock<Option<Header>>> = Arc::new(RwLock::new(None));
    let last_map_pose_shared = Arc::new(RwLock::new(
        Pose3D {
            position: Position {
                x: 0.0,
                y: 0.0,
                z: 0.0
            },
            orientation: Orientation {
                roll: 0.0,
                pitch: 0.0,
                yaw: 0.0
            },
        }
    ));
    let swath_counter_shared: Arc<RwLock<usize>> = Arc::new(RwLock::new(0));

    // Node ----------
    let ctx = Context::create()?;
    let mut node = Node::create(ctx, "map_generation_node", "")?;

    // Parameters ----------
    #[allow(non_snake_case)]
    // ! TRUE LOG ! let LOG = node.get_parameter::<bool>("log").ok().unwrap_or(false) as bool;
    let LOG = false; // ! THIS IS FAKE LOG FOR DEBUGGING, REMOVE IT LATER AND UNCOMMENT THE "! TRUE LOG !" TO GO BACK TO NORMAL
    
    let map_update_every_n_swaths = node.get_parameter::<i64>("local_map_generator.map_update_every_n_swaths").ok().unwrap_or(0) as usize;
    let map_update_max_time_gap = node.get_parameter::<f64>("local_map_generator.map_update_max_time_gap").ok().unwrap_or(0.0);
    let map_resolution = node.get_parameter::<f64>("local_map_generator.map_resolution").ok().unwrap_or(0.0);
    let chunk_size = node.get_parameter::<i64>("local_map_generator.chunk_size").ok().unwrap_or(0);
    let chunk_max_age = node.get_parameter::<i64>("local_map_generator.chunk_max_age").ok().unwrap_or(0) as u32;
    let beam_weight_threshold = node.get_parameter::<f64>("local_map_generator.beam_weight_threshold").ok().unwrap_or(0.0);
    let probabilistic_map_threshold = node.get_parameter::<f64>("local_map_generator.probabilistic_map_threshold").ok().unwrap_or(0.0);
    let fill_inn_min_neighbors = node.get_parameter::<i64>("local_map_generator.fill_inn_min_neighbors").ok().unwrap_or(0) as u8;
    let fill_inn_passes = node.get_parameter::<i64>("local_map_generator.fill_inn_passes").ok().unwrap_or(0) as u8;
    let map_offset_yaw_deg = node.get_parameter::<f64>("local_map_generator.map_offset_yaw_deg").ok().unwrap_or(0.0);

    let transducer_port_pose_position_x = node.get_parameter::<f64>("local_map_generator.transducers.port.pose.position.x").ok().unwrap_or(0.0) as f64;
    let transducer_port_pose_position_y = node.get_parameter::<f64>("local_map_generator.transducers.port.pose.position.y").ok().unwrap_or(0.0) as f64;
    let transducer_port_pose_position_z = node.get_parameter::<f64>("local_map_generator.transducers.port.pose.position.z").ok().unwrap_or(0.0) as f64;
    let transducer_port_pose_orientation_roll = node.get_parameter::<f64>("local_map_generator.transducers.port.pose.orientation.roll").ok().unwrap_or(0.0) as f64;
    let transducer_port_pose_orientation_pitch = node.get_parameter::<f64>("local_map_generator.transducers.port.pose.orientation.pitch").ok().unwrap_or(0.0) as f64;
    let transducer_port_pose_orientation_yaw = node.get_parameter::<f64>("local_map_generator.transducers.port.pose.orientation.yaw").ok().unwrap_or(0.0) as f64;
    let transducer_port_max_range = node.get_parameter::<f64>("local_map_generator.transducers.port.max_range").ok().unwrap_or(0.0) as f64;
    let transducer_port_theta = node.get_parameter::<f64>("local_map_generator.transducers.port.theta").ok().unwrap_or(0.0) as f64;
    let transducer_port_is_reversed = node.get_parameter::<bool>("local_map_generator.transducers.port.is_reversed").ok().unwrap_or(false) as bool;
    let transducer_port_blind_zone_scale = node.get_parameter::<f64>("local_map_generator.transducers.port.blind_zone_scale").ok().unwrap_or(0.0) as f64;
    
    let transducer_starboard_pose_position_x = node.get_parameter::<f64>("local_map_generator.transducers.starboard.pose.position.x").ok().unwrap_or(0.0) as f64;
    let transducer_starboard_pose_position_y = node.get_parameter::<f64>("local_map_generator.transducers.starboard.pose.position.y").ok().unwrap_or(0.0) as f64;
    let transducer_starboard_pose_position_z = node.get_parameter::<f64>("local_map_generator.transducers.starboard.pose.position.z").ok().unwrap_or(0.0) as f64;
    let transducer_starboard_pose_orientation_roll = node.get_parameter::<f64>("local_map_generator.transducers.starboard.pose.orientation.roll").ok().unwrap_or(0.0) as f64;
    let transducer_starboard_pose_orientation_pitch = node.get_parameter::<f64>("local_map_generator.transducers.starboard.pose.orientation.pitch").ok().unwrap_or(0.0) as f64;
    let transducer_starboard_pose_orientation_yaw = node.get_parameter::<f64>("local_map_generator.transducers.starboard.pose.orientation.yaw").ok().unwrap_or(0.0) as f64;
    let transducer_starboard_max_range = node.get_parameter::<f64>("local_map_generator.transducers.starboard.max_range").ok().unwrap_or(0.0) as f64;
    let transducer_starboard_theta = node.get_parameter::<f64>("local_map_generator.transducers.starboard.theta").ok().unwrap_or(0.0) as f64;
    let transducer_starboard_is_reversed = node.get_parameter::<bool>("local_map_generator.transducers.starboard.is_reversed").ok().unwrap_or(false) as bool;
    let transducer_starboard_blind_zone_scale = node.get_parameter::<f64>("local_map_generator.transducers.starboard.blind_zone_scale").ok().unwrap_or(0.0) as f64;

    r2r::log_info!("map_generation_node", "log: {}", LOG);
    r2r::log_info!("map_generation_node", "");
    r2r::log_info!("map_generation_node", "local_map_generator.map_update_every_n_swaths:   {}",   map_update_every_n_swaths);
    r2r::log_info!("map_generation_node", "local_map_generator.map_update_max_time_gap:     {}",     map_update_max_time_gap);
    r2r::log_info!("map_generation_node", "local_map_generator.map_resolution:              {}",              map_resolution);
    r2r::log_info!("map_generation_node", "local_map_generator.chunk_size:                  {}",                  chunk_size);
    r2r::log_info!("map_generation_node", "local_map_generator.chunk_max_age:               {}",               chunk_max_age);
    r2r::log_info!("map_generation_node", "local_map_generator.beam_weight_threshold:       {}",       beam_weight_threshold);
    r2r::log_info!("map_generation_node", "local_map_generator.probabilistic_map_threshold: {}", probabilistic_map_threshold);
    r2r::log_info!("map_generation_node", "local_map_generator.fill_inn_min_neighbors:      {}",      fill_inn_min_neighbors);
    r2r::log_info!("map_generation_node", "local_map_generator.fill_inn_passes:             {}",             fill_inn_passes);
    r2r::log_info!("map_generation_node", "local_map_generator.map_offset_yaw_deg:          {}",          map_offset_yaw_deg);
    r2r::log_info!("map_generation_node", "");
    r2r::log_info!("map_generation_node", "local_map_generator.transducers.port.pose.position.x:        {:.4} [m]  ",        transducer_port_pose_position_x);
    r2r::log_info!("map_generation_node", "local_map_generator.transducers.port.pose.position.y:        {:.4} [m]  ",        transducer_port_pose_position_y);
    r2r::log_info!("map_generation_node", "local_map_generator.transducers.port.pose.position.z:        {:.4} [m]  ",        transducer_port_pose_position_z);
    r2r::log_info!("map_generation_node", "local_map_generator.transducers.port.pose.orientation.roll:  {:.2} [deg]",  transducer_port_pose_orientation_roll);
    r2r::log_info!("map_generation_node", "local_map_generator.transducers.port.pose.orientation.pitch: {:.2} [deg]", transducer_port_pose_orientation_pitch);
    r2r::log_info!("map_generation_node", "local_map_generator.transducers.port.pose.orientation.yaw:   {:.2} [deg]",   transducer_port_pose_orientation_yaw);
    r2r::log_info!("map_generation_node", "local_map_generator.transducers.port.max_range:              {:.2} [m]  ",              transducer_port_max_range);
    r2r::log_info!("map_generation_node", "local_map_generator.transducers.port.theta:                  {:.6} [deg]",                  transducer_port_theta);
    r2r::log_info!("map_generation_node", "local_map_generator.transducers.port.is_reversed:            {}         ",            transducer_port_is_reversed);
    r2r::log_info!("map_generation_node", "local_map_generator.transducers.port.blind_zone_scale:       {:.4}      ",       transducer_port_blind_zone_scale);
    r2r::log_info!("map_generation_node", "");
    r2r::log_info!("map_generation_node", "local_map_generator.transducers.starboard.pose.position.x:        {:.4} [m]  ",        transducer_starboard_pose_position_x);
    r2r::log_info!("map_generation_node", "local_map_generator.transducers.starboard.pose.position.y:        {:.4} [m]  ",        transducer_starboard_pose_position_y);
    r2r::log_info!("map_generation_node", "local_map_generator.transducers.starboard.pose.position.z:        {:.4} [m]  ",        transducer_starboard_pose_position_z);
    r2r::log_info!("map_generation_node", "local_map_generator.transducers.starboard.pose.orientation.roll:  {:.2} [deg]",  transducer_starboard_pose_orientation_roll);
    r2r::log_info!("map_generation_node", "local_map_generator.transducers.starboard.pose.orientation.pitch: {:.2} [deg]", transducer_starboard_pose_orientation_pitch);
    r2r::log_info!("map_generation_node", "local_map_generator.transducers.starboard.pose.orientation.yaw:   {:.2} [deg]",   transducer_starboard_pose_orientation_yaw);
    r2r::log_info!("map_generation_node", "local_map_generator.transducers.starboard.max_range:              {:.2} [m]  ",              transducer_starboard_max_range);
    r2r::log_info!("map_generation_node", "local_map_generator.transducers.starboard.theta:                  {:.6} [deg]",                  transducer_starboard_theta);
    r2r::log_info!("map_generation_node", "local_map_generator.transducers.starboard.is_reversed:            {}         ",            transducer_starboard_is_reversed);
    r2r::log_info!("map_generation_node", "local_map_generator.transducers.starboard.blind_zone_scale:       {:.4}      ",       transducer_starboard_blind_zone_scale);

    let sonar = SonarParams {
        transducer_port: TransducerParams {
            offset: Pose3D {
                position: Position {
                    x: transducer_port_pose_position_x,
                    y: transducer_port_pose_position_y,
                    z: transducer_port_pose_position_z,
                },
                orientation: Orientation {
                    roll: transducer_port_pose_orientation_roll.to_radians(),
                    pitch: transducer_port_pose_orientation_pitch.to_radians(),
                    yaw: transducer_port_pose_orientation_yaw.to_radians(),
                },
            },
            max_range: transducer_port_max_range,
            theta: transducer_port_theta.to_radians(),
            is_reversed: transducer_port_is_reversed,
            blind_zone_scale: transducer_port_blind_zone_scale,
        },
        transducer_stb: TransducerParams {
            offset: Pose3D {
                position: Position {
                    x: transducer_starboard_pose_position_x,
                    y: transducer_starboard_pose_position_y,
                    z: transducer_starboard_pose_position_z,
                },
                orientation: Orientation {
                    roll: transducer_starboard_pose_orientation_roll.to_radians(),
                    pitch: transducer_starboard_pose_orientation_pitch.to_radians(),
                    yaw: transducer_starboard_pose_orientation_yaw.to_radians(),
                },
            },
            max_range: transducer_starboard_max_range,
            theta: transducer_starboard_theta.to_radians(),
            is_reversed: transducer_starboard_is_reversed,
            blind_zone_scale: transducer_starboard_blind_zone_scale,
        },
    };

    // Initialize Map Generation ----------
    let map_generator_shared = Arc::new(RwLock::new(MapGenerator::new(
        sonar,
        map_resolution,
        chunk_size,
        chunk_max_age,
        beam_weight_threshold,
        probabilistic_map_threshold,
        fill_inn_min_neighbors,
        fill_inn_passes,
        map_offset_yaw_deg.to_radians(),
    )));

    // Subscribers ----------
    let sub_swath_pose = node.subscribe::<PoseStamped>("/sss_slam/data_processing/swath/pose", QosProfile::default())?;
    let sub_geometric_correction = node.subscribe::<PolygonStamped>("/sss_slam/data_processing/swath/geometric_correction", QosProfile::default())?;
    let sub_swath_processed = node.subscribe::<RawSonarImage>("/sss_slam/data_processing/swath/processed", QosProfile::default())?;

    // Publishers ----------
    let pub_map_pose = node.create_publisher::<PoseStamped>("/sss_slam/data_processing/map_generation/pose", QosProfile::default())?;
    let pub_map_origin = node.create_publisher::<PoseStamped>("/sss_slam/data_processing/map_generation/map_origin", QosProfile::default())?;
    let pub_map = node.create_publisher::<Image>("/sss_slam/data_processing/map_generation/map", QosProfile::default())?;
    
    // Loggers ----------
    // If logging is enabled, these logger instances are created and used for data logging
    let logger_performance_shared = Arc::new(RwLock::new(if LOG { Some(LoggerPerformance::new()) } else { None }));
    let logger_map_pose_shared = Arc::new(RwLock::new(if LOG { Some(LoggerMapPose::new()) } else { None }));
    let logger_map_origin_shared = Arc::new(RwLock::new(if LOG { Some(LoggerMapOrigin::new()) } else { None }));
    let logger_map_shared = Arc::new(RwLock::new(if LOG { Some(LoggerMap::new()) } else { None }));
    // Initialize ROS2 (STOP) --------------------------------------------------



    // ROS2 Handlers (START) --------------------------------------------------
    // Spin ----------
    let handle = tokio::task::spawn_blocking(move || loop {
        node.spin_once(std::time::Duration::from_millis(10));
    });

    // Swath Pose ----------
    let pose_clone = pose_shared.clone();
    let swath_pose_task = sub_swath_pose.for_each({
        move |msg| {
            // Acquire shared resource
            let mut pose = pose_clone.write().unwrap();

            // Save ROS2 message to shared resource for future processing
            // Position
            pose.position.x = msg.pose.position.x;
            pose.position.y = msg.pose.position.y;
            pose.position.z = msg.pose.position.z;

            // Orientation
            let q = msg.pose.orientation;
            let quat = UnitQuaternion::from_quaternion(Quaternion::new(q.w, q.x, q.y, q.z));
            let (roll, pitch, yaw) = quat.euler_angles();
            pose.orientation.roll = roll;
            pose.orientation.pitch = pitch;
            pose.orientation.yaw = yaw;
            
            return future::ready(());
        }
    });

    // Geometric Correction ----------
    let geometric_correction_clone = geometric_correction_shared.clone();
    let geometric_task = sub_geometric_correction.for_each({
        move |msg| {
            // Ensure the message format is correct
            if msg.polygon.points.len() >= 2 {
                // Acquire shared resource
                let mut geometric_correction = geometric_correction_clone.write().unwrap();
                
                // Port
                geometric_correction.r_fbr_port = msg.polygon.points[0].x as f64;
                geometric_correction.h_port = msg.polygon.points[0].z as f64;

                // Starboard
                geometric_correction.r_fbr_stb = msg.polygon.points[1].x as f64;
                geometric_correction.h_stb = msg.polygon.points[1].z as f64;
            }
            
            future::ready(())
        }
    });

    // Swath Processed ----------
    let pose_clone = pose_shared.clone();
    let geometric_clone = geometric_correction_shared.clone();

    let last_map_header_clone = last_map_header_shared.clone();
    let last_map_pose_clone = last_map_pose_shared.clone();

    let map_generator_clone = map_generator_shared.clone();

    let pub_map_pose_clone = pub_map_pose.clone();
    let pub_map_origin_clone = pub_map_origin.clone();
    let pub_map_clone = pub_map.clone();

    let logger_performance_clone = logger_performance_shared.clone();
    let logger_map_pose_clone = logger_map_pose_shared.clone();
    let logger_map_origin_clone = logger_map_origin_shared.clone();
    let logger_map_clone = logger_map_shared.clone();

    let swath_counter_clone = swath_counter_shared.clone();

    let swath_processed_task = sub_swath_processed.for_each({
        move |msg| {
            if let Some(l) = &mut *logger_performance_clone.write().unwrap() {
                l.start();
            }

            let t = msg.header.stamp.sec as f64 + msg.header.stamp.nanosec as f64 * 1e-9;

            // Extract sonar data -----
            // Fast path explanation:
            // - Vec::with_capacity(n) allocates memory but does NOT initialize it (len = 0)
            // - set_len(n) marks the buffer as fully sized WITHOUT writing zeros (avoids unnecessary work)
            // - copy_from_slice then fills the memory in one go
            // Compared to data[..n].to_vec():
            // - both allocate + copy
            // - but this avoids implicit initialization/potential extra work
            // This matters for high-rate data (like sonar) where we want minimal overhead per message
            // SAFETY: set_len is safe here because we immediately overwrite ALL elements before use
            let data = &msg.image.data;
            let n = msg.samples_per_beam as usize;

            let mut port = Vec::with_capacity(n);
            let mut starboard = Vec::with_capacity(n);

            unsafe {
                port.set_len(n);
                starboard.set_len(n);
            }

            port.copy_from_slice(&data[..n]);
            starboard.copy_from_slice(&data[n..2*n]);

            let swath_processed = SwathProcessed {
                timestamp: t,
                port,
                starboard,
                samples_per_beam: msg.samples_per_beam as u64,
            };

            // Acquire shared resources -----
            let pose = pose_clone.read().unwrap().clone();
            let geometric = geometric_clone.read().unwrap().clone();

            // ? NOTE:
            // ? We encase in brackets to ensure memory safe and real time safe approach
            // ? This is to ensure in an event of an edge case where we get sonar data 
            // ? while we are still processing map data later on, we don't override the precessing data
            // ? Instead we wait until Map is generated before buffering new data
            {
                // Generate Map -----
                let mut map_generator = map_generator_clone.write().unwrap();
                map_generator.buffer_processed_swath_into_map(
                    &pose,
                    geometric,
                    swath_processed
                );
            }

            // Update latest data ----------
            // This ensures if no new data comes in
            // The timer task can still trigger and send data in correct manner from where we left off here
            {
                let mut last_map_header = last_map_header_clone.write().unwrap();
                *last_map_header = Some(msg.header.clone());
            }
            {
                let mut last_map_pose = last_map_pose_clone.write().unwrap();
                *last_map_pose = pose.clone();
            }

            // Map update logic ----------
            // If we have acuminated enough processed swaths
            // Create and publish the map and all the necessary data
            {
                let mut swath_counter = swath_counter_clone.write().unwrap();
                *swath_counter += 1;
            }
            let swath_counter = swath_counter_clone.read().unwrap().clone();

            if swath_counter >= map_update_every_n_swaths {
                {
                    let mut swath_counter = swath_counter_clone.write().unwrap();
                    *swath_counter = 0;
                }

                publish_map(
                    &map_generator_clone,
                    &pose_clone,

                    &msg.header,
                    &pub_map_pose_clone,
                    &pub_map_origin_clone,
                    &pub_map_clone,

                    &logger_performance_clone,
                    &logger_map_pose_clone,
                    &logger_map_origin_clone,
                    &logger_map_clone,
                );
            }
            else { 
                if let Some(l) = &mut *logger_performance_clone.write().unwrap() {
                    l.stop(t);
                }
            }

            future::ready(())
        }
    });

    // Map Timer ----------
    // Timer-based safety trigger for dense map publishing.
    //
    // The normal map update path happens after enough swaths have been buffered.
    // This timer exists as a fallback: if swaths stop arriving before that count
    // is reached, the timer detects that the latest swath timestamp has stopped
    // changing and forces a map publish from the latest buffered map state.
    let last_map_header_clone = last_map_header_shared.clone();
    let last_map_pose_clone = last_map_pose_shared.clone();

    let map_generator_clone = map_generator_shared.clone();

    let pub_map_pose_clone = pub_map_pose.clone();
    let pub_map_origin_clone = pub_map_origin.clone();
    let pub_map_clone = pub_map.clone();

    let logger_performance_clone = logger_performance_shared.clone();
    let logger_map_pose_clone = logger_map_pose_shared.clone();
    let logger_map_origin_clone = logger_map_origin_shared.clone();
    let logger_map_clone = logger_map_shared.clone();

    let swath_counter_clone = swath_counter_shared.clone();

    let map_timer_task = tokio::spawn(async move {
        let mut timer = interval(Duration::from_secs_f64(map_update_max_time_gap));
        let mut last_t: Option<f64> = None;

        loop {
            // Wait one timer period before checking whether new swath data arrived.
            timer.tick().await;

            if let Some(l) = &mut *logger_performance_clone.write().unwrap() {
                l.start();
            }

            // Read the latest stored map header.
            // If no swath has ever been processed yet, there is nothing to do.
            let last_map_header = last_map_header_clone.read().unwrap().clone();
            let Some(last_map_header) = last_map_header else {
                continue;
            };

            // Convert the latest stored ROS header timestamp into seconds.
            let now_t = last_map_header.stamp.sec as f64 + last_map_header.stamp.nanosec as f64 * 1e-9;

            // First timer hit after startup:
            // just store the timestamp and wait for the next timer period.
            let Some(prev_t) = last_t else {
                last_t = Some(now_t);
                continue;
            };

            // If the timestamp has not changed since the previous timer check,
            // no new swath has arrived during the whole timer period.
            // In that case, force a map publish from the latest buffered state.
            if (now_t - prev_t).abs() < 1e-9 {
                {
                    let mut swath_counter = swath_counter_clone.write().unwrap();
                    *swath_counter = 0;
                }

                publish_map(
                    &map_generator_clone,
                    &last_map_pose_clone,

                    &last_map_header,
                    &pub_map_pose_clone,
                    &pub_map_origin_clone,
                    &pub_map_clone,

                    &logger_performance_clone,
                    &logger_map_pose_clone,
                    &logger_map_origin_clone,
                    &logger_map_clone,
                );
            }

            // Update the timer's internal timestamp tracker for the next round.
            last_t = Some(now_t);
        }
    });
    // ROS2 Handlers (STOP) --------------------------------------------------



    // ROS2 (START) --------------------------------------------------
    let (_, _, _, timer_result) = tokio::join!(
        swath_pose_task,
        geometric_task,
        swath_processed_task,
        map_timer_task,
    );

    timer_result?;

    handle.await?;

    Ok(())
    // ROS2 (STOP) --------------------------------------------------
}



// Helper Functions (START) --------------------------------------------------
fn publish_map(
    map_generator_shared: &Arc<RwLock<MapGenerator>>,
    map_pose_shared: &Arc<RwLock<Pose3D>>,

    header: &Header,
    pub_map_pose: &Publisher<PoseStamped>,
    pub_map_origin: &Publisher<PoseStamped>,
    pub_map: &Publisher<Image>,

    logger_performance_shared: &Arc<RwLock<Option<LoggerPerformance>>>,
    logger_map_pose_shared: &Arc<RwLock<Option<LoggerMapPose>>>,
    logger_map_origin_shared: &Arc<RwLock<Option<LoggerMapOrigin>>>,
    logger_map_shared: &Arc<RwLock<Option<LoggerMap>>>,
) {
    // Calculate ----------
    // Get pose shared variable
    let pose = map_pose_shared.read().unwrap().clone();

    // Calculate map
    let map = {
        let mut map_generator = map_generator_shared.write().unwrap();
        map_generator.calculate_map(&pose)
    };

    // If the calculated map is empty, skip publishing.
    // This avoids sending invalid/useless outputs when no active map data exists yet.
    if map.width == 0 || map.height == 0 {
        return;
    }

    // Publish ----------
    // Pose
    let mut pose_msg = PoseStamped::default();
    pose_msg.header.stamp = header.stamp.clone();
    pose_msg.header.frame_id = "map".to_string();

    pose_msg.pose.position.x = pose.position.x;
    pose_msg.pose.position.y = pose.position.y;
    pose_msg.pose.position.z = pose.position.z;

    let quat = UnitQuaternion::from_euler_angles(
        pose.orientation.roll,
        pose.orientation.pitch,
        pose.orientation.yaw,
    );
    let q = quat.quaternion();
    pose_msg.pose.orientation.x = q.i;
    pose_msg.pose.orientation.y = q.j;
    pose_msg.pose.orientation.z = q.k;
    pose_msg.pose.orientation.w = q.w;

    let _ = pub_map_pose.publish(&pose_msg);

    // Origin
    let mut map_origin_msg = PoseStamped::default();
    map_origin_msg.header.stamp = header.stamp.clone();
    map_origin_msg.header.frame_id = "map".to_string();

    map_origin_msg.pose.position.x = map.pose.x as f64;
    map_origin_msg.pose.position.y = map.pose.y as f64;
    map_origin_msg.pose.position.z = 0.0;

    let quat = UnitQuaternion::from_euler_angles(0.0, 0.0, map.pose.yaw);
    let q = quat.quaternion();
    map_origin_msg.pose.orientation.x = q.i;
    map_origin_msg.pose.orientation.y = q.j;
    map_origin_msg.pose.orientation.z = q.k;
    map_origin_msg.pose.orientation.w = q.w;

    let _ = pub_map_origin.publish(&map_origin_msg);

    // Map
    let mut map_msg = Image::default();
    map_msg.header.stamp = header.stamp.clone();
    map_msg.header.frame_id = "map".to_string();

    map_msg.height = map.height as u32;
    map_msg.width = map.width as u32;

    map_msg.encoding = "mono8".to_string();
    map_msg.is_bigendian = 0;
    map_msg.step = map.width as u32;

    map_msg.data = map.data.iter().flatten().copied().collect();

    let _ = pub_map.publish(&map_msg);

    // Log data ----------
    let t = header.stamp.sec as f64 + header.stamp.nanosec as f64 * 1e-9;

    if let Some(l) = &mut *logger_performance_shared.write().unwrap() {
        l.stop(t);
    }
    if let Some(l) = &mut *logger_map_pose_shared.write().unwrap() {
        l.log(t, &pose);
    }
    if let Some(l) = &mut *logger_map_origin_shared.write().unwrap() {
        l.log(t, &map.pose);
    }
    if let Some(l) = &mut *logger_map_shared.write().unwrap() {
        l.log(t, &map);
    }
}
// Helper Functions (STOP) --------------------------------------------------