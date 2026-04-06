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
// Libraries for ROS2 ----------
use r2r::{
    Context,
    Node,
    QosProfile, 
    nav_msgs::msg::Odometry, 
    marine_acoustic_msgs::msg::Dvl, 
    marine_acoustic_msgs::msg::RawSonarImage,
    geometry_msgs::msg::PoseStamped,
    geometry_msgs::msg::Point32,
    geometry_msgs::msg::PolygonStamped,
};
// Libraries for Swath processing ----------
use swath_processing_node::swath_processing_lib::types::{
    Position,
    Orientation,
    Pose3D,
    AltitudeMeasurement,
    SwathRaw,
    TransducerParams,
    SonarParams,
};
use swath_processing_node::swath_processing_lib::processor::{
    process_swath,
};
use swath_processing_node::swath_processing_lib::utils::{
    LoggerPerformance,
    LoggerPose,
    LoggerAltitudeDvl,
    LoggerGeometricCorrection,
    LoggerSwathRaw,
    LoggerSwathProcessed,
};



#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize ROS2 (START) --------------------------------------------------
    // Shared variables ----------
    let pose_shared = Arc::new(RwLock::new(Pose3D {
        position: Position {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        orientation: Orientation {
            roll: 0.0, 
            pitch: 0.0, 
            yaw: 0.0,
        },
    }));
    let altitude_shared = Arc::new(RwLock::new(AltitudeMeasurement {
        value: 0.0,
    }));

    // Node ----------
    let ctx = Context::create()?;
    let mut node = Node::create(ctx, "swath_processing_node", "")?;

    // Parameters ----------
    #[allow(non_snake_case)]
    // ! TRUE LOG ! let LOG = node.get_parameter::<bool>("log").ok().unwrap_or(false) as bool;
    let LOG = false; // ! THIS IS FAKE LOG FOR DEBUGGING, REMOVE IT LATER AND UNCOMMENT THE "! TRUE LOG !" TO GO BACK TO NORMAL
    
    let illumination_ema_period = node.get_parameter::<i64>("local_map_generator.illumination_ema_period").ok().unwrap_or(0) as usize;
    
    let transducer_port_pose_position_x = node.get_parameter::<f64>("local_map_generator.transducers.port.pose.position.x").ok().unwrap_or(0.0) as f64;
    let transducer_port_pose_position_y = node.get_parameter::<f64>("local_map_generator.transducers.port.pose.position.y").ok().unwrap_or(0.0) as f64;
    let transducer_port_pose_position_z = node.get_parameter::<f64>("local_map_generator.transducers.port.pose.position.z").ok().unwrap_or(0.0) as f64;
    let transducer_port_pose_orientation_roll = node.get_parameter::<f64>("local_map_generator.transducers.port.pose.orientation.roll").ok().unwrap_or(0.0) as f64;
    let transducer_port_pose_orientation_pitch = node.get_parameter::<f64>("local_map_generator.transducers.port.pose.orientation.pitch").ok().unwrap_or(0.0) as f64;
    let transducer_port_pose_orientation_yaw = node.get_parameter::<f64>("local_map_generator.transducers.port.pose.orientation.yaw").ok().unwrap_or(0.0) as f64;
    let transducer_port_max_range = node.get_parameter::<f64>("local_map_generator.transducers.port.max_range").ok().unwrap_or(0.0) as f64;
    let transducer_port_alpha = node.get_parameter::<f64>("local_map_generator.transducers.port.alpha").ok().unwrap_or(0.0) as f64;
    let transducer_port_is_reversed = node.get_parameter::<bool>("local_map_generator.transducers.port.is_reversed").ok().unwrap_or(false) as bool;
    let transducer_port_blind_zone_scale = node.get_parameter::<f64>("local_map_generator.transducers.port.blind_zone_scale").ok().unwrap_or(0.0) as f64;
    
    let transducer_starboard_pose_position_x = node.get_parameter::<f64>("local_map_generator.transducers.starboard.pose.position.x").ok().unwrap_or(0.0) as f64;
    let transducer_starboard_pose_position_y = node.get_parameter::<f64>("local_map_generator.transducers.starboard.pose.position.y").ok().unwrap_or(0.0) as f64;
    let transducer_starboard_pose_position_z = node.get_parameter::<f64>("local_map_generator.transducers.starboard.pose.position.z").ok().unwrap_or(0.0) as f64;
    let transducer_starboard_pose_orientation_roll = node.get_parameter::<f64>("local_map_generator.transducers.starboard.pose.orientation.roll").ok().unwrap_or(0.0) as f64;
    let transducer_starboard_pose_orientation_pitch = node.get_parameter::<f64>("local_map_generator.transducers.starboard.pose.orientation.pitch").ok().unwrap_or(0.0) as f64;
    let transducer_starboard_pose_orientation_yaw = node.get_parameter::<f64>("local_map_generator.transducers.starboard.pose.orientation.yaw").ok().unwrap_or(0.0) as f64;
    let transducer_starboard_max_range = node.get_parameter::<f64>("local_map_generator.transducers.starboard.max_range").ok().unwrap_or(0.0) as f64;
    let transducer_starboard_alpha = node.get_parameter::<f64>("local_map_generator.transducers.starboard.alpha").ok().unwrap_or(0.0) as f64;
    let transducer_starboard_is_reversed = node.get_parameter::<bool>("local_map_generator.transducers.starboard.is_reversed").ok().unwrap_or(false) as bool;
    let transducer_starboard_blind_zone_scale = node.get_parameter::<f64>("local_map_generator.transducers.starboard.blind_zone_scale").ok().unwrap_or(0.0) as f64;

    r2r::log_info!("swath_processing_node", "log: {}", LOG);
    r2r::log_info!("map_generation_node", "");
    r2r::log_info!("swath_processing_node", "local_map_generator.illumination_ema_period: {}", illumination_ema_period);
    r2r::log_info!("map_generation_node", "");
    r2r::log_info!("swath_processing_node", "local_map_generator.transducers.port.pose.position.x:        {:.4} [m]  ",        transducer_port_pose_position_x);
    r2r::log_info!("swath_processing_node", "local_map_generator.transducers.port.pose.position.y:        {:.4} [m]  ",        transducer_port_pose_position_y);
    r2r::log_info!("swath_processing_node", "local_map_generator.transducers.port.pose.position.z:        {:.4} [m]  ",        transducer_port_pose_position_z);
    r2r::log_info!("swath_processing_node", "local_map_generator.transducers.port.pose.orientation.roll:  {:.2} [deg]",  transducer_port_pose_orientation_roll);
    r2r::log_info!("swath_processing_node", "local_map_generator.transducers.port.pose.orientation.pitch: {:.2} [deg]", transducer_port_pose_orientation_pitch);
    r2r::log_info!("swath_processing_node", "local_map_generator.transducers.port.pose.orientation.yaw:   {:.2} [deg]",   transducer_port_pose_orientation_yaw);
    r2r::log_info!("swath_processing_node", "local_map_generator.transducers.port.max_range:              {:.2} [m]  ",              transducer_port_max_range);
    r2r::log_info!("swath_processing_node", "local_map_generator.transducers.port.alpha:                  {:.6} [deg]",                  transducer_port_alpha);
    r2r::log_info!("swath_processing_node", "local_map_generator.transducers.port.is_reversed:            {}         ",            transducer_port_is_reversed);
    r2r::log_info!("swath_processing_node", "local_map_generator.transducers.port.blind_zone_scale:       {:.4}      ",       transducer_port_blind_zone_scale);
    r2r::log_info!("map_generation_node", "");
    r2r::log_info!("swath_processing_node", "local_map_generator.transducers.starboard.pose.position.x:        {:.4} [m]  ",        transducer_starboard_pose_position_x);
    r2r::log_info!("swath_processing_node", "local_map_generator.transducers.starboard.pose.position.y:        {:.4} [m]  ",        transducer_starboard_pose_position_y);
    r2r::log_info!("swath_processing_node", "local_map_generator.transducers.starboard.pose.position.z:        {:.4} [m]  ",        transducer_starboard_pose_position_z);
    r2r::log_info!("swath_processing_node", "local_map_generator.transducers.starboard.pose.orientation.roll:  {:.2} [deg]",  transducer_starboard_pose_orientation_roll);
    r2r::log_info!("swath_processing_node", "local_map_generator.transducers.starboard.pose.orientation.pitch: {:.2} [deg]", transducer_starboard_pose_orientation_pitch);
    r2r::log_info!("swath_processing_node", "local_map_generator.transducers.starboard.pose.orientation.yaw:   {:.2} [deg]",   transducer_starboard_pose_orientation_yaw);
    r2r::log_info!("swath_processing_node", "local_map_generator.transducers.starboard.max_range:              {:.2} [m]  ",              transducer_starboard_max_range);
    r2r::log_info!("swath_processing_node", "local_map_generator.transducers.starboard.alpha:                  {:.6} [deg]",                  transducer_starboard_alpha);
    r2r::log_info!("swath_processing_node", "local_map_generator.transducers.starboard.is_reversed:            {}         ",            transducer_starboard_is_reversed);
    r2r::log_info!("swath_processing_node", "local_map_generator.transducers.starboard.blind_zone_scale:       {:.4}      ",       transducer_starboard_blind_zone_scale);

    let sonar = SonarParams {
        transducer_port: TransducerParams {
            offset: Pose3D {
                position: Position {
                    x: transducer_port_pose_position_x,
                    y: transducer_port_pose_position_y,
                    z: transducer_port_pose_position_z,
                },
                orientation: Orientation {
                    roll:  transducer_port_pose_orientation_roll.to_radians(),
                    pitch: transducer_port_pose_orientation_pitch.to_radians(),
                    yaw:   transducer_port_pose_orientation_yaw.to_radians(),
                },
            },
            max_range: transducer_port_max_range,
            alpha: transducer_port_alpha.to_radians(),
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
                    roll:  transducer_starboard_pose_orientation_roll.to_radians(),
                    pitch: transducer_starboard_pose_orientation_pitch.to_radians(),
                    yaw:   transducer_starboard_pose_orientation_yaw.to_radians(),
                },
            },
            max_range: transducer_starboard_max_range,
            alpha: transducer_starboard_alpha.to_radians(),
            is_reversed: transducer_starboard_is_reversed,
            blind_zone_scale: transducer_starboard_blind_zone_scale,
        },
    };

    // Subscribers ----------
    let sub_state_estimate = node.subscribe::<Odometry>("/sss_slam/data_processing/estimate/state", QosProfile::default())?;
    let sub_dvl = node.subscribe::<Dvl>("/hardware/dvl", QosProfile::default())?;
    let sub_side_scan_sonar = node.subscribe::<RawSonarImage>("/hardware/side_scan_sonar", QosProfile::default())?;

    // Publishers ----------
    let pub_pose = node.create_publisher::<PoseStamped>("/sss_slam/data_processing/swath/pose", QosProfile::default())?;
    let pub_geometric_correction = node.create_publisher::<PolygonStamped>("/sss_slam/data_processing/swath/geometric_correction", QosProfile::default())?;
    let pub_swath = node.create_publisher::<RawSonarImage>("/sss_slam/data_processing/swath/processed", QosProfile::default())?;
    
    // Loggers ----------
    // If logging is enabled, these logger instances are created and used for data logging
    let logger_performance = if LOG { Some(LoggerPerformance::new()) } else { None };
    let logger_pose = if LOG { Some(LoggerPose::new()) } else { None };
    let logger_altitude_dvl = if LOG { Some(LoggerAltitudeDvl::new()) } else { None };
    let logger_geometric_correction = if LOG { Some(LoggerGeometricCorrection::new()) } else { None };
    let logger_swath_raw = if LOG { Some(LoggerSwathRaw::new()) } else { None };
    let logger_swath_processed = if LOG { Some(LoggerSwathProcessed::new()) } else { None };
    // Initialize ROS2 (STOP) --------------------------------------------------



    // ROS2 Handlers (START) --------------------------------------------------
    // Spin ----------
    let handle = tokio::task::spawn_blocking(move || loop {
        node.spin_once(std::time::Duration::from_millis(10));
    });

    // State Estimate ----------
    let pose_clone = pose_shared.clone();

    let state_estimate_task = sub_state_estimate.for_each({
        move |msg| {
            // Acquire shared resource
            let mut pose = pose_clone.write().unwrap();

            // Save ROS2 message to shared resource for future processing
            // Position
            pose.position.x = msg.pose.pose.position.x;
            pose.position.y = msg.pose.pose.position.y;
            pose.position.z = msg.pose.pose.position.z;

            // Orientation
            let q = msg.pose.pose.orientation;
            let quat = UnitQuaternion::from_quaternion(Quaternion::new(q.w, q.x, q.y, q.z));
            let (roll, pitch, yaw) = quat.euler_angles();
            pose.orientation.roll = roll;
            pose.orientation.pitch = pitch;
            pose.orientation.yaw = yaw;

            return future::ready(());
        }
    });

    // DVL ----------
    let altitude_clone = altitude_shared.clone();

    let dvl_task = sub_dvl.for_each({
        move |msg| {
            // Ensure we only process relevant DVL data
            // DVL outputs multiple modes (e.g. water-track, bottom-track) and configurations
            // We only accept bottom-track measurements from a standard piston (Janus) DVL
            // since those correspond to seabed-referenced measurements used for altitude and 
            // sound speed under water
            if msg.velocity_mode != Dvl::DVL_MODE_BOTTOM as u8 {
                return future::ready(());
            }
            if msg.dvl_type != Dvl::DVL_TYPE_PISTON as u8 {
                return future::ready(());
            }

            // Altitude
            // When beam ranges are valid and at least one beam is good, the DVL provides
            // a seabed-referenced altitude estimate. This is the primary quantity used
            // for swath processing (e.g. height above seabed for correct projection).
            if msg.beam_ranges_valid && msg.num_good_beams > 0 {
                let mut altitude = altitude_clone.write().unwrap();
                altitude.value = msg.altitude;
            }

            return future::ready(());
        }
    });

    // Side Scan Sonar ----------
    let pose_clone = pose_shared.clone();
    let altitude_clone = altitude_shared.clone();

    // If logging is enabled, these logger instances are copied and transferred ownership to be used for data logging
    // Move loggers into the async closure (ownership transfer) so they can be mutably used inside "move" context
    let mut logger_performance = logger_performance;
    let mut logger_pose = logger_pose;
    let mut logger_altitude_dvl = logger_altitude_dvl;
    let mut logger_geometric_correction = logger_geometric_correction;
    let mut logger_swath_raw = logger_swath_raw;
    let mut logger_swath_processed = logger_swath_processed;

    let sonar_task = sub_side_scan_sonar.for_each({
        move |msg: RawSonarImage| {
            // If logging is enabled, start logging performance
            if let Some(l) = &mut logger_performance {
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

            let swath_raw = SwathRaw {
                timestamp: t,
                port,
                starboard,
                samples_per_beam: msg.samples_per_beam as u64,
            };

            // Acquire shared resources -----
            let pose = pose_clone.read().unwrap();
            let altitude = altitude_clone.read().unwrap();

            // Process Data -----
            let swath_processed = process_swath(
                &swath_raw,
                &pose,
                &altitude,
                &sonar,
                illumination_ema_period,
            );

            // Publish Data -----
            // Pose
            let mut pose_msg = PoseStamped::default();
            pose_msg.header.stamp = msg.header.stamp.clone();
            pose_msg.header.frame_id = "base_link".to_string();

            pose_msg.pose.position.x = swath_processed.pose_interpolated.position.x;
            pose_msg.pose.position.y = swath_processed.pose_interpolated.position.y;
            pose_msg.pose.position.z = swath_processed.pose_interpolated.position.z;

            let quat = UnitQuaternion::from_euler_angles(
                swath_processed.pose_interpolated.orientation.roll,
                swath_processed.pose_interpolated.orientation.pitch,
                swath_processed.pose_interpolated.orientation.yaw,
            );
            let q = quat.quaternion();
            pose_msg.pose.orientation.x = q.i;
            pose_msg.pose.orientation.y = q.j;
            pose_msg.pose.orientation.z = q.k;
            pose_msg.pose.orientation.w = q.w;

            let _ = pub_pose.publish(&pose_msg);

            // Geometric Correction
            // points[n] = (x         , y  ,      z) // Geometry description
            // points[0] = (r_fbr_port, 0.0, h_port) // Port geometry
            // points[1] = (r_fbr_stb , 0.0,  h_stb) // Starboard geometry
            let mut geometric_correction_msg = PolygonStamped::default();
            geometric_correction_msg.header.stamp = msg.header.stamp.clone();
            geometric_correction_msg.header.frame_id = "base_link".to_string();

            geometric_correction_msg.polygon.points = vec![
                Point32 {
                    x: swath_processed.geometric_correction_data.r_fbr_port as f32,
                    y: 0.0,
                    z: swath_processed.geometric_correction_data.h_port as f32,
                },
                Point32 {
                    x: swath_processed.geometric_correction_data.r_fbr_stb as f32,
                    y: 0.0,
                    z: swath_processed.geometric_correction_data.h_stb as f32,
                },
            ];

            let _ = pub_geometric_correction.publish(&geometric_correction_msg);

            // Swath Processed 
            let mut swath_processed_msg = RawSonarImage::default();
            swath_processed_msg.header = msg.header.clone();

            swath_processed_msg.ping_info.frequency = msg.ping_info.frequency;

            swath_processed_msg.samples_per_beam = msg.samples_per_beam;
            swath_processed_msg.sample0 = msg.sample0;

            swath_processed_msg.image.dtype = msg.image.dtype;
            swath_processed_msg.image.beam_count = msg.image.beam_count;
            swath_processed_msg.image.data = swath_processed
                .port.iter()
                .chain(&swath_processed.starboard)
                .copied()
                .collect();

            let _ = pub_swath.publish(&swath_processed_msg);

            // Log Data -----
            // If logging is enabled, log the following data
            if let Some(l) = &mut logger_performance {
                l.stop(t);
            }
            if let Some(l) = &mut logger_swath_raw {
                l.log(t, &swath_raw.port, &swath_raw.starboard);
            }
            if let Some(l) = &mut logger_pose {
                l.log(t, &swath_processed.pose_interpolated);
            }
            if let Some(l) = &mut logger_altitude_dvl {
                l.log(t, altitude.value);
            }
            if let Some(l) = &mut logger_geometric_correction {
                l.log(t, &swath_processed.geometric_correction_data);
            }
            if let Some(l) = &mut logger_swath_processed {
                l.log(t, &swath_processed.port, &swath_processed.starboard);
            }

            return future::ready(());
        }
    });
    // ROS2 Handlers (STOP) --------------------------------------------------



    // ROS2 (START) --------------------------------------------------
    tokio::join!(
        state_estimate_task, 
        dvl_task,
        sonar_task,
    );

    handle.await?;

    Ok(())
    // ROS2 (STOP) --------------------------------------------------
}