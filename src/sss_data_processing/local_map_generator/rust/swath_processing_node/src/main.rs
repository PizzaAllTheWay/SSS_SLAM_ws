// Libraries for conversions ----------
use nalgebra::{
    Quaternion, 
    UnitQuaternion
};
// Libraries for realtime ----------
use std::sync::{Arc, RwLock};
use futures::{
    future, 
    stream::StreamExt
};
// Libraries for ROS2 ----------
use r2r::{
    QosProfile, 
    nav_msgs::msg::Odometry, 
    marine_acoustic_msgs::msg::Dvl, 
    marine_acoustic_msgs::msg::RawSonarImage,
    geometry_msgs::msg::PoseStamped,
    geometry_msgs::msg::PointStamped,
};
// Libraries for Swath processing ----------
use swath_processing_node::swath_processing_lib::types::{
    Position,
    Orientation,
    Pose3D,
    AltitudeMeasurement,
    SoundSpeed,
    SwathRaw,
};
use swath_processing_node::swath_processing_lib::processor::{
    process_swath,
};
use swath_processing_node::swath_processing_lib::utils::{
    LoggerPose,
    LoggerAltitude,
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
    let sound_speed_shared = Arc::new(RwLock::new(SoundSpeed {
        value: 1500.0,
    }));

    // Node ----------
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "swath_processing_node", "")?;

    // Parameters ----------
    #[allow(non_snake_case)]
    let LOG: bool = node.get_parameter::<bool>("log").ok().unwrap_or(false) as bool;
    let max_range: f32 = node.get_parameter::<f64>("swath_processing.max_range").ok().unwrap_or(0.0) as f32;

    r2r::log_info!("swath_processing_node", "log: {}", LOG);
    r2r::log_info!("swath_processing_node", "swath_processing.max_range: {:.2}", max_range);

    // Subscribers ----------
    let sub_state_estimate = node.subscribe::<Odometry>("/sss_slam/data_processing/estimate/state", QosProfile::default())?;
    let sub_dvl = node.subscribe::<Dvl>("/hardware/dvl", QosProfile::default())?;
    let sub_side_scan_sonar = node.subscribe::<RawSonarImage>("/hardware/side_scan_sonar", QosProfile::default())?;

    // Publishers ----------
    let pub_pose = node.create_publisher::<PoseStamped>("/sss_slam/data_processing/swath/pose", QosProfile::default())?;
    let pub_altitude = node.create_publisher::<PointStamped>("/sss_slam/data_processing/swath/altitude", QosProfile::default())?;
    let pub_swath = node.create_publisher::<RawSonarImage>("/sss_slam/data_processing/swath/processed", QosProfile::default())?;
    
    // Loggers ----------
    // If logging is enabled, these logger instances are created and used for data logging
    let logger_pose = if LOG { Some(LoggerPose::new()) } else { None };
    let logger_altitude = if LOG { Some(LoggerAltitude::new()) } else { None };
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
    let sound_speed_clone = sound_speed_shared.clone();

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

            // CASE 1: Altitude (preferred measurement)
            // When beam ranges are valid and at least one beam is good, the DVL provides
            // a seabed-referenced altitude estimate. This is the primary quantity used
            // for swath processing (e.g. height above seabed for correct projection).
            if msg.beam_ranges_valid && msg.num_good_beams > 0 {
                let mut altitude = altitude_clone.write().unwrap();
                altitude.value = msg.altitude;
            }

            // CASE 2: Sound speed (environmental parameter)
            // Some DVL packets contain only the speed of sound in water. This is not a
            // geometric measurement, but is required for accurate sonar range conversion
            // and acoustic propagation modeling.
            if msg.sound_speed > 0.0 {
                let mut sound_speed = sound_speed_clone.write().unwrap();
                sound_speed.value = msg.sound_speed;
            }

            return future::ready(());
        }
    });

    // Side Scan Sonar ----------
    let pose_clone = pose_shared.clone();
    let altitude_clone = altitude_shared.clone();
    let sound_clone = sound_speed_shared.clone();

    let pub_swath = pub_swath.clone();

    // If logging is enabled, these logger instances are copied and transferred ownership to be used for data logging
    // Move loggers into the async closure (ownership transfer) so they can be mutably used inside "move" context
    let mut logger_pose = logger_pose;
    let mut logger_altitude = logger_altitude;
    let mut logger_swath_raw = logger_swath_raw;
    let mut logger_swath_processed = logger_swath_processed;

    let sonar_task = sub_side_scan_sonar.for_each({
        move |msg: RawSonarImage| {
            let t = msg.header.stamp.sec as f64 + msg.header.stamp.nanosec as f64 * 1e-9;

            // Extract sonar data -----
            // Fast path explanation:
            // - Vec::with_capacity(n) allocates memory but does NOT initialize it (len = 0)
            // - set_len(n) marks the buffer as fully sized WITHOUT writing zeros (avoids unnecessary work)
            // - copy_from_slice then fills the memory in one go
            // Compared to data[..n].to_vec():
            // - both allocate + copy
            // - but this avoids implicit initialization / potential extra work
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
                timestamp: msg.header.stamp.sec as f64 + msg.header.stamp.nanosec as f64 * 1e-9,
                port,
                starboard,
                samples_per_beam: msg.samples_per_beam,
                max_range: max_range,
            };

            // Acquire shared resources -----
            let pose = pose_clone.read().unwrap();
            let altitude = altitude_clone.read().unwrap();
            let sound_speed = sound_clone.read().unwrap();

            // Process Data -----
            let swath_processed = process_swath(&swath_raw, &pose, &altitude, &sound_speed);

            // Publish Data -----
            // Pose
            let mut pose_msg = PoseStamped::default();
            pose_msg.header.stamp = msg.header.stamp.clone();
            pose_msg.header.frame_id = "base_link".to_string();

            pose_msg.pose.position.x = swath_processed.pose.position.x;
            pose_msg.pose.position.y = swath_processed.pose.position.y;
            pose_msg.pose.position.z = swath_processed.pose.position.z;

            let quat = UnitQuaternion::from_euler_angles(
                swath_processed.pose.orientation.roll,
                swath_processed.pose.orientation.pitch,
                swath_processed.pose.orientation.yaw,
            );
            let q = quat.quaternion();
            pose_msg.pose.orientation.x = q.i;
            pose_msg.pose.orientation.y = q.j;
            pose_msg.pose.orientation.z = q.k;
            pose_msg.pose.orientation.w = q.w;

            let _ = pub_pose.publish(&pose_msg);

            // Altitude 
            let mut altitude_msg = PointStamped::default();
            altitude_msg.header.stamp = msg.header.stamp.clone();
            altitude_msg.header.frame_id = "base_link".to_string();

            altitude_msg.point.z = swath_processed.altitude.value;

            let _ = pub_altitude.publish(&altitude_msg);

            // Swath Processed 
            let mut swath_processed_msg = RawSonarImage::default();
            swath_processed_msg.header = msg.header.clone();

            swath_processed_msg.ping_info.frequency = msg.ping_info.frequency;
            swath_processed_msg.ping_info.sound_speed = sound_speed.value;

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
            if let Some(l) = &mut logger_swath_raw {
                l.log(t, &swath_raw.port, &swath_raw.starboard);
            }
            if let Some(l) = &mut logger_pose {
                l.log(t, &swath_processed.pose);
            }
            if let Some(l) = &mut logger_altitude {
                l.log(t, swath_processed.altitude.value);
            }
            if let Some(l) = &mut logger_swath_processed {
                l.log(t, &swath_processed.port, &swath_processed.starboard);
            }

            // ! Debugging
            r2r::log_info!("swath_processing_node", "Swath Processed at timestamp: {:.4}", t);

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