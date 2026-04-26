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
    geometry_msgs::msg::PoseStamped,
    geometry_msgs::msg::PointStamped,
    sensor_msgs::msg::Image,
    sensor_msgs::msg::PointCloud2,
};
// Libraries for Swath processing ----------
use feature_extraction_node::feature_extraction_lib::types::{
    Altitude,
    Position,
    Orientation,
    Pose3D,
    Pose2DMap,
    Map,
    Landmark,
};
use feature_extraction_node::feature_extraction_lib::extractor::FeatureExtractor;
use feature_extraction_node::feature_extraction_lib::utils::{
    LoggerPerformance,
    LoggerMapPose,
    LoggerLandmarks,
};



#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize ROS2 (START) --------------------------------------------------
    // Shared variables ----------
    let altitude_shared = Arc::new(RwLock::new(Altitude {
        value: 0.0,
    }));
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
    let origin_shared = Arc::new(RwLock::new(
        Pose2DMap {
            x: 0,
            y: 0,

            yaw: 0.0,
        }
    ));

    // Node ----------
    let ctx = Context::create()?;
    let mut node = Node::create(ctx, "feature_extraction_node", "")?;

    // Parameters ----------
    #[allow(non_snake_case)]
    // ! TRUE LOG ! let LOG = node.get_parameter::<bool>("log").ok().unwrap_or(false) as bool;
    let LOG = false; // ! THIS IS FAKE LOG FOR DEBUGGING, REMOVE IT LATER AND UNCOMMENT THE "! TRUE LOG !" TO GO BACK TO NORMAL
    
    let filter_d = node.get_parameter::<i64>("local_map_generator.filter_d").ok().unwrap_or(0) as i32;
    let filter_sigma_color = node.get_parameter::<f64>("local_map_generator.filter_sigma_color").ok().unwrap_or(0.0);
    let filter_sigma_space = node.get_parameter::<f64>("local_map_generator.filter_sigma_space").ok().unwrap_or(0.0);
    let local_window_size = node.get_parameter::<i64>("local_map_generator.local_window_size").ok().unwrap_or(0) as i32;
    let local_offset = node.get_parameter::<f64>("local_map_generator.local_offset").ok().unwrap_or(0.0);
    let search_radius = node.get_parameter::<i64>("local_map_generator.search_radius").ok().unwrap_or(0) as i32;
    let min_support = node.get_parameter::<i64>("local_map_generator.min_support").ok().unwrap_or(0) as i32;
    let landmark_area_min = node.get_parameter::<i64>("local_map_generator.landmark_area_min").ok().unwrap_or(0) as i32;
    let sigma_r = node.get_parameter::<f64>("local_map_generator.sigma_r").ok().unwrap_or(0.0);
    let sigma_theta_deg = node.get_parameter::<f64>("local_map_generator.sigma_theta").ok().unwrap_or(0.0);
    let alpha_r = node.get_parameter::<f64>("local_map_generator.alpha_r").ok().unwrap_or(0.0);
    let alpha_theta = node.get_parameter::<f64>("local_map_generator.alpha_theta").ok().unwrap_or(0.0);
    let height_estimation_enabled = node.get_parameter::<bool>("local_map_generator.height_estimation_enabled").ok().unwrap_or(true);
    let shadow_area_min_ratio = node.get_parameter::<f64>("local_map_generator.shadow_area_min_ratio").ok().unwrap_or(0.0);
    let shadow_area_max_ratio = node.get_parameter::<f64>("local_map_generator.shadow_area_max_ratio").ok().unwrap_or(0.0);
    let shadow_threshold_bias = node.get_parameter::<f64>("local_map_generator.shadow_threshold_bias").ok().unwrap_or(0.0);
    let alpha_height_estimate = node.get_parameter::<f64>("local_map_generator.alpha_height_estimate").ok().unwrap_or(0.0);

    let map_resolution = node.get_parameter::<f64>("local_map_generator.map_resolution").ok().unwrap_or(0.0);
    let map_offset_yaw_deg = node.get_parameter::<f64>("local_map_generator.map_offset_yaw_deg").ok().unwrap_or(0.0);

    r2r::log_info!("feature_extraction_node", "log: {}", LOG);
    r2r::log_info!("feature_extraction_node", "");
    r2r::log_info!("feature_extraction_node", "local_map_generator.filter_d:                  {}    [pixel]    ",                  filter_d);
    r2r::log_info!("feature_extraction_node", "local_map_generator.filter_sigma_color:        {:.2}            ",        filter_sigma_color);
    r2r::log_info!("feature_extraction_node", "local_map_generator.filter_sigma_space:        {:.2}            ",        filter_sigma_space);
    r2r::log_info!("feature_extraction_node", "local_map_generator.local_window_size:         {}    [pixel]    ",         local_window_size);
    r2r::log_info!("feature_extraction_node", "local_map_generator.local_offset:              {:.4}            ",              local_offset);
    r2r::log_info!("feature_extraction_node", "local_map_generator.search_radius:             {}    [pixel]    ",             search_radius);
    r2r::log_info!("feature_extraction_node", "local_map_generator.min_support:               {}    [pixel]    ",               min_support);
    r2r::log_info!("feature_extraction_node", "local_map_generator.landmark_area_min:         {}    [pixel]    ",         landmark_area_min);
    r2r::log_info!("feature_extraction_node", "local_map_generator.sigma_r:                   {:.4} [m]        ",                   sigma_r);
    r2r::log_info!("feature_extraction_node", "local_map_generator.sigma_theta:               {:.4} [deg]      ",           sigma_theta_deg);
    r2r::log_info!("feature_extraction_node", "local_map_generator.alpha_r:                   {:.4}            ",                   alpha_r);
    r2r::log_info!("feature_extraction_node", "local_map_generator.alpha_theta:               {:.4}            ",               alpha_theta);
    r2r::log_info!("feature_extraction_node", "local_map_generator.height_estimation_enabled: {}               ", height_estimation_enabled);
    r2r::log_info!("feature_extraction_node", "local_map_generator.shadow_area_min_ratio:     {:.4}            ",     shadow_area_min_ratio);
    r2r::log_info!("feature_extraction_node", "local_map_generator.shadow_area_max_ratio:     {:.4}            ",     shadow_area_max_ratio);
    r2r::log_info!("feature_extraction_node", "local_map_generator.shadow_threshold_bias:     {:.4} [intensity]",     shadow_threshold_bias);
    r2r::log_info!("feature_extraction_node", "local_map_generator.alpha_height_estimate:     {:.4}            ",     alpha_height_estimate);
    r2r::log_info!("feature_extraction_node", "");
    r2r::log_info!("feature_extraction_node", "local_map_generator.map_resolution:     {} [m/pixel]",     map_resolution);
    r2r::log_info!("feature_extraction_node", "local_map_generator.map_offset_yaw_deg: {} [deg]    ", map_offset_yaw_deg);

    // Initialize Feature Extraction ----------
    let feature_extractor_shared = Arc::new(RwLock::new(FeatureExtractor::new(
        filter_d,
        filter_sigma_color,
        filter_sigma_space,

        local_window_size,
        local_offset,
        search_radius,
        min_support,

        landmark_area_min,

        sigma_r,
        sigma_theta_deg.to_radians(),
        alpha_r,
        alpha_theta,

        height_estimation_enabled,
        shadow_area_min_ratio,
        shadow_area_max_ratio,
        shadow_threshold_bias,
        alpha_height_estimate,

        map_offset_yaw_deg.to_radians(),
    )));

    // Subscribers ----------
    let sub_map_altitude = node.subscribe::<PointStamped>("/sss_slam/data_processing/map_generation/altitude", QosProfile::default())?;
    let sub_map_pose = node.subscribe::<PoseStamped>("/sss_slam/data_processing/map_generation/pose", QosProfile::default())?;
    let sub_map_origin = node.subscribe::<PoseStamped>("/sss_slam/data_processing/map_generation/origin", QosProfile::default())?;
    let sub_map = node.subscribe::<Image>("/sss_slam/data_processing/map_generation/map", QosProfile::default())?;

    // Publishers ----------
    let pub_map_pose = node.create_publisher::<PoseStamped>("/sss_slam/data_processing/feature_extraction/pose", QosProfile::default())?;
    let pub_landmarks = node.create_publisher::<PointCloud2>("/sss_slam/data_processing/feature_extraction/landmarks", QosProfile::default())?;

    // Loggers ----------
    // If logging is enabled, these logger instances are created and used for data logging
    let logger_performance_shared = Arc::new(RwLock::new(if LOG { Some(LoggerPerformance::new()) } else { None }));
    let logger_map_pose_shared = Arc::new(RwLock::new(if LOG { Some(LoggerMapPose::new()) } else { None }));
    let logger_landmarks_shared = Arc::new(RwLock::new(if LOG { Some(LoggerLandmarks::new()) } else { None }));
    // Initialize ROS2 (STOP) --------------------------------------------------



    // ROS2 Handlers (START) --------------------------------------------------
    // Spin ----------
    let handle = tokio::task::spawn_blocking(move || loop {
        node.spin_once(std::time::Duration::from_millis(10));
    });

    // Map Altitude ----------
    let altitude_clone = altitude_shared.clone();
    let map_altitude_task = sub_map_altitude.for_each({
        move |msg| {
            {
                // Acquire shared resource
                let mut altitude = altitude_clone.write().unwrap();

                // Write message data to shared resource
                altitude.value = msg.point.z;
            }
            
            return future::ready(());
        }
    });

    // Map Pose ----------
    let pose_clone = pose_shared.clone();
    let map_pose_task = sub_map_pose.for_each({
        move |msg| {
            // Acquire shared resource
            let mut pose = pose_clone.write().unwrap();

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

    // Map Origin ----------
    let origin_clone = origin_shared.clone();
    let map_origin_task = sub_map_origin.for_each({
        move |msg| {
            // Acquire shared resource
            let mut origin = origin_clone.write().unwrap();

            // Position
            origin.x = msg.pose.position.x as i64;
            origin.y = msg.pose.position.y as i64;

            // Orientation
            let q = msg.pose.orientation;
            let quat = UnitQuaternion::from_quaternion(Quaternion::new(q.w, q.x, q.y, q.z));
            let (_roll, _pitch, yaw) = quat.euler_angles();
            origin.yaw = yaw;
            
            return future::ready(());
        }
    });

    // Map ----------
    let altitude_clone = altitude_shared.clone();
    let pose_clone = pose_shared.clone();
    let origin_clone = origin_shared.clone();

    let feature_extractor_clone = feature_extractor_shared.clone();

    let logger_performance_clone = logger_performance_shared.clone();
    let logger_map_pose_clone = logger_map_pose_shared.clone();
    let logger_landmarks_clone = logger_landmarks_shared.clone();

    let map_task = sub_map.for_each({
        move |msg| {
            if let Some(l) = &mut *logger_performance_clone.write().unwrap() {
                l.start();
            }

            // Acquire shared resource
            let altitude = *altitude_clone.read().unwrap();
            let pose = *pose_clone.read().unwrap();
            let origin = *origin_clone.read().unwrap();

            // Build Map data structure
            let width = msg.width as usize;
            let height = msg.height as usize;
            let step = msg.step as usize;

            let mut data = vec![vec![0u8; width]; height];

            for y in 0..height {
                let row_start = y * step;
                let row_end = row_start + width;
                data[y].copy_from_slice(&msg.data[row_start..row_end]);
            }

            let map = Map {
                pose: origin,
                resolution: map_resolution,
                width,
                height,
                data,
            };

            // Extract Features -----
            let landmark_set = {
                let mut extractor = feature_extractor_clone.write().unwrap();

                match extractor.extract_features_from_map(
                    &map,
                    &pose,
                    &altitude
                ) {
                    Ok(landmark_set) => Some(landmark_set),
                    Err(_) => None,
                }
            };

            // Publish -----
            // At this point the feature extractor has either returned a valid LandmarkSet or failed/skipped.
            // We only publish when a valid LandmarkSet exists, because the pose and landmark message should
            // represent one complete feature-extraction result for this map frame.
            //
            // Two topics are published:
            //
            // 1) Current map/vehicle pose as PoseStamped.
            //    This gives downstream nodes the pose that was used when extracting the landmarks.
            //    The landmark measurements themselves are stored relative to this current map/vehicle origin,
            //    so publishing the pose together with the same timestamp makes later debugging, plotting,
            //    data association, and SLAM integration easier.
            //
            // 2) Full landmark set as one PointCloud2 message.
            //    This is not used as a normal XYZ point cloud. Instead, PointCloud2 is used as a standard
            //    ROS2 table-like binary container. Each landmark becomes one record/"point", and each field
            //    inside that record stores one scalar landmark value such as label ID, range, bearing,
            //    covariance, height, and descriptors.
            //
            // Keeping all landmarks in one message is useful because the whole set belongs to the same map
            // frame and timestamp. This avoids publishing one small message per landmark, avoids ambiguity
            // about which landmarks were extracted together, and keeps synchronization simple.
            //
            // A custom LandmarkSet.msg would be cleaner semantically, but then every machine that wants to
            // echo, bag, replay, visualize, or subscribe to this topic must also build and source that custom
            // message package. For testing, logging, and quick inspection, PointCloud2 is more portable.
            if let Some(landmark_set) = landmark_set.as_ref() {
                // Pose ----------
                let mut pose_msg = PoseStamped::default();
                pose_msg.header.stamp = msg.header.stamp.clone();
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

                // Landmark field layout ----------
                // These names define the fixed schema of one landmark record inside the PointCloud2 message.
                // Every landmark is packed with exactly these fields, in exactly this order.
                //
                // Each field is stored as FLOAT64, even values like `label_id` and `area`.
                // This keeps the record layout simple because every field has the same 8-byte size.
                // Consumers only need to know that field i starts at byte offset i * 8.
                let field_names = [
                    "label_id",
                    "z_r",
                    "z_theta",
                    "R_z_rr",
                    "R_z_rtheta",
                    "R_z_thetar",
                    "R_z_thetatheta",
                    "estimated_height",
                    "estimated_height_std",
                    "mean_intensity",
                    "std",
                    "contrast",
                    "entropy",
                    "area",
                    "weak_polar_r",
                    "weak_polar_theta",
                    "height_value",
                    "height_std",
                    "radial_intensity_gradient",
                ];

                // Build PointCloud2 field descriptors ----------
                // The `fields` vector tells ROS how to interpret the raw byte buffer.
                // Since every value is encoded as f64, each field has:
                // - datatype = 8, meaning FLOAT64 in sensor_msgs/PointField
                // - count = 1, meaning one scalar value
                // - offset = i * 8, because f64 takes 8 bytes
                let mut fields = Vec::new();

                for (i, name) in field_names.iter().enumerate() {
                    fields.push(r2r::sensor_msgs::msg::PointField {
                        name: name.to_string(),
                        offset: (i * 8) as u32,
                        datatype: 8, // FLOAT64
                        count: 1,
                    });
                }

                // Number of bytes used by one complete landmark record.
                let point_step = (field_names.len() * 8) as u32;

                // Pack landmark records ----------
                // `data` is the raw PointCloud2 byte buffer.
                // Each iteration appends one full landmark record to the end of the buffer.
                // The order of values pushed here must match `field_names` above.
                let mut data = Vec::<u8>::new();

                for (label_id, landmark) in &landmark_set.landmarks {
                    push_landmark_as_pointcloud_record(
                        &mut data,
                        *label_id,
                        landmark,
                    );
                }

                // Build Landmark PointCloud2 message ----------
                let mut landmark_msg = PointCloud2::default();
                landmark_msg.header.stamp = msg.header.stamp.clone();
                landmark_msg.header.frame_id = "map".to_string();

                // Use a 1D unordered cloud/table layout.
                // height = 1 means this is one row.
                // width = number of landmarks means one record per landmark.
                landmark_msg.height = 1;
                landmark_msg.width = landmark_set.landmarks.len() as u32;

                // Attach field schema and binary landmark data.
                landmark_msg.fields = fields;
                landmark_msg.is_bigendian = false;
                landmark_msg.point_step = point_step;
                landmark_msg.row_step = point_step * landmark_msg.width;
                landmark_msg.data = data;

                // true means there are no invalid/missing records in this message.
                landmark_msg.is_dense = true;

                let _ = pub_landmarks.publish(&landmark_msg);
            }

            // Log data -----
            // Always log performance if LOG is enabled
            // Only log data if we have some features extracted and LOG is enabled
            let t = msg.header.stamp.sec as f64 + msg.header.stamp.nanosec as f64 * 1e-9;

            if let Some(l) = &mut *logger_performance_clone.write().unwrap() {
                l.stop(t);
            }
            
            if let Some(landmark_set) = landmark_set.as_ref() {
                if let Some(l) = &mut *logger_map_pose_clone.write().unwrap() {
                    l.log(t, &pose);
                }
                if let Some(l) = &mut *logger_landmarks_clone.write().unwrap() {
                    l.log(t, landmark_set);
                }
            }
            
            return future::ready(());
        }
    });
    // ROS2 Handlers (STOP) --------------------------------------------------



    // ROS2 (START) --------------------------------------------------
    tokio::join!(
        map_altitude_task,
        map_pose_task,
        map_origin_task,
        map_task,
    );

    handle.await?;

    Ok(())
    // ROS2 (STOP) --------------------------------------------------
}



// Helper Functions (START) --------------------------------------------------
fn push_f64(data: &mut Vec<u8>, value: f64) {
    data.extend_from_slice(&value.to_le_bytes());
}

fn push_i32_as_f64(data: &mut Vec<u8>, value: i32) {
    data.extend_from_slice(&(value as f64).to_le_bytes());
}

// Packs one landmark into one PointCloud2 record.
//
// Big picture:
// PointCloud2 stores point data as a flat byte buffer, where the `fields` array later describes
// what each value means and where it lives inside each record. Here we therefore serialize every
// landmark property as one f64 field and append them in a fixed order. This makes the topic easy
// to inspect with standard ROS2 tools without requiring a custom message package.
//
// A custom Landmark.msg would be cleaner semantically, but it also means every machine that wants
// to echo, visualize, bag, replay, or subscribe to the topic must build and source that custom
// interface package first. For debugging, logging, bag analysis, and quick external inspection,
// using PointCloud2 is more portable because it is already supported by normal ROS2 tooling.
fn push_landmark_as_pointcloud_record(data: &mut Vec<u8>, label_id: i32, landmark: &Landmark) {
    // Landmark ID.
    // Store the connected-component label as part of the record so the same landmark can be
    // identified in plots, logs, and debugging tools.
    push_i32_as_f64(data, label_id);

    // Landmark measurement and covariance.
    // `z` stores the measured polar position of the landmark relative to the current map/vehicle origin,
    // while `R_z` stores the 2x2 measurement uncertainty matrix used later for weighting/matching.
    push_f64(data, landmark.z.r);
    push_f64(data, landmark.z.theta);
    push_f64(data, landmark.R_z[(0, 0)]);
    push_f64(data, landmark.R_z[(0, 1)]);
    push_f64(data, landmark.R_z[(1, 0)]);
    push_f64(data, landmark.R_z[(1, 1)]);

    // Estimated landmark height.
    // Store both the rough shadow-based height estimate and its heuristic standard deviation.
    push_f64(data, landmark.estimated_height.value);
    push_f64(data, landmark.estimated_height.std);

    // Landmark descriptors.
    // Store appearance, geometry, and weak contextual descriptors in the same record so each
    // landmark is self-contained when published, logged, or inspected later.
    push_f64(data, landmark.d.strong.mean_intensity);
    push_f64(data, landmark.d.strong.std);
    push_f64(data, landmark.d.strong.contrast);
    push_f64(data, landmark.d.strong.entropy);
    push_i32_as_f64(data, landmark.d.weak.area);
    push_f64(data, landmark.d.weak.polar_coordinates.r);
    push_f64(data, landmark.d.weak.polar_coordinates.theta);
    push_f64(data, landmark.d.weak.height_value);
    push_f64(data, landmark.d.weak.height_std);
    push_f64(data, landmark.d.weak.radial_intensity_gradient);
}
// Helper Functions (STOP) --------------------------------------------------
