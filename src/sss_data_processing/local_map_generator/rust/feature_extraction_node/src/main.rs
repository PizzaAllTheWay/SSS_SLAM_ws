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
};
// Libraries for Swath processing ----------
use feature_extraction_node::feature_extraction_lib::types::{

};
use feature_extraction_node::feature_extraction_lib::extractor::{
    
};



#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize ROS2 (START) --------------------------------------------------
    // Shared variables ----------

    // Node ----------
    let ctx = Context::create()?;
    let mut node = Node::create(ctx, "feature_extraction_node", "")?;

    // Parameters ----------

    // Subscribers ----------

    // Publishers ----------
    // Initialize ROS2 (STOP) --------------------------------------------------



    // ROS2 Handlers (START) --------------------------------------------------
    // Spin ----------
    let handle = tokio::task::spawn_blocking(move || loop {
        node.spin_once(std::time::Duration::from_millis(10));
    });

    
    // ROS2 Handlers (STOP) --------------------------------------------------



    // ROS2 (START) --------------------------------------------------
    tokio::join!(
        // ! TODO
    );

    handle.await?;

    Ok(())
    // ROS2 (STOP) --------------------------------------------------
}