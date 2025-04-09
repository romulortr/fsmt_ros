
#include <fsmt_base_local_planner/utils.hpp>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>

void tf_transform_to_geometry_transform(tf::StampedTransform& tf_transform,
    geometry_msgs::TransformStamped& geom_transform){

    // Fill in the header
    geom_transform.header.stamp = tf_transform.stamp_;
    geom_transform.header.frame_id = tf_transform.frame_id_;
    geom_transform.child_frame_id = tf_transform.child_frame_id_;
    
    // Fill in the translation
    geom_transform.transform.translation.x = tf_transform.getOrigin().x();
    geom_transform.transform.translation.y = tf_transform.getOrigin().y();
    geom_transform.transform.translation.z = tf_transform.getOrigin().z();
    
    // Fill in the rotation
    geom_transform.transform.rotation.x = tf_transform.getRotation().x();
    geom_transform.transform.rotation.y = tf_transform.getRotation().y();
    geom_transform.transform.rotation.z = tf_transform.getRotation().z();
    geom_transform.transform.rotation.w = tf_transform.getRotation().w();
}

void tf_transform_to_fsmt_transform(tf::StampedTransform& tf_transform,
    fsmt_transform_t& fsmt_transform){
    // Fill in the translation.
    fsmt_transform.x = tf_transform.getOrigin().x();
    fsmt_transform.y = tf_transform.getOrigin().y();
    
    // Get rotation as quaternion.
    tf::Quaternion q = tf_transform.getRotation();
    // Extracts yaw. 
    float yaw = (float) tf::getYaw(q);
    // Fill in rotation.
    fsmt_transform.cos_yaw = cosf(yaw);
    fsmt_transform.sin_yaw = sinf(yaw);
        
}

// void tf_transform_to_fsmt_pose(const tf::StampedTransform& transform) {
//     pose.x = transform.getOrigin().x();
//     pose.y = transform.getOrigin().y();
//     pose.yaw = tf::getYaw(transform.getRotation());  // Convert quaternion to yaw
//     return pose;
// }