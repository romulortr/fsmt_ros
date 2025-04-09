#ifndef FSMT_BASE_LOCAL_PLANNER_UTILS_H
#define FSMT_BASE_LOCAL_PLANNER_UTILS_H

#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <fsmt/data_structure/transform.h>

void tf_transform_to_geometry_transform(tf::StampedTransform& tf_transform,
    geometry_msgs::TransformStamped& geom_transform);

void tf_transform_to_fsmt_transform(tf::StampedTransform& tf_transform,
    fsmt_transform_t& fsmt_transform);



#endif
