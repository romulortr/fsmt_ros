#ifndef FSMT_BASE_LOCAL_PLANNER_UTILS_H
#define FSMT_BASE_LOCAL_PLANNER_UTILS_H

#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>

void tf_transform_to_geometry_transform(tf::StampedTransform& tf_transform,
    geometry_msgs::TransformStamped& geom_transform);

#endif
