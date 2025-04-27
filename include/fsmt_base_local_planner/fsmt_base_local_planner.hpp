#ifndef FSMT_BASE_LOCAL_PLANNER_H
#define FSMT_BASE_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>

// Messages
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

// Core TF2
#include <tf/transform_listener.h>
// #include <tf2_ros/buffer.h>

// For tf2::doTransform
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <fsmt/data_structure/lidar.h>
#include <fsmt/navigation/navigation_with_global_wp_plan.h>

int ROS2FSMTLaserScan(const sensor_msgs::LaserScan& ros_laser_scan, fsmt_lidar_t* fsmt_lidar);

class FSMTBaseLocalPlanner : public nav_core::BaseLocalPlanner {
public:
    FSMTBaseLocalPlanner();
    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
    bool isGoalReached() override;

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void fsmt_point_array_to_marker(visualization_msgs::Marker &marker, 
        std::string frame_id, fsmt_cartesian_point_array_t *fsmt_array,
        float color[3]);

    bool GetTransform(std::string to_frame, std::string from_frame, 
        ros::Time timestamp, tf::StampedTransform &tf_transform);

private:
    ros::Subscriber laser_scan_sub_;
    ros::Publisher marker_fsmt_edge_pub_;
    ros::Publisher marker_fsmt_whisker_pub_;
    ros::Publisher marker_vehicle_at_final_time_pub_;
    ros::Publisher marker_local_goal_pub_;

    bool initialized_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    sensor_msgs::LaserScan ros_laser_scan_;  

    // tf2_ros::Buffer tf_buffer_;  // member variable
    fsmt_lidar_t *fsmt_lidar_;
    tf::TransformListener tf_listener_;

    fsmt_cartesian_point_array_t *plan_array_;

    fsmt_navigation_t *navigation_;

    bool tube_configured_;
    int recovery_mode_;
};

void fsmt_points_to_marker(visualization_msgs::Marker &marker, std::string frame_id, 
    fsmt_cartesian_point_t *points, size_t size, float color[3]);

#endif