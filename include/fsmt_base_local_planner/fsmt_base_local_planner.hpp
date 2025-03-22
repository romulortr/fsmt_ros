#ifndef FSMT_BASE_LOCAL_PLANNER_H
#define FSMT_BASE_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>

#include <fsmt/data_structure/lidar.h>

int ROS2FSMTLaserScan(const sensor_msgs::LaserScan& ros_laser_scan, fsmt_lidar_t* fsmt_lidar);

class FSMTBaseLocalPlanner : public nav_core::BaseLocalPlanner {
public:
    FSMTBaseLocalPlanner();
    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
    bool isGoalReached() override;

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

private:
    ros::Subscriber laser_scan_sub_;
    ros::Publisher marker_pub_;
    bool initialized_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    sensor_msgs::LaserScan ros_laser_scan_;  

    fsmt_lidar_t *fsmt_lidar_;
};

#endif