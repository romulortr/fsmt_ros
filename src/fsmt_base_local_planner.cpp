#include <fsmt_base_local_planner/fsmt_base_local_planner.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(FSMTBaseLocalPlanner, nav_core::BaseLocalPlanner)

FSMTBaseLocalPlanner::FSMTBaseLocalPlanner() : initialized_(false) {}

void FSMTBaseLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        ROS_INFO("Initializing FSMT Base Local Planner...");
        initialized_ = true;
    }

    ros::NodeHandle nh("~/" + name);
    
    // Subscribe to LiDAR scan topic
    laser_scan_sub_ = nh.subscribe("/front/scan", 1, &FSMTBaseLocalPlanner::lidarCallback, this);

}

bool FSMTBaseLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
    if (plan.empty()) {
        ROS_WARN("Received an empty plan!");
        return false;
    }

    global_plan_ = plan;
    return true;
}

bool FSMTBaseLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    if (global_plan_.empty()) return false;

    // Example: Move forward with a constant velocity
    if(ros_laser_scan_.ranges.empty()) return false;


    ROS_INFO("ros_laser_scan_.ranges[(int) ros_laser_scan_.ranges.size()/2]: %f\n", ros_laser_scan_.ranges[(int) ros_laser_scan_.ranges.size()/2]);
    if(ros_laser_scan_.ranges[(int) ros_laser_scan_.ranges.size()/2] > 1)
        cmd_vel.linear.x = 0.2;
    else
        cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0.0;
    return true;
}

bool FSMTBaseLocalPlanner::isGoalReached() {
    return global_plan_.empty();
}

void FSMTBaseLocalPlanner::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Store the entire LaserScan message
    ros_laser_scan_ = *msg;

}