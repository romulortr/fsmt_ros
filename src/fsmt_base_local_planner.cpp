#include <fsmt_base_local_planner/fsmt_base_local_planner.hpp>
#include <fsmt/fsmt.h>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>

PLUGINLIB_EXPORT_CLASS(FSMTBaseLocalPlanner, nav_core::BaseLocalPlanner)

FSMTBaseLocalPlanner::FSMTBaseLocalPlanner() : initialized_(false) {}

void FSMTBaseLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        ROS_INFO("Initializing FSMT Base Local Planner...");
        initialized_ = true;
    }

    ros::NodeHandle nh("~/" + name);

    // Publisher
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
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
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;


    visualization_msgs::Marker points;
    points.header.frame_id = "base_link";  // Change frame ID if needed
    points.header.stamp = ros::Time::now();
    points.ns = "points";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;

    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;

    // Set scale for point size
    points.scale.x = 0.1;  // Width of points
    points.scale.y = 0.1;

    // Set color (RGBA)
    points.color.r = 1.0;
    points.color.g = 0.0;
    points.color.b = 0.0;
    points.color.a = 1.0;  // Fully visible

    fsmt_point_array_t samples;
    samples.points = (fsmt_point_t*) malloc(sizeof(fsmt_point_t)*100);
    samples.number_of_points = 0;
    samples.max_number_of_points = 100;

    float length = 1;
    float radius = 10;

    swept_volume(&samples, radius, length);

    ROS_INFO("Number of points: %ld", samples.number_of_points);

    // Add some points
    for (size_t i = 0; i < samples.number_of_points; i++) {
        geometry_msgs::Point p;
        p.x = samples.points[i].x;
        p.y = samples.points[i].y;  
        p.z = 0;

        points.points.push_back(p);
    }

    marker_pub_.publish(points);
    // ROS_INFO("Published points to RViz");
    free (samples.points);
    return true;
}

bool FSMTBaseLocalPlanner::isGoalReached() {
    return global_plan_.empty();
}

void FSMTBaseLocalPlanner::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Store the entire LaserScan message
    ros_laser_scan_ = *msg;

}