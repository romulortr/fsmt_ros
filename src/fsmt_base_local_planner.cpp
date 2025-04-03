#include <fsmt_base_local_planner/fsmt_base_local_planner.hpp>
#include <fsmt/fsmt.h>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>

#include <fsmt/score.h>
#include <fsmt_base_local_planner/utils.hpp>

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
    fsmt_lidar_ = NULL;

    // FSMT memory allocation.
    plan_array_ = fsmt_cartesian_point_array_create(100);
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
    // wait for a global plan.
    if (global_plan_.empty()) return false;
    // wait for a valid lidar reading.
    if(ros_laser_scan_.ranges.empty()) return false;
    
    size_t number_of_beams = ros_laser_scan_.ranges.size();
    if(fsmt_lidar_ == NULL)
    {
        fsmt_lidar_ = fsmt_lidar_create(number_of_beams);
    }

    if(fsmt_lidar_->config.size.max < number_of_beams)
    {
        fsmt_lidar_destroy(&fsmt_lidar_);
        fsmt_lidar_ = fsmt_lidar_create(number_of_beams);
    }

    // The global plan must be transformed from its current frame (e.g., map or odom frame)
    // to the robot frame (e.g., base_link).
    // First, get the transform from global plan frame to robot frame.
    tf::StampedTransform tf_transform;
    GetTransform("base_link", global_plan_.front().header.frame_id, 
        global_plan_.front().header.stamp, tf_transform);
    // Next we convert the message from TF standard to geometry message.
    geometry_msgs::TransformStamped geom_transform;
    tf_transform_to_geometry_transform(tf_transform, geom_transform);
    // Transform global to robot frame.
    size_t *number_of_points = &plan_array_->size;
    float distance=0, distance_to_last_point=0;
    plan_array_->size = 0;
    for (size_t i=1; i<global_plan_.size(); i++ ) {
        float dx = global_plan_[i].pose.position.x-global_plan_[i-1].pose.position.x;
        float dy = global_plan_[i].pose.position.y-global_plan_[i-1].pose.position.y;
        distance += sqrtf(dx*dx+dy*dy);

        if (distance - distance_to_last_point > 0.1){
            geometry_msgs::PoseStamped pose_base_link;
            tf2::doTransform(global_plan_[i], pose_base_link, geom_transform);  // Using the pre-fetched transform
            plan_array_->points[*number_of_points].x = pose_base_link.pose.position.x;
            plan_array_->points[*number_of_points].y = pose_base_link.pose.position.y;
            *number_of_points += 1;
            distance_to_last_point = distance;
        }
        if (distance > 1){
            break;
        }
    } 

    fsmt_circle_t circle;
    fsmt_circle_fitting_kasa(plan_array_, &circle);

    cmd_vel.linear.x = 1.;
    if(fabs(circle.radius) > 10 ){
        cmd_vel.angular.z = 0.0;
    }else{
        cmd_vel.angular.z = cmd_vel.linear.x/circle.radius;
    }

    float length = 1;
    float radius = circle.radius;

    fsmt_cartesian_point_array_t samples;
    samples.points = (fsmt_cartesian_point_t*) malloc(sizeof(fsmt_cartesian_point_t)*100);
    samples.size = 0;
    samples.max_size = 100;

    swept_volume(&samples, radius, length);

    ROS2FSMTLaserScan(ros_laser_scan_, fsmt_lidar_);

    // Add some points
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

    for (size_t i = 0; i < samples.size; i++) {
        geometry_msgs::Point p;
        p.x = samples.points[i].x;
        p.y = samples.points[i].y;  
        p.z = 0;

        points.points.push_back(p);
    }

    marker_pub_.publish(points);

    free (samples.points);


    return true;
}

bool FSMTBaseLocalPlanner::isGoalReached() {
    return global_plan_.empty();
}

void FSMTBaseLocalPlanner::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    ros_laser_scan_ = *msg;
}

int ROS2FSMTLaserScan(const sensor_msgs::LaserScan& ros_laser_scan, fsmt_lidar_t* fsmt_lidar)
{
    if(ros_laser_scan.ranges.size() > fsmt_lidar->config.size.max)
    {
        return -1;
    }

    // Copying measurements.
    size_t number_of_beams = ros_laser_scan.ranges.size();
    const std::vector<float>& ros_ranges = ros_laser_scan.ranges;  // Avoid repeated access
    float *fsmt_ranges = fsmt_lidar->measurements.ranges;
    for (size_t i = 0; i < number_of_beams; ++i) {
        fsmt_ranges[i] = ros_ranges[i];
    }
    fsmt_lidar->measurements.size = number_of_beams;
    
    // Copy configuration.
    fsmt_lidar_config(fsmt_lidar, 
        ros_laser_scan.angle_min, 
        ros_laser_scan.angle_max, 
        ros_laser_scan.range_min, 
        ros_laser_scan.range_max, 
        ros_laser_scan.angle_increment);

    return 1;
}

bool FSMTBaseLocalPlanner::GetTransform(std::string to_frame, 
    std::string from_frame, ros::Time timestamp, tf::StampedTransform tf_transform)
{
    try{
        // Fetch the transform from "odom" frame to "base_link" frame
        tf_listener_.lookupTransform(to_frame, from_frame, 
            timestamp, tf_transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("Could not get transform: %s", ex.what());
        return false;
    }
}



//DESTROY
//     fsmt_cartesian_point_array_destroy(&plan_array);

    // cartesian_to_polar(&samples, &polar, fsmt_lidar_);
    // int is_available = availability(&polar, fsmt_lidar_);
    // ROS_INFO("Number of points in polar: %ld", polar.size);
    // ROS_INFO("is available: %d", is_available);
    // for (size_t i=0; i< polar.size; i++) {
    //     size_t index = polar.ranges[i].index;
    //     // ROS_INFO("Index %ld: range min = %f, range: %f", index, polar.ranges[i].range1, ros_laser_scan_.ranges[index]);
    // }

    // fsmt_sensor_array_t polar;
    // polar.ranges = (fsmt_range_t*) malloc(sizeof(fsmt_range_t)*100);
    // polar.size = 0;
    // polar.max_size = 100;


    // if(is_available == 1){
    //     cmd_vel.linear.x = 0.0;
    //     cmd_vel.angular.z = 0.0;
    // }else{
    // free (polar.ranges);
