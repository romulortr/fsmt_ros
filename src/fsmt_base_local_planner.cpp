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
    fsmt_lidar_ = NULL;
}

bool FSMTBaseLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
    if (plan.empty()) {
        ROS_WARN("Received an empty plan!");
        return false;
    }
    std::cout << "RECEIVED NEW PLAN" << std::endl;
    tf::StampedTransform tf_transform;
    geometry_msgs::TransformStamped tf_to_base;
    try {
        // Fetch the transform from "odom" frame to "base_link" frame
        tf_listener_.lookupTransform("/base_link", "/odom", ros::Time(0), tf_transform);
        // Now convert tf::StampedTransform to geometry_msgs::TransformStamped
       
        // Fill in the header
        tf_to_base.header.stamp = tf_transform.stamp_;
        tf_to_base.header.frame_id = tf_transform.frame_id_;
        tf_to_base.child_frame_id = tf_transform.child_frame_id_;
        
        // Fill in the translation
        tf_to_base.transform.translation.x = tf_transform.getOrigin().x();
        tf_to_base.transform.translation.y = tf_transform.getOrigin().y();
        tf_to_base.transform.translation.z = tf_transform.getOrigin().z();
        
        // Fill in the rotation
        tf_to_base.transform.rotation.x = tf_transform.getRotation().x();
        tf_to_base.transform.rotation.y = tf_transform.getRotation().y();
        tf_to_base.transform.rotation.z = tf_transform.getRotation().z();
        tf_to_base.transform.rotation.w = tf_transform.getRotation().w();

        // Now you can use 'transform_stamped' as needed (e.g., publish, log, etc.)
        ROS_INFO("Converted Transform: [%.2f, %.2f, %.2f] (Translation), [%.2f, %.2f, %.2f, %.2f] (Rotation)",
                 tf_to_base.transform.translation.x,
                 tf_to_base.transform.translation.y,
                 tf_to_base.transform.translation.z,
                 tf_to_base.transform.rotation.x,
                 tf_to_base.transform.rotation.y,
                 tf_to_base.transform.rotation.z,
                 tf_to_base.transform.rotation.w);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("Could not get transform: %s", ex.what());
        return false;
    }

    global_plan_ = plan;
    ROS_INFO_STREAM("-------------------");
    fsmt_cartesian_point_array_t *array = fsmt_cartesian_point_array_create(100);
    size_t *number_of_points = &array->size;
    float distance=0;
    float distance_to_last_point = 0;
    for (size_t i=1; i<plan.size(); i++ ) {
        float dx = plan[i].pose.position.x-plan[i-1].pose.position.x;
        float dy = plan[i].pose.position.y-plan[i-1].pose.position.y;
        distance += sqrtf(dx*dx+dy*dy);

        geometry_msgs::PoseStamped pose_base_link;
        tf2::doTransform(plan[i], pose_base_link, tf_to_base);  // Using the pre-fetched transform
        if (distance - distance_to_last_point > 0.1){
            printf("distance: %f, pose: %f, %f\n", distance,
                pose_base_link.pose.position.x,
                pose_base_link.pose.position.y);
            array->points[*number_of_points].x = pose_base_link.pose.position.x;
            array->points[*number_of_points].y = pose_base_link.pose.position.y;
            *number_of_points += 1;
            distance_to_last_point = distance;
        }
    }
    printf("\n");

    
    fsmt_cartesian_point_array_destroy(&array);
    return true;
}

bool FSMTBaseLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    if (global_plan_.empty()) return false;

    // Example: Move forward with a constant velocity
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

    ROS2FSMTLaserScan(ros_laser_scan_, fsmt_lidar_);



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

    fsmt_cartesian_point_array_t samples;
    samples.points = (fsmt_cartesian_point_t*) malloc(sizeof(fsmt_cartesian_point_t)*100);
    samples.size = 0;
    samples.max_size = 100;

    // fsmt_sensor_array_t polar;
    // polar.ranges = (fsmt_range_t*) malloc(sizeof(fsmt_range_t)*100);
    // polar.size = 0;
    // polar.max_size = 100;

    float length = -1;
    float radius = -1;

    swept_volume(&samples, radius, length);

    // cartesian_to_polar(&samples, &polar, fsmt_lidar_);
    // int is_available = availability(&polar, fsmt_lidar_);
    // ROS_INFO("Number of points in polar: %ld", polar.size);
    // ROS_INFO("is available: %d", is_available);
    // for (size_t i=0; i< polar.size; i++) {
    //     size_t index = polar.ranges[i].index;
    //     // ROS_INFO("Index %ld: range min = %f, range: %f", index, polar.ranges[i].range1, ros_laser_scan_.ranges[index]);
    // }
    // Add some points
    for (size_t i = 0; i < samples.size; i++) {
        geometry_msgs::Point p;
        p.x = samples.points[i].x;
        p.y = samples.points[i].y;  
        p.z = 0;

        points.points.push_back(p);
    }

    marker_pub_.publish(points);
    // ROS_INFO("Published points to RViz");
    free (samples.points);
    // free (polar.ranges);


    // if(is_available == 1){
    //     cmd_vel.linear.x = 0.0;
    //     cmd_vel.angular.z = 0.0;
    // }else{
    std::cout << "MOVING" << std::endl;
    cmd_vel.linear.x = 0.05;
    cmd_vel.angular.z = 0.0;
    // }

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
    
    // Copying configuration.
    fsmt_lidar_config(fsmt_lidar, 
        ros_laser_scan.angle_min, 
        ros_laser_scan.angle_max, 
        ros_laser_scan.range_min, 
        ros_laser_scan.range_max, 
        ros_laser_scan.angle_increment);

    return 1;
}
