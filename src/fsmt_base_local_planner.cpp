#include <fsmt_base_local_planner/fsmt_base_local_planner.hpp>
#include <fsmt/tube.h>
#include <pluginlib/class_list_macros.h>

#include <fsmt/score.h>
#include <fsmt/data_structure/transform.h>
#include <fsmt/utils.h>
#include <fsmt/cartesian_tube.h>
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
    marker_fsmt_edge_pub_ = nh.advertise<visualization_msgs::Marker>("marker_fsmt_edge_tube", 10);
    marker_fsmt_whisker_pub_ = nh.advertise<visualization_msgs::Marker>("marker_fsmt_whisker_tube", 10);
    marker_vehicle_at_final_time_pub_ = nh.advertise<visualization_msgs::Marker>
        ("marker_vehicle_at_final_time_pub_", 10);
    marker_local_goal_pub_ = nh.advertise<visualization_msgs::Marker>
    ("marker_local_goal_pub_", 10);

    // Subscribe to LiDAR scan topic
    laser_scan_sub_ = nh.subscribe("/front/scan", 1, &FSMTBaseLocalPlanner::lidarCallback, this);
    odom_sub_ = nh.subscribe("/odometry/filtered", 1, &FSMTBaseLocalPlanner::odomCallback, this);
    fsmt_lidar_ = NULL;

    // FSMT memory allocation.
    plan_array_ = fsmt_cartesian_point_array_create(1000);

    // motion tubes
    float max_path_length = 1.;
    float angle_step_in_deg = 0.25;
    float final_angle_in_deg = 90;
    size_t number_of_tubes = 2*(final_angle_in_deg/angle_step_in_deg+1);

    // motion tubes
    float radius[number_of_tubes];
    for(size_t i=0; i<number_of_tubes/2; i++){
        float angle = i*angle_step_in_deg*(3.1415/180);
        if(fabs(angle) < 0.001)
            angle = 0.001;
        radius[2*i] = max_path_length/angle;
        radius[2*i+1] = -max_path_length/angle;
    }

    // Core Library
    navigation_ = fsmt_navigation_create(number_of_tubes, 300);
    if(navigation_ == NULL){
        // DO something..
    }

    navigation_->length = max_path_length;
    navigation_->control.velocity.angular_rate = 0;
    navigation_->control.velocity.forward = 0;

    navigation_->recovery.backward->size = number_of_tubes;
    current_velocity_.velocity.forward = 0;
    current_velocity_.velocity.angular_rate = 0;
    tube_configured_ = false;
    recovery_mode_ = false;
    printf("FINISHED CREATION!\n");

}

bool FSMTBaseLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
    if (plan.empty() || plan.empty() > 1000) {
        ROS_WARN("Received an empty plan!");
        return true;
    }
    global_plan_ = plan;
    return true;
}

bool FSMTBaseLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    // wait for a global plan.
    if (global_plan_.empty()){
        std::cout << "empty plan" << std::endl;
        return false;
    } 

    // wait for a valid lidar reading.
    if(fsmt_lidar_ == NULL){
        std::cout << "fsmt_lidar is null" << std::endl;
        return false;
    } 

    // The global plan must be transformed from its current frame (e.g., map or odom frame)
    // to the robot frame (e.g., base_link).
    // First, get the transform from global plan frame to robot frame.
    tf::StampedTransform tf_transform;
    bool has_transform = GetTransform("/base_link", global_plan_.front().header.frame_id, 
        global_plan_.front().header.stamp, tf_transform);
    if (!has_transform) return false;
    // Next we convert the message from TF standard to geometry message.
    fsmt_transform_t fsmt_transform;
    tf_transform_to_fsmt_transform(tf_transform, fsmt_transform);
    fsmt_pose_t fsmt_pose;
    tf_transform_to_fsmt_pose(tf_transform, &fsmt_pose);

    // Transform to fsmt data structure
    size_t *number_of_points = &plan_array_->size;
    *number_of_points = 0;
    for (size_t i=0; i<global_plan_.size(); i++ )
    {
        plan_array_->points[*number_of_points].x = global_plan_[i].pose.position.x;
        plan_array_->points[*number_of_points].y = global_plan_[i].pose.position.y;
        *number_of_points += 1;
    }

    // Transform from global to robot frame.
    fsmt_point_array_frame_transformation(&fsmt_transform, plan_array_, plan_array_);
    
    // Evaluate motion tubes.
    int des_index = fsmt_evaluate(navigation_, fsmt_lidar_, plan_array_, NULL, &current_velocity_);
    if(des_index >= 0)
    {
        printf("HER!\n");
        fsmt_maneuver_t *maneuver = &navigation_->solution.tube->maneuver;
        cmd_vel.linear.x = 0.5;
        cmd_vel.angular.z =  cmd_vel.linear.x / maneuver->radius;
    }else
    {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
    }
    // visualization
    float marker_fsmt_edge_color[3] = {0.0,1.0,0.0};
    float marker_fsmt_whisker_color[3] = {0.0,0.5,1};
    float marker_vehicle_at_final_time_color[3] = {0.0,0.0,1.0};
    float marker_local_goal_color[3] = {1.0,0.0,0.5};
    visualization_msgs::Marker marker_fsmt_edge;
    visualization_msgs::Marker marker_fsmt_whisker;
    visualization_msgs::Marker marker_vehicle_at_final_time;
    visualization_msgs::Marker marker_local_goal;

    // if(des_index >= 0 && (navigation_->state==FSMT_STATE_RECOVERY_MOVE_BACKWARD ||
    //     navigation_->state == FSMT_STATE_NORMAL)){


    //     fsmt_cartesian_tube_t *cartesian_tube = !(navigation_->state==FSMT_STATE_RECOVERY_MOVE_BACKWARD)? 
    //         navigation_->tubes[navigation_->solution.index.horizon]->tube[des_index]->cartesian:
    //         navigation_->recovery.backward->tube[des_index]->cartesian ;
     

    //     fsmt_point_array_to_marker(marker_fsmt_edge, 
    //         "base_link",
    //         cartesian_tube->samples.edge,
    //         marker_fsmt_edge_color);

    //     fsmt_point_array_to_marker(marker_fsmt_whisker, 
    //         "base_link",
    //         cartesian_tube->samples.whiskers.inner,
    //         marker_fsmt_whisker_color);
    //     fsmt_point_array_to_marker(marker_fsmt_whisker, 
    //         "base_link",
    //         cartesian_tube->samples.whiskers.outer,
    //         marker_fsmt_whisker_color);
    //     fsmt_point_array_to_marker(marker_fsmt_whisker, 
    //         "base_link",
    //         cartesian_tube->samples.whiskers.front,
    //         marker_fsmt_whisker_color);
    //     fsmt_cartesian_point_t vehicle_edge_at_final_time[4] = {
    //         cartesian_tube->at_final_time.p1_front_right,
    //         cartesian_tube->at_final_time.p2_front_left,
    //         cartesian_tube->at_final_time.p3_rear_left,
    //         cartesian_tube->at_final_time.p4_rear_right
    //     };   
    //     fsmt_points_to_marker(marker_vehicle_at_final_time,
    //         "base_link",
    //         vehicle_edge_at_final_time,
    //         4,
    //         marker_vehicle_at_final_time_color
    //     );
    //     if(!(navigation_->state==FSMT_STATE_RECOVERY_MOVE_BACKWARD))
    //     {
    //         fsmt_points_to_marker(marker_local_goal,
    //             "base_link",
    //             &navigation_->tubes[navigation_->solution.index.horizon]->local_goal,
    //             1,
    //             marker_local_goal_color
    //         );
    //     }else{
    //         fsmt_points_to_marker(marker_local_goal,
    //             "base_link",
    //             &navigation_->recovery.backward->local_goal,
    //             1,
    //             marker_local_goal_color
    //         );
    //     }

    // }else if (des_index==-2){
    //     fsmt_point_array_to_marker(marker_fsmt_edge, 
    //         "base_link",
    //         navigation_->recovery.rotate.cartesian,
    //         marker_fsmt_edge_color);
    // }


    // // Publish
    // marker_fsmt_edge_pub_.publish(marker_fsmt_edge);
    // marker_fsmt_whisker_pub_.publish(marker_fsmt_whisker);
    // marker_vehicle_at_final_time_pub_.publish(marker_vehicle_at_final_time);
    // marker_local_goal_pub_.publish(marker_local_goal);

    // cmd_vel.linear.x = navigation_->control.velocity.forward;
    // cmd_vel.angular.z = navigation_->control.velocity.angular_rate;
    return true;
}

bool FSMTBaseLocalPlanner::isGoalReached() {
    return global_plan_.empty();
}

void FSMTBaseLocalPlanner::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    ros_laser_scan_ = *msg;

    size_t number_of_beams = ros_laser_scan_.ranges.size();
    if(fsmt_lidar_ == NULL)
    {
        fsmt_lidar_ = fsmt_lidar_create(number_of_beams);
    }

    if(fsmt_lidar_->config.size.max < number_of_beams)
    {
        fsmt_lidar_destroy(&fsmt_lidar_);
        fsmt_lidar_ = fsmt_lidar_create(number_of_beams);
        if(fsmt_lidar_ == NULL)
        {
            return;
        }
    }
    ROS2FSMTLaserScan(ros_laser_scan_, fsmt_lidar_);

    // Configure motion tube (only once).
    // @TODO: this has to move to a configuration file
    fsmt_params_t params;
    params.sampling.sampling_step.edge = 0.1;
    params.sampling.sampling_step.whiskers = 0.1;
    params.sampling.whiskers_distance.front = 0.05;
    params.sampling.whiskers_distance.side = 0.1;
    params.vehicle.width = 0.5;
    params.vehicle.length = 0.4;
    params.vehicle.distance_axle_to_front = 0.2;
    params.vehicle.distance_axle_to_rear = 0.2;
    params.vehicle.maximum_curvature = 5;


    if(tube_configured_==false)
    {
        fsmt_navigation_configure(navigation_, &params, fsmt_lidar_);
        tube_configured_ = true;
    }
}

void FSMTBaseLocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    current_velocity_.velocity.forward = msg->twist.twist.linear.x;
    current_velocity_.velocity.angular_rate = msg->twist.twist.angular.z;
}

void fsmt_points_to_marker(visualization_msgs::Marker &marker, std::string frame_id, 
    fsmt_cartesian_point_t *points, size_t size, float color[3])
{
    marker.header.frame_id = frame_id;  // Change frame ID if needed
    marker.header.stamp = ros::Time::now();
    marker.ns = "points";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;

    // Set scale for point size
    marker.scale.x = 0.075;  // Width of points
    marker.scale.y = 0.075;

    // Set color (RGBA)
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1.0;  // Fully visible
 
    for (size_t i = 0; i < size; i++) {
        geometry_msgs::Point p;
        p.x = points[i].x;
        p.y = points[i].y;  
        p.z = 0;

        marker.points.push_back(p);
    }
}

void FSMTBaseLocalPlanner::fsmt_point_array_to_marker(visualization_msgs::Marker &marker, std::string frame_id, 
    fsmt_cartesian_point_array_t *fsmt_array, float color[3])
{
    marker.header.frame_id = frame_id;  // Change frame ID if needed
    marker.header.stamp = ros::Time::now();
    marker.ns = "points";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;

    // Set scale for point size
    marker.scale.x = 0.05;  // Width of points
    marker.scale.y = 0.05;

    // Set color (RGBA)
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1.0;  // Fully visible
 
    for (size_t i = 0; i < fsmt_array->size; i++) {
        geometry_msgs::Point p;
        p.x = fsmt_array->points[i].x;
        p.y = fsmt_array->points[i].y;  
        p.z = 0;
        marker.points.push_back(p);
    }
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
    std::string from_frame, ros::Time timestamp, tf::StampedTransform &tf_transform)
{
    try{
        // Fetch the transform from "odom" frame to "base_link" frame
        tf_listener_.lookupTransform(to_frame, from_frame, 
            timestamp, tf_transform);

    } catch (tf::TransformException &ex) {
        ROS_ERROR("Could not get transform: %s", ex.what());
        return false;
    }
    return true;
}
