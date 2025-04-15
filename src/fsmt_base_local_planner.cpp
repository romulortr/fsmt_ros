#include <fsmt_base_local_planner/fsmt_base_local_planner.hpp>
#include <fsmt/fsmt.h>
#include <pluginlib/class_list_macros.h>

#include <fsmt/score.h>
#include <fsmt/data_structure/transform.h>
#include <fsmt/utils.h>
#include <fsmt/cartesian_tube.h>
#include <fsmt_base_local_planner/utils.hpp>

int count = 0;
PLUGINLIB_EXPORT_CLASS(FSMTBaseLocalPlanner, nav_core::BaseLocalPlanner)

FSMTBaseLocalPlanner::FSMTBaseLocalPlanner() : initialized_(false) {}

void FSMTBaseLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        ROS_INFO("Initializing FSMT Base Local Planner...");
        initialized_ = true;
    }

    ros::NodeHandle nh("~/" + name);

    // Publisher
    marker_fsmt_pub_ = nh.advertise<visualization_msgs::Marker>("marker_fsmt_tube", 10);
    marker_vehicle_at_final_time_pub_ = nh.advertise<visualization_msgs::Marker>
        ("marker_vehicle_at_final_time_pub_", 10);
    marker_local_goal_pub_ = nh.advertise<visualization_msgs::Marker>
    ("marker_local_goal_pub_", 10);

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
        plan_array_->points[i].x = global_plan_[i].pose.position.x;
        plan_array_->points[i].y = global_plan_[i].pose.position.y;

        tf::Quaternion qt;
        tf::quaternionMsgToTF(global_plan_[i].pose.orientation, qt);
        plan_orientation_[i] =wrap_to_pi((float) tf::getYaw(qt) + fsmt_pose.theta);
 
        *number_of_points += 1;
    }

    // Transform from global to robot frame.
    fsmt_point_array_frame_transformation(&fsmt_transform, plan_array_, plan_array_);

    size_t number_of_curvatures = 15;
    float max_path_lenth = .75*1.57;
    // float radius[number_of_curvatures] = {-3, -2, -1.5, -1, -0.75, 1000, 0.75 , 1, 1.5, 2, 3};
    float radius[number_of_curvatures] = {1000, 5, -5, 3, -3, 2, -2, 1.5, -1.5, 1.25, -1.25, 1, -1, 0.75, -0.75};
    // Core Library
    fsmt_cartesian_tube_t *tube[number_of_curvatures];
    fsmt_params_t params = {
        .sampling = {
            .spatial_step = 0.1
        },
        .vehicle = {
            .width = 0.5,
            .lenght = 0.4,
            .distance_axle_to_front = 0.2,
            .distance_axle_to_rear = 0.2,
            .maximum_curvature = 10,

        }
    };
    


    // visualization
    float marker_fsmt_color[3] = {0.0,1.0,0.0};
    float marker_vehicle_at_final_time_color[3] = {0.0,0.0,1.0};
    float marker_local_goal_color[3] = {1.0,0.0,0.5};
    visualization_msgs::Marker marker_fsmt;
    visualization_msgs::Marker marker_vehicle_at_final_time;
    visualization_msgs::Marker marker_local_goal;

    float path_length = 0;
    size_t plan_local_goal_index = 0;
    while (path_length < max_path_lenth && plan_local_goal_index+1 < plan_array_->size){
        float plan_step_dx = global_plan_[plan_local_goal_index+1].pose.position.x - global_plan_[plan_local_goal_index].pose.position.x;
        float plan_step_dy = global_plan_[plan_local_goal_index+1].pose.position.y - global_plan_[plan_local_goal_index].pose.position.y;   
        path_length += sqrtf(plan_step_dx*plan_step_dx + plan_step_dy*plan_step_dy);
        plan_local_goal_index += 1;
    }

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
    fsmt_sensor_point_array_t *sensor = fsmt_sensor_point_array_create(100);

    std::cout << "EVALUATING" << std::endl;
    int des_index = -1;
    float best_metric = 100;
    fsmt_cartesian_point_t local_goal = {
        .x = plan_array_->points[plan_local_goal_index].x,
        .y = plan_array_->points[plan_local_goal_index].y};
    for(size_t i=0; i<number_of_curvatures; i++)
    {

        fsmt_maneuver_t maneuver = {
            .radius = radius[i],
            .length = max_path_lenth
        };        
        tube[i] = fsmt_cartesian_tube_create(100) ;
        fsmt_cartesian_tube_compute(tube[i], &params, &maneuver);

        fsmt_sensor_point_array_reset(sensor);
        cartesian_to_sensor(tube[i]->samples, sensor, fsmt_lidar_);
        int is_available = availability(sensor, fsmt_lidar_);
        
        // score.


        int distance_to_local_goal = !fsmt_distance_to_local_goal(
            tube[i], &local_goal);
        // int distance_to_local_goal = fsmt_distance_to_local_goal(
        //         tube[i], &local_goal);
        float distance_to_local_goal_continuous = 0;
        if (distance_to_local_goal != 0){
            distance_to_local_goal_continuous = fsmt_distance_to_rectangle(
                tube[i], &local_goal);
        }else{
            distance_to_local_goal_continuous = -fsmt_distance_to_rectangle(
                tube[i], &local_goal);           
        }
        std::cout << "Tube #" << i <<  ": available " << is_available << ".. distance (discrete): " << distance_to_local_goal <<
            "distance (continuous): " << distance_to_local_goal_continuous << std::endl;
        if (is_available == 0)
            continue;
    
        float current_metric = distance_to_local_goal_continuous + distance_to_local_goal*10;
        if (current_metric < best_metric ){
            des_index = i;
            best_metric = current_metric;
        }

    }
    // visualization
    if(des_index >= 0){
        fsmt_point_array_to_marker(marker_fsmt, 
            "base_link",
            tube[des_index]->samples,
            marker_fsmt_color);
        fsmt_cartesian_point_t vehicle_edge_at_final_time[4] = {
            tube[des_index]->at_final_time.p1_front_right,
            tube[des_index]->at_final_time.p2_front_left,
            tube[des_index]->at_final_time.p3_rear_left,
            tube[des_index]->at_final_time.p4_rear_right
        };   
        fsmt_points_to_marker(marker_vehicle_at_final_time,
            "base_link",
            vehicle_edge_at_final_time,
            4,
            marker_vehicle_at_final_time_color
        );
        fsmt_points_to_marker(marker_local_goal,
            "base_link",
            &local_goal,
            1,
            marker_local_goal_color
        );
    }
    
    // fsmt_circle_t circle;
    // fsmt_circle_fitting_kasa(plan_array_, &circle);



    // Visualization markers
    // visualization_msgs::Marker marker_vehicle_at_final_time;
    // fsmt_cartesian_point_t vehicle_edge_at_final_time[4] = {
    //     tube->at_final_time.p1_front_right,
    //     tube->at_final_time.p2_front_left,
    //     tube->at_final_time.p3_rear_left,
    //     tube->at_final_time.p4_rear_right
    // };   
    // fsmt_point_array_to_marker(marker_vehicle_at_final_time, 
    //     "base_link",
    //     vehicle_edge_at_final_time,
    //     4,
    //     {1.0,0.0,0.0});


    // Publish
    marker_fsmt_pub_.publish(marker_fsmt);
    marker_vehicle_at_final_time_pub_.publish(marker_vehicle_at_final_time);
    marker_local_goal_pub_.publish(marker_local_goal);

    std::cout << "des_index: " << des_index << std::endl;  
    if(des_index > -1){
        std::cout << "radius[des_index]: " <<  radius[des_index] << std::endl;  
        cmd_vel.linear.x = 1;
        cmd_vel.angular.z = cmd_vel.linear.x/radius[des_index];
    }else{
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
    }

    if (des_index == -1)
        des_index = number_of_curvatures;
    for(size_t i=0; i<des_index; i++)
    {
        fsmt_cartesian_tube_destroy(&tube[i]);
    }

    return true;
}

bool FSMTBaseLocalPlanner::isGoalReached() {
    return global_plan_.empty();
}

void FSMTBaseLocalPlanner::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    ros_laser_scan_ = *msg;
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
    std::cout << "transform frame. To: " << to_frame << ".. From: " << from_frame << std::endl;
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
