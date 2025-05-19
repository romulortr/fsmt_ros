/**
 * @file navigation_with_global_wp_plan.c
 * @brief .
 *
 * Detailed description here.
 */

#include <fsmt/navigation/navigation_with_global_wp_plan.h>

#include <fsmt/score.h>
#include <fsmt/utils.h>

#include <stdint.h>
#include <math.h>
#include <stdio.h>

#include <stdlib.h>
#include <errno.h>

fsmt_navigation_t* fsmt_navigation_create(size_t number_of_tubes, size_t number_of_samples)
{
    fsmt_navigation_t *navigation = (fsmt_navigation_t*) malloc(
        sizeof(fsmt_navigation_t));
    if(navigation==NULL)
    {
        // Set error code.
        errno = ENOMEM;  
        return NULL;
    }

    navigation->nominal_horizon = fsmt_tube_array_create(number_of_tubes, number_of_samples);
    navigation->long_horizon = fsmt_tube_array_create(number_of_tubes, number_of_samples);

    navigation->recovery.backward = fsmt_tube_array_create(number_of_tubes, 
        number_of_samples);
    navigation->recovery.rotate.sensor = fsmt_sensor_point_array_create(1000);
    navigation->recovery.rotate.cartesian = fsmt_cartesian_point_array_create(1000);

    // if(navigation->tubes == NULL)
    // {
    //     fsmt_navigation_destroy(&navigation);
    //     // Set error code.
    //     errno = ENOMEM;  
    //     return NULL;
    // }

    fsmt_navigation_reset(navigation);

    return navigation;
}

void fsmt_navigation_destroy(fsmt_navigation_t **navigation)
{
    if(navigation==NULL)
    {
        return;
    }

    if(*navigation != NULL)
    {
        fsmt_tube_array_destroy(&(*navigation)->nominal_horizon);
        fsmt_tube_array_destroy(&(*navigation)->long_horizon);
        fsmt_tube_array_destroy(&(*navigation)->recovery.backward);
        fsmt_cartesian_point_array_destroy(
            &(*navigation)->recovery.rotate.cartesian);
        fsmt_sensor_point_array_destroy(&(*navigation)->recovery.rotate.sensor);
    }
 
    free(*navigation);
}

void fsmt_navigation_reset(fsmt_navigation_t *navigation)
{
    fsmt_tube_array_reset(navigation->nominal_horizon);
    fsmt_tube_array_reset(navigation->long_horizon);
    fsmt_tube_array_reset(navigation->recovery.backward);
    fsmt_cartesian_point_array_reset(navigation->recovery.rotate.cartesian);
    fsmt_sensor_point_array_reset(navigation->recovery.rotate.sensor);

    navigation->state = FSMT_STATE_NORMAL;
}

void fsmt_tube_array_configure(fsmt_tube_array_t *array, fsmt_params_t *params, fsmt_lidar_t *lidar, float length_of_tube)
{
    float angle_step_in_deg = 0.5;
    float final_angle_in_deg = 90;
    size_t number_of_tubes = 2*(final_angle_in_deg/angle_step_in_deg+1);

    fsmt_tube_array_reset(array);
    for(size_t i=0; i<number_of_tubes/2; i++)
    {
        // Final angle of the vehicle when performing this maneuver.
        float angle = i*angle_step_in_deg*(M_PI/180.0f); 
        // Curvature computed below is always non-negative (because angle
        // and path length are always non-negative).
        float curvature = angle/fabs(length_of_tube);    
        // Threshold to avoid division by zero when computing the radius.
        if(curvature < 0.001f) curvature = 0.001f;
        
        for(size_t j=0; j<2; j++)
        {
            if (j==1) curvature = -curvature;
            array->tube[2*i+j]->maneuver.length = length_of_tube;
            array->tube[2*i+j]->maneuver.radius = 0.9f/curvature;    
            fsmt_polar_tube_configure(
                &array->tube[2*i+j]->polar, 
                &array->tube[2*i+j]->maneuver, 
                params);
            fsmt_cartesian_tube_configure(
                array->tube[2*i+j]->cartesian, 
                &array->tube[2*i+j]->maneuver,
                params
            );
            fsmt_sensor_tube_from_cartesian_tube(
                array->tube[2*i+j]->sensor,
                array->tube[2*i+j]->cartesian,
                lidar
            );
        }
    }
    array->size = number_of_tubes;
    array->length = length_of_tube;
}

void fsmt_navigation_configure(fsmt_navigation_t *navigation, fsmt_params_t *params, fsmt_lidar_t *lidar)
{
    fsmt_navigation_reset(navigation);

    fsmt_tube_array_configure(navigation->nominal_horizon, params, lidar, 0.75);
    fsmt_tube_array_configure(navigation->long_horizon, params, lidar, 2*0.75);

    fsmt_tube_array_configure(navigation->recovery.backward, params, lidar, -0.75);

    fsmt_cartesian_point_t p_sensor_origin = {
        .x = 0.055,
        .y = 0
    };
    float theta=0;
    float c = cosf(theta);
    float s = sinf(theta);
    float radius = 0.4;
    for(size_t i=0; i<lidar->measurements.size; i++)
    {
        float beam_angle = lidar->config.angle.min + lidar->config.angular_step*i;

        fsmt_cartesian_point_t p_circle;

        fsmt_circle_ray_intersection(&p_sensor_origin, beam_angle, radius,
            &p_circle); 
        float xi = p_circle.x;
        float yi = p_circle.y;
        float xl = c*xi - s*yi + p_sensor_origin.x;
        float yl = s*xi + c*yi + p_sensor_origin.y;
        float distance = sqrtf(yl*yl + xl*xl);

        navigation->recovery.rotate.cartesian->points[i].x = xi;
        navigation->recovery.rotate.cartesian->points[i].y = yi;

        navigation->recovery.rotate.sensor->points[i].range1 = distance;
        navigation->recovery.rotate.sensor->points[i].index = i;
    }
    navigation->recovery.rotate.cartesian->size = lidar->measurements.size;
    navigation->recovery.rotate.sensor->size = lidar->measurements.size;

}

/**
 * Discrete: Available or not
 * Discrete: Contains goal, continuous: position error
 * Discrete: Contains orientation, continuous: orientation error
 * Discrete: centralized, countable: steps to obstacle
 * Continuous: smallest change in actuation
*/
int fsmt_tube_array_evaluate_forward(fsmt_navigation_t *navigation, fsmt_control_t *control, fsmt_lidar_t *lidar)
{
    fsmt_tube_array_t *tubes = navigation->nominal_horizon;
    fsmt_cartesian_point_t local_goal_position = tubes->local_goal;
    float local_goal_orientation = tubes->local_orientation;
    navigation->solution.number_of_feasible_tubes = 0;

    struct{
        int index;
        int distance_to_goal;
        float distance_to_goal_cont;
        float distance_to_orientation;
        int is_outer_whisker_available;
        int is_inner_whisker_available;
        int is_front_whisker_available;
        int outer_whisker_counter;
        float change_in_actuation;
    }best;

    best.index=-1;
    best.distance_to_goal= 10000;
    best.distance_to_goal_cont= 10000;
    best.distance_to_orientation=3.1415;
    best.is_outer_whisker_available = -1;
    best.is_inner_whisker_available = -1;
    best.is_front_whisker_available = -1;
    best.change_in_actuation = 1000;

    float current_forward_vel = control->velocity.forward;
    float current_angular_rate = control->velocity.angular_rate;
    size_t number_of_tubes = tubes->size;

    for(size_t i=0; i<number_of_tubes; i++)
    {
        bool new_best = false;
        fsmt_tube_t *ith_tube = tubes->tube[i];
        // is available?
        int is_available = fsmt_availability(ith_tube->sensor->samples.edge, lidar);
        if(!is_available)
            continue;
        navigation->solution.number_of_feasible_tubes += 1;
        // distance to local goal
        float distance_to_goal_cont = fsmt_distance_to_local_goal(ith_tube->cartesian,
            &local_goal_position);
    
        int distance_to_goal = distance_to_goal_cont < 0? 0 : 
            fsmt_discrete_distance(distance_to_goal_cont, 0, 0.2) + 1;

        if(distance_to_goal > best.distance_to_goal+1 || distance_to_goal > 6){
            continue;
        }else if (distance_to_goal < best.distance_to_goal){
            new_best = true;
        }
            
        // centralization
        int is_outer_whisker_available = fsmt_availability(ith_tube->sensor->samples.whiskers.outer, lidar);
        if(best.is_outer_whisker_available > is_outer_whisker_available  && new_best==false){
            continue;
        }else if (best.is_outer_whisker_available < is_outer_whisker_available){
            new_best = true;
        }

        int is_front_whisker_available = fsmt_availability(ith_tube->sensor->samples.whiskers.front, lidar);
        if(best.is_front_whisker_available > is_front_whisker_available  && new_best==false){
            continue;
        }else if (best.is_front_whisker_available < is_front_whisker_available){
            new_best = true;
        }

        int is_inner_whisker_available = fsmt_availability(ith_tube->sensor->samples.whiskers.inner, lidar);
        if(best.is_inner_whisker_available > is_inner_whisker_available  && new_best==false){
            continue;
        }else if (best.is_inner_whisker_available < is_inner_whisker_available){
            new_best = true;
        }

        // distance to orientation
        float distance_to_orientation = fsmt_orientation_to_local_goal(&ith_tube->polar,
            local_goal_orientation);

        if(distance_to_orientation > M_PI/2)
        {
            continue;        
        }
        
        distance_to_orientation =  
            fsmt_discrete_distance(distance_to_orientation, 0, M_PI/9);

        if(distance_to_orientation > best.distance_to_orientation && new_best==false){
            continue;
        }else if (distance_to_orientation < best.distance_to_orientation){
            new_best = true;
        }

        // change in actuaction (continuous metric).
        float change_in_actuation = 0;
        if (control->velocity.forward > 0)
        {
            float angular_rate = current_forward_vel/ith_tube->maneuver.radius;
            change_in_actuation = fabs(current_angular_rate - angular_rate);
        }

        if(change_in_actuation > best.change_in_actuation && new_best==false){
            continue;
        }else if (change_in_actuation < best.change_in_actuation){
            new_best = true;
        }

        // Ultimately, we select the path that is the closest to the goal.
        if(distance_to_goal_cont > best.distance_to_goal_cont && new_best==false){
            continue;
        }

        best.index = i;
        best.distance_to_goal = distance_to_goal;
        best.distance_to_goal_cont = distance_to_goal_cont;
        best.is_outer_whisker_available = is_outer_whisker_available;
        best.is_front_whisker_available = is_front_whisker_available;
        best.is_inner_whisker_available = is_inner_whisker_available;
        best.distance_to_orientation = distance_to_orientation;
        best.change_in_actuation = change_in_actuation;
    }
    navigation->solution.rate_of_feasible_tubes = 
        ((float) navigation->solution.number_of_feasible_tubes)/number_of_tubes;
    navigation->solution.distance_to_goal = best.distance_to_goal;
    return best.index;
}

int fsmt_tube_array_evaluate_backward(fsmt_navigation_t *navigation, fsmt_control_t *control, fsmt_lidar_t *lidar)
{
    fsmt_tube_array_t *tubes = navigation->recovery.backward;
    fsmt_cartesian_point_t local_goal_position = navigation->nominal_horizon->local_goal;
    float local_goal_orientation = navigation->nominal_horizon->local_orientation;
    navigation->solution.number_of_feasible_tubes = 0;

    size_t number_of_tubes = tubes->size;
    bool free_array[number_of_tubes];
    int free_array_positive[number_of_tubes];
    int gdistance_to_goal[number_of_tubes];
    int best_distance_to_goal = 1000;

    for(size_t i=0; i<number_of_tubes; i++)
    {
        gdistance_to_goal[i] = 1000;
        free_array_positive[i] = 0;
        free_array[i] = 0;
        fsmt_tube_t *ith_tube = tubes->tube[i];
        
        int is_available = fsmt_availability(ith_tube->sensor->samples.edge, lidar);
        
        if(is_available == 0) continue;
 
        float distance_to_goal_cont = fsmt_distance_to_local_goal(ith_tube->cartesian,
            &local_goal_position);

        int distance_to_goal = distance_to_goal_cont < 0? 0 : 
        fsmt_discrete_distance(distance_to_goal_cont, 0, 0.4) + 1;
        gdistance_to_goal[i] = distance_to_goal;
        
        if(distance_to_goal > best_distance_to_goal) continue;
        
        best_distance_to_goal = distance_to_goal;

        free_array[i] = 1;
        free_array_positive[i] = free_array[i];
        if(i>0){
            free_array_positive[i] += free_array_positive[i-1];
        }
    }

    int free_array_negative[number_of_tubes];
    int best_index = -1;
    int best_result = 0;
    for(size_t i=0; i<number_of_tubes; i++)
    {
        int j = number_of_tubes - i - 1;
        free_array_negative[j] = free_array[j];
        if(j<number_of_tubes-1 && free_array[j]==1 && gdistance_to_goal[j]<=best_distance_to_goal+2){
            free_array_negative[j] += free_array_negative[j+1];
        }
        int result = free_array_negative[j] > free_array_positive[j]? free_array_positive[j]: free_array_negative[j];
        if (result > best_result)
        {
            best_result = result;
            best_index = j;
        }
    }    
    return best_index;
}

int fsmt_evaluate(fsmt_navigation_t *navigation, fsmt_lidar_t *lidar, fsmt_cartesian_point_array_t *plan, float *orientation,
    fsmt_control_t *current_control)
{
    // pointers to simplify notation;
    fsmt_tube_array_t *nominal_horizon = navigation->nominal_horizon;

    // Extract local goal and orientation
    float des_path_length = nominal_horizon->length;
    float path_length = 0;
    size_t plan_local_goal_index = 0;

    while (path_length < des_path_length && plan_local_goal_index+1 < plan->size){
        float plan_step_dx = plan->points[plan_local_goal_index+1].x - plan->points[plan_local_goal_index].x;
        float plan_step_dy = plan->points[plan_local_goal_index+1].y - plan->points[plan_local_goal_index].y;   
        path_length += sqrtf(plan_step_dx*plan_step_dx + plan_step_dy*plan_step_dy);
        plan_local_goal_index += 1;
    }
    fsmt_cartesian_point_t local_goal = {
        .x = plan->points[plan_local_goal_index].x,
        .y = plan->points[plan_local_goal_index].y};
    navigation->nominal_horizon->local_goal = local_goal;
    navigation->nominal_horizon->local_orientation = atan2f(
        plan->points[plan_local_goal_index].y - plan->points[plan_local_goal_index-10].y,
        plan->points[plan_local_goal_index].x - plan->points[plan_local_goal_index-10].x 
    );

    // Short term orientation.
    float head_orientation = atan2f(
        plan->points[10].y - plan->points[0].y,
        plan->points[10].x - plan->points[0].x 
    );

    float radius;
    int best_horizon = -1;
    int best_index=-1;
    // CONFIGURE
    switch (navigation->state)
    {
    case FSMT_STATE_NORMAL:
        best_index = fsmt_tube_array_evaluate_forward(navigation, current_control, lidar);
        if(best_index >= 0)
        {
            navigation->solution.tube = navigation->nominal_horizon->tube[best_index];
            radius = navigation->nominal_horizon->tube[best_index]->maneuver.radius;
        }else
        {
            navigation->state = FSMT_STATE_STOP_TO_RECOVER;
        }
        break;
    
    case FSMT_STATE_STOP_TO_RECOVER:
        navigation->solution.tube = NULL;
        if(fabs(current_control->velocity.forward) < 0.025 && fabs(current_control->velocity.angular_rate) < 0.025)
        {
            navigation->state = FSMT_STATE_START_RECOVER;
        } 
        break;
    
    case FSMT_STATE_START_RECOVER:
        best_index = fsmt_availability(navigation->recovery.rotate.sensor, lidar);
        if(fsmt_availability(navigation->recovery.rotate.sensor, lidar) == 1)
        {
            navigation->recovery.rotate.orientation_error = head_orientation;
            navigation->state = FSMT_STATE_RECOVERY_ROTATE;
        }
        else{
            best_index = fsmt_tube_array_evaluate_backward(navigation, current_control, lidar);
            if(best_index >= 0)
            {
                navigation->state = FSMT_STATE_RECOVERY_MOVE_BACKWARD;
            }else{
                navigation->state = FSMT_STATE_RECOVERY_MOVE_BACKWARD_BLIND;
            }
        }
        break;

    case FSMT_STATE_RECOVERY_MOVE_BACKWARD:
        best_index = fsmt_tube_array_evaluate_backward(navigation, current_control, lidar);
        if(best_index >= 0)
        {
            navigation->solution.tube = navigation->recovery.backward->tube[best_index];
            radius = navigation->recovery.backward->tube[best_index]->maneuver.radius;

        }else{
            navigation->solution.tube = NULL;
            navigation->state = FSMT_STATE_STOP_TO_NORMAL;
        }

        if(fsmt_availability(navigation->recovery.rotate.sensor, lidar))
        {
            navigation->solution.tube = NULL;
            navigation->state = FSMT_STATE_STOP_TO_RECOVER;
        }

        fsmt_tube_array_evaluate_forward(navigation, current_control, lidar);
        if (navigation->solution.rate_of_feasible_tubes >= 0.2 && 
            navigation->solution.distance_to_goal <= 1) // gresult has the distance to goal
        {
            navigation->solution.tube = NULL;
            navigation->state = FSMT_STATE_STOP_TO_NORMAL;
        }
        break;

    case FSMT_STATE_RECOVERY_ROTATE:
        navigation->solution.tube = NULL;
        radius = navigation->recovery.rotate.orientation_error/fabs(navigation->recovery.rotate.orientation_error);
        fsmt_tube_array_evaluate_forward(navigation, current_control, lidar);
        if (navigation->solution.rate_of_feasible_tubes >= 0.2 && 
            navigation->solution.distance_to_goal <= 1)
        {
            navigation->solution.tube = NULL;
            navigation->state = FSMT_STATE_STOP_TO_NORMAL;
        }
        break;

    case FSMT_STATE_STOP_TO_NORMAL:
        if(fabs(current_control->velocity.forward) < 0.05 || fabs(current_control->velocity.angular_rate) < 0.05)
        {
            navigation->state = FSMT_STATE_NORMAL;
        } 
        break;

    case FSMT_STATE_RECOVERY_MOVE_BACKWARD_BLIND:
        navigation->solution.tube = NULL;
        radius = 1000000;
        fsmt_tube_array_evaluate_forward(navigation, current_control, lidar);
        if (navigation->solution.rate_of_feasible_tubes >= 0.2 && 
            navigation->solution.distance_to_goal <= 1)
        {
            navigation->solution.tube = NULL;
            navigation->state = FSMT_STATE_STOP_TO_NORMAL;
        }
    break;

    default:
        break;
    }

    // ACTION
    switch (navigation->state)
    {
    case FSMT_STATE_NORMAL:
        navigation->solution.control.velocity.forward = 1;
        navigation->solution.control.velocity.angular_rate = 1/radius;
        break;
    
    case FSMT_STATE_STOP_TO_RECOVER:
    case FSMT_STATE_START_RECOVER:
    case FSMT_STATE_STOP_TO_NORMAL:
        navigation->solution.control.velocity.forward = 0;
        navigation->solution.control.velocity.angular_rate = 0; 
        break;
    
    case FSMT_STATE_RECOVERY_ROTATE:
        navigation->solution.control.velocity.forward = 0;
        navigation->solution.control.velocity.angular_rate = (M_PI/4)*radius;
        break;

    case FSMT_STATE_RECOVERY_MOVE_BACKWARD:
    case FSMT_STATE_RECOVERY_MOVE_BACKWARD_BLIND:
        navigation->solution.control.velocity.forward = -0.5;
        navigation->solution.control.velocity.angular_rate = -0.5/radius;
        break;

    default:
        break;
    }

    return best_index;
}

