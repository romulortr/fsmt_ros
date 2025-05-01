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

float glength_of_tube[] = { 0.9, 1.5/.75, 1., 1., 1.};
float gspeed[] = {0.75, 0.75, 0.75, 0.75, 0.75};
float gwhiskers[] = {0.1,0.1, 0.1, 0.1, 0.1};
int gresult = 100;
int isouter;
int isfront;
float ggsignal;
float grotation;
float gntubesavailable = 0;
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

    for(size_t i=0; i<2; i++)
        navigation->tubes[i] = fsmt_tube_array_create(number_of_tubes, number_of_samples);
    navigation->recovery.backward = fsmt_tube_array_create(number_of_tubes, 
        number_of_samples);
    navigation->recovery.rotate.sensor = fsmt_sensor_point_array_create(1000);
    navigation->recovery.rotate.cartesian = fsmt_cartesian_point_array_create(1000);

    if(navigation->tubes == NULL)
    {
        fsmt_navigation_destroy(&navigation);
        // Set error code.
        errno = ENOMEM;  
        return NULL;
    }

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
        for(size_t i=0; i<2; i++)
            fsmt_tube_array_destroy(&(*navigation)->tubes[i]);
        fsmt_tube_array_destroy(&(*navigation)->recovery.backward);
        fsmt_cartesian_point_array_destroy(
            &(*navigation)->recovery.rotate.cartesian);
        fsmt_sensor_point_array_destroy(&(*navigation)->recovery.rotate.sensor);
    }
 
    free(*navigation);
}

void fsmt_navigation_reset(fsmt_navigation_t *navigation)
{
    for(size_t i=0; i<2; i++)
        fsmt_tube_array_reset(navigation->tubes[i]);
    fsmt_tube_array_reset(navigation->recovery.backward);
    fsmt_cartesian_point_array_reset(navigation->recovery.rotate.cartesian);
    fsmt_sensor_point_array_reset(navigation->recovery.rotate.sensor);

    navigation->control.velocity.angular_rate = 0;
    navigation->control.velocity.forward = 0;
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
}

void fsmt_navigation_configure(fsmt_navigation_t *navigation, fsmt_params_t *params, fsmt_lidar_t *lidar)
{
    fsmt_navigation_reset(navigation);

    for(size_t i=0; i<2; i++)
    {
        fsmt_tube_array_t *array = navigation->tubes[i];
        params->sampling.whiskers_distance.side = 0.1;
        params->sampling.whiskers_distance.front = 0.05;
        params->vehicle.width = 0.5;
        fsmt_tube_array_configure(array, params, lidar, glength_of_tube[i]);
    }
    fsmt_tube_array_configure(navigation->recovery.backward, params, lidar, -glength_of_tube[0]);
    printf("CONFIGURATION finished\n");
    // for(size_t i=0; i<number_of_tubes; i++)
    // {
    //     navigation->recovery.backward->tube[i]->maneuver.length = -path_length;
    //     navigation->recovery.backward->tube[i]->maneuver.radius = radius[i];
    //     fsmt_polar_tube_configure(
    //         &navigation->recovery.backward->tube[i]->polar, 
    //         &navigation->recovery.backward->tube[i]->maneuver, 
    //         &params);
    //     fsmt_cartesian_tube_configure(
    //         navigation->recovery.backward->tube[i]->cartesian, 
    //         &params, 
    //         &navigation->recovery.backward->tube[i]->maneuver);
    // }
    // navigation->tubes->size = number_of_tubes;
    // navigation->recovery.backward->size = number_of_tubes;

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
int fsmt_tube_array_evaluate_forward(fsmt_tube_array_t *tubes, fsmt_control_t *control, fsmt_lidar_t *lidar,
    fsmt_cartesian_point_t local_goal, float local_goal_orientation)
{

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
    gntubesavailable =0;
    for(size_t i=0; i<number_of_tubes; i++)
    {
        bool new_best = false;
        fsmt_tube_t *ith_tube = tubes->tube[i];
        // is available?
        // printf("1\n");
        int is_available = fsmt_availability(ith_tube->sensor->samples.edge, lidar);
        if(!is_available)
            continue;
        gntubesavailable = gntubesavailable+1;
        // distance to local goal
        // printf("2\n");
        float distance_to_goal_cont = fsmt_distance_to_local_goal(ith_tube->cartesian,
            &local_goal);
    
        int distance_to_goal = distance_to_goal_cont < 0? 0 : 
            fsmt_discrete_distance(distance_to_goal_cont, 0, 0.2) + 1;

        if(distance_to_goal > best.distance_to_goal+1 || distance_to_goal > 6){
            continue;
        }else if (distance_to_goal < best.distance_to_goal){
            new_best = true;
        }
            
        // centralization
        // printf("3\n");
        int is_outer_whisker_available = fsmt_availability(ith_tube->sensor->samples.whiskers.outer, lidar);
        if(best.is_outer_whisker_available > is_outer_whisker_available  && new_best==false){
            continue;
        }else if (best.is_outer_whisker_available < is_outer_whisker_available){
            new_best = true;
        }

        // printf("4\n");
        int is_front_whisker_available = fsmt_availability(ith_tube->sensor->samples.whiskers.front, lidar);
        if(best.is_front_whisker_available > is_front_whisker_available  && new_best==false){
            continue;
        }else if (best.is_front_whisker_available < is_front_whisker_available){
            new_best = true;
        }

        // printf("5\n");
        int is_inner_whisker_available = fsmt_availability(ith_tube->sensor->samples.whiskers.inner, lidar);
        if(best.is_inner_whisker_available > is_inner_whisker_available  && new_best==false){
            continue;
        }else if (best.is_inner_whisker_available < is_inner_whisker_available){
            new_best = true;
        }

        // printf("5\n");
        // // distance to orientation
        float distance_to_orientation = fsmt_orientation_to_local_goal(&ith_tube->polar,
            local_goal_orientation);

        if(distance_to_orientation > M_PI/2)
        {
            // printf("here: %f, local goal: %f\n",  ith_tube->polar.limits.angle.final, local_goal_orientation);
            // printf("length: %f, radius: %f", i, ith_tube->maneuver.length, ith_tube->maneuver.radius);
            continue;        
        }
            
        
        distance_to_orientation =  
            fsmt_discrete_distance(distance_to_orientation, 0, M_PI/9);

        if(distance_to_orientation > best.distance_to_orientation && new_best==false){
            continue;
        }else if (distance_to_orientation < best.distance_to_orientation){
            new_best = true;
        }

        // printf("6\n");
        // // change in actuaction (continuous metric).
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
        gresult = best.distance_to_goal;
        isouter = best.is_outer_whisker_available;
        isfront = best.is_front_whisker_available;
    }
    gntubesavailable = gntubesavailable/number_of_tubes;
    return best.index;
}

int fsmt_tube_array_evaluate_backward(fsmt_tube_array_t *tubes, fsmt_control_t *control, fsmt_lidar_t *lidar,
    fsmt_cartesian_point_t local_goal, float local_goal_orientation)
{
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
            &local_goal);

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
    fsmt_control_t *current_control, bool *is_forward)
{
    *is_forward = true;
    int best_index=-1;
    // Extract local goal andorientation
    float path_length = 0;
    size_t plan_local_goal_index = 0;
    for(int i=0; i<2; i++)
    {
        while (path_length < glength_of_tube[i] && plan_local_goal_index+1 < plan->size){
            float plan_step_dx = plan->points[plan_local_goal_index+1].x - plan->points[plan_local_goal_index].x;
            float plan_step_dy = plan->points[plan_local_goal_index+1].y - plan->points[plan_local_goal_index].y;   
            path_length += sqrtf(plan_step_dx*plan_step_dx + plan_step_dy*plan_step_dy);
            plan_local_goal_index += 1;
        }
        fsmt_cartesian_point_t local_goal = {
            .x = plan->points[plan_local_goal_index].x,
            .y = plan->points[plan_local_goal_index].y};
        navigation->tubes[i]->local_goal = local_goal;
        navigation->tubes[i]->local_orientation = atan2f(
            plan->points[plan_local_goal_index].y - plan->points[plan_local_goal_index-10].y,
            plan->points[plan_local_goal_index].x - plan->points[plan_local_goal_index-10].x 
        );
    }
   float local_orientation0 = atan2f(
        plan->points[10].y - plan->points[0].y,
        plan->points[10].x - plan->points[0].x 
    );

    float des_ang_rate; 
    float amax = 0.75;
    float dwmax = M_PI/3;
    float control_frequency = 20.0;
    float a,dw;
    float radius;
    int bbest_index = -1;
    float des_forward_vel;
    navigation->hindex = 1000;
    float ntubesavailable=0;
    gresult = 1000;
    float vel_max = 1.0;
    float vel_min = 0.25;
    float rate_max = 0.75;
    float rate_min = 0.2;
    
    float m = (vel_max-vel_min)/(rate_max-rate_min);
    float b = vel_max - m*rate_max;
    int best_horizon = -1;
    
    switch (navigation->state)
    {
    case FSMT_STATE_NORMAL:
        for(int i=1; i>=0; i--)
        {
             gntubesavailable = 0;
            best_index = fsmt_tube_array_evaluate_forward(navigation->tubes[i], current_control, lidar,
                navigation->tubes[i]->local_goal, navigation->tubes[i]->local_orientation);
            navigation->hindex = i;
            best_horizon = i;
            printf("IN THE LOOP: %d, %f, %d\n", i, gntubesavailable, best_index);

            if(i==1 && gntubesavailable >= 1.0f)
            {
                break;
            }
        }
        ntubesavailable = gntubesavailable;
        if(best_index >= 0)
        {
            radius = navigation->tubes[navigation->hindex]->tube[best_index]->maneuver.radius;
            printf("best_horizon: %d, ntubesavailable: %f, radius: %f .. \n", 
                navigation->hindex, ntubesavailable, radius);           
            if(best_horizon==1){
                des_forward_vel = 1.75;
            }else if(ntubesavailable < rate_min){
                des_forward_vel = vel_min;
            }else if (ntubesavailable > 1.99){
                des_forward_vel = 1.2;
            }else if (ntubesavailable > rate_max || fabs(radius) > 2){
                des_forward_vel = vel_max;
            }else{
                des_forward_vel = m*ntubesavailable + b;
            }     
            printf("des_Forward_vel: %f\n", des_forward_vel);       
            
            a = (des_forward_vel - navigation->control.velocity.forward)*control_frequency;
            a = a > 2*amax? 2*amax : a;
            a = a < -amax? -amax : a;
            navigation->control.velocity.forward = navigation->control.velocity.forward + a/control_frequency;
            navigation->control.velocity.angular_rate = navigation->control.velocity.forward/radius;
        }else{
             navigation->state = FSMT_STATE_STOP_TO_RECOVER;
        }
        break;
    
    case FSMT_STATE_STOP_TO_RECOVER:
        printf("FSMT_STATE_STOP_TO_RECOVER\n");
            // amax*=3;
            a = (0 - navigation->control.velocity.forward)*control_frequency;
            a = a < -2.5*amax? -2.5*amax : a;
            a = a > 2.5*amax? 2.5*amax : a;
            if(fabs(navigation->control.velocity.angular_rate) < 0.0001)
                radius = 1000;
            else
                radius = navigation->control.velocity.forward/navigation->control.velocity.angular_rate;
            navigation->control.velocity.forward = navigation->control.velocity.forward + a/control_frequency;
            navigation->control.velocity.angular_rate = navigation->control.velocity.forward/radius;
            if(fabs(current_control->velocity.forward) < 0.025 && fabs(current_control->velocity.angular_rate) < 0.025)
            {
                navigation->control.velocity.forward = 0;
                navigation->control.velocity.angular_rate = 0;
                navigation->state = FSMT_STATE_START_RECOVER;
            } 
        break;
    
    case FSMT_STATE_START_RECOVER:
        printf("FSMT_STATE_START_RECOVER\n");
        best_index = fsmt_availability(navigation->recovery.rotate.sensor, lidar);
        if(best_index == 1)
        {
            best_index = -2;
            printf("print: %f\n", local_orientation0);
            ggsignal = local_orientation0/fabs(local_orientation0);
            grotation = local_orientation0;
            navigation->state = FSMT_STATE_RECOVERY_ROTATE;
        }else{
            best_index = fsmt_tube_array_evaluate_backward(navigation->recovery.backward, current_control, lidar,
            navigation->recovery.backward->local_goal, navigation->recovery.backward->local_orientation);
            if(best_index >= 0)
            {
                navigation->state = FSMT_STATE_RECOVERY_MOVE_BACKWARD;
            }else{
                navigation->control.velocity.forward = -0.2;    
            }
            best_index = -1;



        }
        break;

    case FSMT_STATE_RECOVERY_MOVE_BACKWARD:
        printf("FSMT_STATE_MOVE_BACKWARD\n");
        navigation->recovery.backward->local_goal = navigation->tubes[0]->local_goal;
        navigation->recovery.backward->local_orientation = navigation->tubes[0]->local_orientation;
        printf("1\n");

        best_index = fsmt_tube_array_evaluate_backward(navigation->recovery.backward, current_control, lidar,
            navigation->recovery.backward->local_goal, navigation->recovery.backward->local_orientation);
            printf("2\n");

        if(best_index >= 0)
        {
            printf("3\n");

            radius = navigation->recovery.backward->tube[best_index]->maneuver.radius;
            a = (-0.5 - navigation->control.velocity.forward)*control_frequency;
            a = a > amax? amax : a;
            a = a < -amax? -amax : a;
            navigation->control.velocity.forward = navigation->control.velocity.forward + a/control_frequency;
            navigation->control.velocity.angular_rate = navigation->control.velocity.forward/radius;
        }else{
            printf("4\n");

            navigation->state = FSMT_STATE_STOP_TO_NORMAL;
        }
        printf("5\n");

        if(fsmt_availability(navigation->recovery.rotate.sensor, lidar))
        {
            navigation->state = FSMT_STATE_STOP_TO_RECOVER;
        }
        printf("6\n");

        fsmt_tube_array_evaluate_forward(navigation->tubes[0], current_control, lidar,
            navigation->tubes[0]->local_goal, navigation->tubes[0]->local_orientation);
            printf("8\n");

            if (gntubesavailable >= 0.2 && gresult <= 1)
            navigation->state = FSMT_STATE_STOP_TO_NORMAL;
        printf("7\n");

        break;

    case FSMT_STATE_RECOVERY_ROTATE:
        printf("FSMT_STATE_RECOVERY_ROTATE\n");
        best_index = 1;// fsmt_availability(navigation->recovery.rotate.sensor, lidar);
        if(best_index == 1)
        {
            best_index = -2;
            dw = (1*ggsignal - navigation->control.velocity.angular_rate)*control_frequency;
            dw = dw > dwmax? dwmax : dw;
            dw = dw < -dwmax? -dwmax : dw;
            navigation->control.velocity.forward = 0;
            navigation->control.velocity.angular_rate += dw/control_frequency;
            printf("local goal orientation: %f\n", local_orientation0);
            grotation += navigation->control.velocity.angular_rate/control_frequency;
            if(fabs(local_orientation0)<M_PI/36)
            {
                // best_index = fsmt_tube_array_evaluate_forward(navigation->tubes, current_control, lidar,
                //     local_goal, local_goal_orientation);
                if (best_index != -1)
                    navigation->state = FSMT_STATE_STOP_TO_NORMAL;
            }

        }
        else{
            navigation->state = FSMT_STATE_STOP_TO_RECOVER;
        }
        break;
    case FSMT_STATE_STOP_TO_NORMAL:
        printf("FSMT_STATE_STOP_TO_NORMAL\n");
            dw = (0 - navigation->control.velocity.angular_rate)*control_frequency;
            dw = dw > 2*dwmax? 2*dwmax : dw;
            dw = dw < -2*dwmax? -2*dwmax : dw;
            navigation->control.velocity.forward = 0;
            navigation->control.velocity.angular_rate += dw/control_frequency;
            if(fabs(current_control->velocity.forward) < 0.05 || fabs(current_control->velocity.angular_rate) < 0.05)
            {
                navigation->control.velocity.forward = 0;
                navigation->control.velocity.angular_rate = 0;
                navigation->state = FSMT_STATE_NORMAL;
            } 
        break;

    default:
        break;
    }

    printf("DONE!\n");
    // if(best_index == -1)
    // {
    //     *is_forward = false;
    //     int is_available = fsmt_availability(navigation->recovery.rotate.sensor, lidar);
    //     if(is_available)
    //     {
    //         best_index = -2;
    //     }else
    //     {
    //         printf("EVALUATING BACKWARD TUBES..\n");
    //         // navigation->recovery.backward->size = 1;
    //         best_index = fsmt_tube_array_evaluate_backward(navigation->recovery.backward, control, lidar,
    //             local_goal, local_goal_orientation);       
    //         printf("backward best index: %d\n", best_index); 
    //     }
    // }

    return best_index;
}

