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
    fsmt_navigation_t *navigation = (fsmt_navigation_t*) malloc(sizeof(fsmt_navigation_t));
    if(navigation==NULL)
    {
        // Set error code.
        errno = ENOMEM;  
        return NULL;
    }

    navigation->tubes = fsmt_tube_array_create(number_of_tubes, number_of_samples);

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
        fsmt_tube_array_destroy(&(*navigation)->tubes);
    }
 
    free(*navigation);
}

void fsmt_navigation_reset(fsmt_navigation_t *navigation)
{
    fsmt_tube_array_reset(navigation->tubes);
}

/**
 * Discrete: Available or not
 * Discrete: Contains goal, continuous: position error
 * Discrete: Contains orientation, continuous: orientation error
 * Discrete: centralized, countable: steps to obstacle
 * Continuous: smallest change in actuation
*/
int fsmt_evaluate(fsmt_navigation_t *navigation, fsmt_lidar_t *lidar, fsmt_cartesian_point_array_t *plan, float *orientation,
    fsmt_control_t *control)
{
    printf("1: %p %p\n", navigation, navigation->tubes);
    struct{
        int index;
        float distance_to_goal;
        float distance_to_orientation;
        int centralization;
        int outer_whisker_counter;
        float change_in_actuation;
    }best;

    best.index=-1;
    best.distance_to_goal= 10000;
    best.distance_to_orientation=3.1415;
    best.centralization = -1;
    best.outer_whisker_counter = -1;
    best.change_in_actuation = 1000;

    // Extract local goal andorientation
    float path_length = 0;
    size_t plan_local_goal_index = 0;
    while (path_length < navigation->length && plan_local_goal_index+1 < plan->size){
        float plan_step_dx = plan->points[plan_local_goal_index+1].x - plan->points[plan_local_goal_index].x;
        float plan_step_dy = plan->points[plan_local_goal_index+1].y - plan->points[plan_local_goal_index].y;   
        path_length += sqrtf(plan_step_dx*plan_step_dx + plan_step_dy*plan_step_dy);
        plan_local_goal_index += 1;
    }
    fsmt_cartesian_point_t local_goal = {
        .x = plan->points[plan_local_goal_index].x,
        .y = plan->points[plan_local_goal_index].y};

    navigation->local_goal = local_goal;
    float local_goal_orientation = atan2f(
        plan->points[plan_local_goal_index].y - plan->points[plan_local_goal_index-5].y,
        plan->points[plan_local_goal_index].x - plan->points[plan_local_goal_index-5].x 
    );
    fsmt_tube_array_t *tubes = navigation->tubes;
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

        // distance to local goal
        float distance_to_goal = fsmt_distance_to_local_goal(ith_tube->cartesian,
            &local_goal);
        distance_to_goal = distance_to_goal < 0? 0 : 
            fsmt_discrete_distance(distance_to_goal, 0, 0.2) + 1;

        if(distance_to_goal > best.distance_to_goal){
            continue;
        }else if (distance_to_goal < best.distance_to_goal){
            new_best = true;
        }
        printf("tube (position): %ld\n", i);
        // if(distance_to_goal > 0 && distance_to_goal > best.distance_to_goal){
        //     continue;
        // }

        // centralization
        // int centralization = fsmt_whisker_distance(ith_tube->sensor, lidar);
        int centralization = fsmt_availability_counter(ith_tube->sensor->samples.whiskers.outer, lidar);
        centralization = centralization > 4? 4: centralization;
        // printf("best.centralization: %d, centralization: %d\n", best.centralization, centralization);
        if(best.centralization > centralization && new_best==false){
            continue;
        }else if (best.centralization < centralization){
            new_best = true;
        }
        printf("tube (centralization): %ld\n", i);
        // // centralization
        // int outer_whisker_counter = fsmt_availability_counter(ith_tube->sensor->samples.whiskers.outer, lidar);
        // if(fabs(ith_tube->maneuver.radius) > 10)
        //     outer_whisker_counter = centralization;
        // // if(centralization == best.centralization){
        //     printf("best.outer_whisker_counter: %d, outer_whisker_counter: %d\n", best.outer_whisker_counter, outer_whisker_counter);
        //     if(best.outer_whisker_counter > outer_whisker_counter){
        //         continue;
        //     }
        // // }

        // // distance to orientation

        float distance_to_orientation = fsmt_orientation_to_local_goal(&ith_tube->polar,
            local_goal_orientation);
        distance_to_orientation =  
            fsmt_discrete_distance(distance_to_orientation, 0, M_PI/9);

        if(distance_to_orientation > best.distance_to_orientation && new_best==false){
            continue;
        }else if (distance_to_orientation < best.distance_to_orientation){
            new_best = true;
        }
        printf("tube (orientation): %ld\n", i);
        // // change in actuaction (continuous metric).
        // // float change_in_actuation = 0;
        // // if (control->velocity.forward > 0)
        // // {
        // //     float angular_rate = current_forward_vel/ith_tube->maneuver.radius;
        // //     change_in_actuation = fabs(current_angular_rate - angular_rate);
        // // }

        // // if(change_in_actuation > best.change_in_actuation){
        // //     continue;
        // // }

        best.index = i;
        best.distance_to_goal = distance_to_goal;
        best.centralization = centralization;
        // best.outer_whisker_counter = outer_whisker_counter;
        best.distance_to_orientation = distance_to_orientation;
        // // best.change_in_actuation = change_in_actuation;
        printf("tube #%ld, ", i);

        printf("distance_to_goal: %f.. ", distance_to_goal);
        printf("centralization: %f.. ", centralization);
        printf("distance_to_orientation: %f.", distance_to_orientation);
        printf("\n");
        // printf("best.distance_to_goal: %f, best.centralization: %d, best.outer_whisker_counter : %d, best.distance_to_orientation: %f\n",
        //     best.distance_to_goal, best.centralization, best.outer_whisker_counter, best.distance_to_orientation);
    }
    return best.index;
}

