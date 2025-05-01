/**
 * @file cartesian_tube.c
 * @brief Implementation of cartesian tube.
 *
 * Detailed description here.
 */

#include <stdlib.h>
#include <errno.h>
#include <math.h>

#include <fsmt/cartesian_tube.h>
#include <fsmt/sampling.h>
#include <fsmt/utils.h>

#include <fsmt/data_structure/transform.h>

#include <stdio.h>

fsmt_cartesian_tube_t* fsmt_cartesian_tube_create(size_t max_size)
{
    fsmt_cartesian_tube_t *tube = (fsmt_cartesian_tube_t *) malloc(sizeof(fsmt_cartesian_tube_t));
    if(tube==NULL){
        // Set error code.
        errno = ENOMEM;  
        return NULL;
    }

    tube->samples.edge = fsmt_cartesian_point_array_create(max_size);
    tube->samples.whiskers.outer = fsmt_cartesian_point_array_create((size_t) max_size/2);
    tube->samples.whiskers.inner = fsmt_cartesian_point_array_create((size_t) max_size/2);
    tube->samples.whiskers.front = fsmt_cartesian_point_array_create((size_t) max_size/2);

    if(tube->samples.edge == NULL ||
        tube->samples.whiskers.outer == NULL ||
        tube->samples.whiskers.inner == NULL ||
        tube->samples.whiskers.front == NULL )
    {
        fsmt_cartesian_tube_destroy(&tube);
        // Set error code.
        errno = ENOMEM;  
        return NULL;
    }

    fsmt_cartesian_tube_reset(tube);

    return tube;
}

void fsmt_cartesian_tube_destroy(fsmt_cartesian_tube_t** tube)
{
    if (tube == NULL)
        return;

    if (*tube != NULL)
    {
        fsmt_cartesian_point_array_destroy(&(*tube)->samples.edge);
        fsmt_cartesian_point_array_destroy(&(*tube)->samples.whiskers.inner);
        fsmt_cartesian_point_array_destroy(&(*tube)->samples.whiskers.outer);
        fsmt_cartesian_point_array_destroy(&(*tube)->samples.whiskers.front);
    }

    free(*tube);
    *tube = NULL;
}

void fsmt_cartesian_tube_reset(fsmt_cartesian_tube_t* tube)
{
    fsmt_cartesian_point_array_reset(tube->samples.edge);
    fsmt_cartesian_point_array_reset(tube->samples.whiskers.outer);
    fsmt_cartesian_point_array_reset(tube->samples.whiskers.inner);
    fsmt_cartesian_point_array_reset(tube->samples.whiskers.front);
}

void fsmt_cartesian_tube_configure(fsmt_cartesian_tube_t *tube, 
    fsmt_maneuver_t *maneuver, fsmt_params_t *params)
{
    float radius = maneuver->radius;
    float length = maneuver->length;   

    float width  = params->vehicle.width;
    float d_front = params->vehicle.distance_axle_to_front;
    float d_rear = params->vehicle.distance_axle_to_rear;
    // sampling step
    float edge_sampling_step = params->sampling.sampling_step.edge;
    float whiskers_sampling_step = params->sampling.sampling_step.whiskers;
    // whiskers params
    float side_whiskers_distance = params->sampling.whiskers_distance.side;
    float front_whiskers_distance = params->sampling.whiskers_distance.front;
    float r_min = 1.0/params->vehicle.maximum_curvature;

    float angular_displacement = length/radius; // negative or positive
    float r_inner = fabs(radius) - width/2;
    float r_outer = sqrtf(d_front*d_front + powf(fabs(radius)+width/2,2));        

    fsmt_cartesian_point_t p_outer, p_inner;

    if (radius > r_min)
    {           
       // Described at the ICP frame
        p_outer.x = width/2 + radius;
        p_outer.y = 1*d_front;
        p_inner.x = -width/2 + radius;
        p_inner.y = 0;        
    }
    else if (radius < -r_min)
    {                
        // Described at the ICP frame
        p_outer.x = width/2 - radius;
        p_outer.y = 1*(-d_front);
        p_inner.x = -width/2 - radius;
        p_inner.y = 0;
    }
    else
    {
        printf("RADIUS IS TOO SMALL!!!!!!!\n");
        return;
    }
    
    // Compute tangent at p_inner_final (final position for the inner radius).
    struct{
        float x, y;
    }tangent_at_p_inner_final;
    if(angular_displacement > 0){
        tangent_at_p_inner_final.x = -sinf(angular_displacement);
        tangent_at_p_inner_final.y = cosf(angular_displacement);
    }
    if(angular_displacement < 0){
        tangent_at_p_inner_final.x = sinf(angular_displacement);
        tangent_at_p_inner_final.y = -cosf(angular_displacement);
    }

    // Distance from reference point to the edge of the vehicle at final position.
    float d_inner_edge, d_outer_edge = 0;
    if(length > 0){
        d_inner_edge = d_front;
        d_outer_edge = 0;
    }
    if(length < 0){
        d_inner_edge = d_rear;
        d_outer_edge = d_front + d_rear;
    }

    float angular_offset;
    fsmt_cartesian_point_array_t *samples_edge = tube->samples.edge;
    // Outer radius.
    angular_offset = atan2f(p_outer.y, p_outer.x);
    fsmt_sample_circular_arc(samples_edge, r_outer, angular_displacement, 
        angular_offset, edge_sampling_step);

    // Outer line segment.
    fsmt_cartesian_point_t p_outer_final = samples_edge->points[samples_edge->size-1];
    fsmt_cartesian_point_t p_outer_edge_final = {
        .x = d_outer_edge*tangent_at_p_inner_final.x + p_outer_final.x,
        .y = d_outer_edge*tangent_at_p_inner_final.y + p_outer_final.y
    };
    fsmt_sample_line_segment(samples_edge, &p_outer_final, &p_outer_edge_final, 0.1);

    // Inner radius.
    angular_offset = atan2f(p_inner.y, p_inner.x);
    fsmt_sample_circular_arc(samples_edge, r_inner, angular_displacement,
        angular_offset, edge_sampling_step);

    // Inner line segment.
    fsmt_cartesian_point_t p_inner_final = samples_edge->points[samples_edge->size-1];
    fsmt_cartesian_point_t p_inner_edge_final = {
        .x = d_inner_edge*tangent_at_p_inner_final.x + p_inner_final.x,
        .y = d_inner_edge*tangent_at_p_inner_final.y + p_inner_final.y
    };
    fsmt_sample_line_segment(samples_edge, &p_inner_final, &p_inner_edge_final, edge_sampling_step);

    // Front edge of the vehicle at final position.
    fsmt_sample_line_segment(samples_edge, &p_outer_edge_final, &p_inner_edge_final, edge_sampling_step);

    // Whiskers
    // Outer whiskers
    fsmt_cartesian_point_array_t *samples_outer = tube->samples.whiskers.outer;
    angular_offset = atan2f(p_outer.y, p_outer.x);
    fsmt_sample_circular_arc(samples_outer, r_outer+side_whiskers_distance, angular_displacement, 
        angular_offset, whiskers_sampling_step);

    // Inner whiskers
    fsmt_cartesian_point_array_t *samples_inner = tube->samples.whiskers.inner;
    angular_offset = atan2f(p_inner.y, p_inner.x);
    fsmt_sample_circular_arc(samples_inner, r_inner-side_whiskers_distance, angular_displacement,
        angular_offset, whiskers_sampling_step);

    // front whiskers
    fsmt_cartesian_point_array_t *samples_front = tube->samples.whiskers.front;
    fsmt_cartesian_point_t p_outer_edge_front_whiskers = {
        .x = (d_outer_edge+front_whiskers_distance)*tangent_at_p_inner_final.x + p_outer_final.x,
        .y = (d_outer_edge+front_whiskers_distance)*tangent_at_p_inner_final.y + p_outer_final.y
    };
    fsmt_cartesian_point_t p_inner_edge_front_whiskers = {
        .x = (d_inner_edge+front_whiskers_distance)*tangent_at_p_inner_final.x + p_inner_final.x,
        .y = (d_inner_edge+front_whiskers_distance)*tangent_at_p_inner_final.y + p_inner_final.y
    };
    fsmt_sample_line_segment(samples_front, &p_outer_edge_front_whiskers, 
        &p_inner_edge_front_whiskers, whiskers_sampling_step);


    // Transform from ICP to body frame.    
    float angle = -radius/fabs(radius) * M_PI/2;
    fsmt_transform_t ICP_to_body_start_transform;      // ICP -> body frame at start time
    ICP_to_body_start_transform.x = 0;
    ICP_to_body_start_transform.y = radius;
    ICP_to_body_start_transform.cos_yaw = cosf(angle);    
    ICP_to_body_start_transform.sin_yaw = sinf(angle);
    fsmt_point_array_frame_transformation(&ICP_to_body_start_transform, samples_edge, samples_edge);
    fsmt_point_array_frame_transformation(&ICP_to_body_start_transform, samples_outer, samples_outer);
    fsmt_point_array_frame_transformation(&ICP_to_body_start_transform, samples_inner, samples_inner);
    fsmt_point_array_frame_transformation(&ICP_to_body_start_transform, samples_front, samples_front);

    // Transform final position of the vehicle.
    fsmt_transform_t body_final_to_ICP_transform;   // body frame at final time -> ICP
    body_final_to_ICP_transform.x = fabs(radius)*cosf(angular_displacement);
    body_final_to_ICP_transform.y = fabs(radius)*sinf(angular_displacement);
    body_final_to_ICP_transform.cos_yaw = cosf(-angle+angular_displacement);    
    body_final_to_ICP_transform.sin_yaw = sinf(-angle+angular_displacement);

    fsmt_transform_t body_final_to_body_start_transform;
    add_transformations(
        &ICP_to_body_start_transform, 
        &body_final_to_ICP_transform,
        &body_final_to_body_start_transform);

    fsmt_cartesian_point_t edge_start_time[4] =
    {
        {.x = d_front, .y = -width/4}, // front-right
        {.x = d_front, .y = width/4}, // front-left
        {.x = -d_rear, .y = width/4}, // rear-left
        {.x = -d_rear, .y = -width/4}, // rear-right
    };
    fsmt_cartesian_point_t edge_final_time[4];
    for(size_t i=0; i<4; i++)
    {
        point_frame_transformation(&body_final_to_body_start_transform,
            &edge_start_time[i], &edge_final_time[i]);
    }
    tube->at_final_time.p1_front_right = edge_final_time[0];
    tube->at_final_time.p2_front_left = edge_final_time[1];
    tube->at_final_time.p3_rear_left = edge_final_time[2];
    tube->at_final_time.p4_rear_right = edge_final_time[3];
}

