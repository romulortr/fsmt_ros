/**
 * @file cartesian_tube.c
 * @brief Implementation of cartesian tube.
 *
 * Detailed description here.
 */

#include <fsmt/sensor_tube.h>
 #include <fsmt/sampling.h>
#include <fsmt/utils.h>

#include <fsmt/data_structure/transform.h>

#include <stdlib.h>
#include <errno.h>

#include <math.h>

#include <stdio.h>  // printf

fsmt_sensor_tube_t* fsmt_sensor_tube_create(size_t max_size)
{
    fsmt_sensor_tube_t *tube = (fsmt_sensor_tube_t *) malloc(sizeof(fsmt_sensor_tube_t));
    if(tube==NULL){
        // Set error code.
        errno = ENOMEM;  
        return NULL;
    }
    tube->samples.edge = fsmt_sensor_point_array_create(max_size);
    tube->samples.whiskers.outer = fsmt_sensor_point_array_create((size_t) max_size/2);
    tube->samples.whiskers.inner = fsmt_sensor_point_array_create((size_t) max_size/2);
    tube->samples.whiskers.front = fsmt_sensor_point_array_create((size_t) max_size/2);
    
    if(tube->samples.edge == NULL ||
        tube->samples.whiskers.outer == NULL ||
        tube->samples.whiskers.inner == NULL ||
        tube->samples.whiskers.front == NULL )
    {
        fsmt_sensor_tube_destroy(&tube);
        // Set error code.
        errno = ENOMEM;  
        return NULL;
    }

    fsmt_sensor_tube_reset(tube);

    return tube;
}

void fsmt_sensor_tube_destroy(fsmt_sensor_tube_t** tube)
{
    if (tube == NULL)
        return;

    if (*tube != NULL)
    {
        fsmt_sensor_point_array_destroy(&(*tube)->samples.edge);
        fsmt_sensor_point_array_destroy(&(*tube)->samples.whiskers.inner);
        fsmt_sensor_point_array_destroy(&(*tube)->samples.whiskers.outer);  
        fsmt_sensor_point_array_destroy(&(*tube)->samples.whiskers.front);  
    }

    free(*tube);
    *tube = NULL;
}

void fsmt_sensor_tube_reset(fsmt_sensor_tube_t* tube)
{
    fsmt_sensor_point_array_reset(tube->samples.edge);
    fsmt_sensor_point_array_reset(tube->samples.whiskers.outer);
    fsmt_sensor_point_array_reset(tube->samples.whiskers.inner);
    fsmt_sensor_point_array_reset(tube->samples.whiskers.front);
}

void fsmt_sensor_tube_from_cartesian_tube(fsmt_sensor_tube_t *sensor_tube, 
    fsmt_cartesian_tube_t *cartesian_tube, fsmt_lidar_t *lidar)
{
    fsmt_cartesian_point_array_t *cartesian_array[4] = {
        cartesian_tube->samples.edge, cartesian_tube->samples.whiskers.inner,
        cartesian_tube->samples.whiskers.outer, cartesian_tube->samples.whiskers.front  
    };

    fsmt_sensor_point_array_t *sensor_array[4] = {
        sensor_tube->samples.edge, sensor_tube->samples.whiskers.inner,
        sensor_tube->samples.whiskers.outer, sensor_tube->samples.whiskers.front 
    };

    for(size_t i=0; i<4; i++)
    {
        fsmt_cartesian_point_array_t *ith_cartesian_array = cartesian_array[i];
        fsmt_sensor_point_array_t *ith_sensor_array = sensor_array[i];
        if(ith_cartesian_array->size > ith_sensor_array->max_size)
        {
            return ;
        }
    
        float x=-0.055, y=0, theta=0;
        float c = cosf(theta);
        float s = sinf(theta);
    
        ith_sensor_array->size = 0;
        for(size_t i=0; i<ith_cartesian_array->size; i++)
        {
            float xi = ith_cartesian_array->points[i].x;
            float yi = ith_cartesian_array->points[i].y;
            float xl = c*xi - s*yi + x;
            float yl = s*xi + c*yi + y;
    
            float beam_angle = atan2f(yl, xl);
            // @TODO: check when min and max are inverted.
            if(beam_angle < lidar->config.angle.min || beam_angle > lidar->config.angle.max){
                continue;
            }
    
    
            float distance = sqrtf(yl*yl + xl*xl);
            if(distance < lidar->config.range.min || distance > lidar->config.range.max){
                continue;
            }
    
            ith_sensor_array->points[ith_sensor_array->size].range1 = distance;
            ith_sensor_array->points[ith_sensor_array->size].index = (size_t) roundf((beam_angle - 
                lidar->config.angle.min)/lidar->config.angular_step);
            ith_sensor_array->size += 1;
        }    
    }

}