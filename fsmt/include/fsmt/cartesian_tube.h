
/**
 * @file cartesian_tube.h
 * @brief Header file for cartesian tube.
 * 
 * @date March 15, 2025
 * @author: Rômulo Rodrigues
 *
 * Detailed description here.
 *
 **/

#ifndef FSMT_CARTESIAN_TUBE_H
#define FSMT_CARTESIAN_TUBE_H

#include <fsmt/data_structure/maneuver.h>
#include <fsmt/data_structure/params.h>
#include <fsmt/data_structure/cartesian_point.h>

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct fsmt_cartesian_tube_s{
    struct{
        fsmt_cartesian_point_array_t *edge;
        struct{
            fsmt_cartesian_point_array_t *outer, *inner;
        }whiskers;
    }samples;

    struct{
        fsmt_cartesian_point_t p1_front_right;
        fsmt_cartesian_point_t p2_front_left;
        fsmt_cartesian_point_t p3_rear_left;
        fsmt_cartesian_point_t p4_rear_right;
    }at_final_time;    
}fsmt_cartesian_tube_t;

fsmt_cartesian_tube_t* fsmt_cartesian_tube_create(size_t max_size);

void fsmt_cartesian_tube_destroy(fsmt_cartesian_tube_t** tube);

void fsmt_cartesian_tube_reset(fsmt_cartesian_tube_t* tube);

void fsmt_cartesian_tube_compute(fsmt_cartesian_tube_t *samples, fsmt_params_t *params, 
    fsmt_maneuver_t *maneuver);

#ifdef __cplusplus
}  // extern C
#endif
 
 #endif
 