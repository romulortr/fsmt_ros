
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

#ifndef FSMT_NAVIGATION_H
#define FSMT_NAVIGATION_H

#include <fsmt/data_structure/lidar.h>
#include <fsmt/data_structure/control.h>

#include <fsmt/tube.h>


#ifdef __cplusplus
 extern "C" {
#endif

typedef struct fsmt_navigation_s{
    fsmt_tube_array_t *tubes;
    float length;
    fsmt_control_t control;
    fsmt_cartesian_point_t local_goal;
}fsmt_navigation_t;

fsmt_navigation_t* fsmt_navigation_create(size_t number_of_tubes, size_t number_of_samples);

void fsmt_navigation_destroy(fsmt_navigation_t **navigation);

void fsmt_navigation_reset(fsmt_navigation_t *navigation);

int fsmt_evaluate(fsmt_navigation_t *navigation, fsmt_lidar_t *lidar, fsmt_cartesian_point_array_t *plan,
    float *orientation, fsmt_control_t *control);

#ifdef __cplusplus
}  // extern C
#endif
 
 #endif
 