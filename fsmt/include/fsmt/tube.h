
/**
 * @file fsmt.h
 * @brief Header file for free space motion tube.
 * 
 * @date March 15, 2022
 * @author: Rômulo Rodrigues
 *
 * Detailed description here.
 *
 **/

#ifndef FSMT_TUBE_H
#define FSMT_TUBE_H

#include <stddef.h>
#include <stdbool.h>

#include <fsmt/data_structure/lidar.h>
#include <fsmt/data_structure/cartesian_point.h>
#include <fsmt/data_structure/maneuver.h>
#include <fsmt/data_structure/sensor_point.h>

#include <fsmt/polar_tube.h>
#include <fsmt/cartesian_tube.h>
#include <fsmt/sensor_tube.h>

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct fsmt_tube_s{
    fsmt_polar_tube_t polar;
    fsmt_cartesian_tube_t *cartesian;
    fsmt_sensor_tube_t *sensor;
    fsmt_maneuver_t maneuver;
}fsmt_tube_t;

typedef struct fsmt_tube_array_s{
    fsmt_tube_t **tube;
    size_t max_size;
    size_t size;
    fsmt_cartesian_point_t local_goal;
    float local_orientation;
    float length;
}fsmt_tube_array_t;

/**
 * @brief Creates a variable of type FSMT_t.
 *
 * This function allocates memory and initializes the fields of an instantiation of FSMT_t. 
 * Memory allocation errors can be handled with errno.
 *
 * @return valid pointer on success, NULL on failure. 
 */
fsmt_tube_t* fsmt_tube_create(size_t max_size);

/**
 * @brief Cleans up resources.
 *
 * This function frees any allocated memory and performs cleanup tasks.
 */
void fsmt_tube_destroy(fsmt_tube_t** tube);

void fsmt_tube_reset(fsmt_tube_t* tube);


fsmt_tube_array_t* fsmt_tube_array_create(size_t number_of_tubes, size_t number_of_samples);
/**
 * @brief Cleans up resources.
 *
 * This function frees any allocated memory and performs cleanup tasks.
 */
void fsmt_tube_array_destroy(fsmt_tube_array_t** array);

void fsmt_tube_array_reset(fsmt_tube_array_t* array);

#ifdef __cplusplus
}  // extern C
#endif
 
 #endif
 