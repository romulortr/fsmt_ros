
/**
 * @file sensor_tube.h
 * @brief Header file for cartesian tube.
 * 
 * @date March 15, 2025
 * @author: Rômulo Rodrigues
 *
 * Detailed description here.
 *
 **/

#ifndef FSMT_SENSOR_TUBE_H
#define FSMT_SENSOR_TUBE_H

#include <fsmt/data_structure/maneuver.h>
#include <fsmt/data_structure/params.h>
#include <fsmt/data_structure/sensor_point.h>
#include <fsmt/data_structure/lidar.h>

#include <fsmt/cartesian_tube.h>

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct fsmt_sensor_tube_s{
    struct{
        fsmt_sensor_point_array_t *edge;
        struct{
            fsmt_sensor_point_array_t *outer, *inner;
        }whiskers;
    }samples;
}fsmt_sensor_tube_t;

fsmt_sensor_tube_t* fsmt_sensor_tube_create(size_t max_size);

void fsmt_sensor_tube_destroy(fsmt_sensor_tube_t** tube);

void fsmt_sensor_tube_reset(fsmt_sensor_tube_t* tube);

void fsmt_sensor_tube_from_cartesian_tube(fsmt_sensor_tube_t *sensor, 
    fsmt_cartesian_tube_t *cartesian, fsmt_lidar_t *lidar);

#ifdef __cplusplus
}  // extern C
#endif
 
 #endif
 