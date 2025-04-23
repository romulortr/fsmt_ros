
/**
 * @file score.h
 * @brief Header file for free space motion tube.
 * 
 * @date March 15, 2022
 * @author: Rômulo Rodrigues
 *
 * Detailed description here.
 *
 **/

#ifndef FSMT_SCORE_H
#define FSMT_SCORE_H

#include <stddef.h>
#include <stdbool.h>

#include <fsmt/data_structure/cartesian_point.h>
#include <fsmt/data_structure/polar_point.h>
#include <fsmt/data_structure/sensor_point.h>
#include <fsmt/data_structure/lidar.h>

#include <fsmt/cartesian_tube.h>
#include <fsmt/polar_tube.h>
#include <fsmt/sensor_tube.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct fsmt_circle_s{
    float xc, yc;
    float radius;
}fsmt_circle_t;

int fsmt_availability(const fsmt_sensor_point_array_t* sensor, const fsmt_lidar_t *lidar);

int fsmt_availability_counter(const fsmt_sensor_point_array_t* sensor, const fsmt_lidar_t *lidar);

float fsmt_distance_to_local_goal(const fsmt_cartesian_tube_t *tube, fsmt_cartesian_point_t *test);

float fsmt_orientation_to_local_goal(const fsmt_polar_tube_t *tube, float desired_final_orientation);

int fsmt_whisker_distance(const fsmt_sensor_tube_t *tube, const fsmt_lidar_t *lidar);

int fsmt_distance_to_plan(fsmt_cartesian_point_array_t* plan, fsmt_circle_t *circle);

int fsmt_circle_fitting_kasa(fsmt_cartesian_point_array_t* array, fsmt_circle_t *circle);



#ifdef __cplusplus
}  // extern C
#endif

  #endif
  