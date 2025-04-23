
/**
 * @file sensor_point.h
 * @brief Header file of fsmt point.
 * 
 * @date March 15, 2024
 * @author: Rômulo Rodrigues
 *
 *
 **/

#ifndef FSMT_DATA_STRUCTURE_SENSOR_POINT_H
#define FSMT_DATA_STRUCTURE_SENSOR_POINT_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct fsmt_sensor_point_s{
    float range1;
    size_t index;
}fsmt_sensor_point_t;

typedef struct fsmt_sensor_point_array_s{
    fsmt_sensor_point_t *points;
    size_t max_size;
    size_t size;
}fsmt_sensor_point_array_t;
 
fsmt_sensor_point_array_t* fsmt_sensor_point_array_create(size_t max_size);

void fsmt_sensor_point_array_destroy(fsmt_sensor_point_array_t** array);

void fsmt_sensor_point_array_reset(fsmt_sensor_point_array_t* array);

#ifdef __cplusplus
}  // extern C
#endif

#endif
