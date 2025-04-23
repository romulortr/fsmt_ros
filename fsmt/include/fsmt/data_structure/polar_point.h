
/**
 * @file polar_point.h
 * @brief Header file of fsmt polar_point.
 * 
 * @date March 15, 2024
 * @author: Rômulo Rodrigues
 *
 *
 **/

#ifndef FSMT_DATA_STRUCTURE_POLAR_POINT_H
#define FSMT_DATA_STRUCTURE_POLAR_POINT_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct fsmt_polar_point_s{
    float radius;
    float angle;
}fsmt_polar_point_t;

typedef struct fsmt_polar_point_array_s{
    fsmt_polar_point_t *points;
    size_t size;
    size_t max_size;
}fsmt_polar_point_array_t;

fsmt_polar_point_array_t* fsmt_polar_point_array_create(size_t max_size);

void fsmt_polar_point_array_destroy(fsmt_polar_point_array_t** array);

void fsmt_polar_point_array_reset(fsmt_polar_point_array_t* array);

int fsmt_polar_point_array_push(fsmt_polar_point_array_t* array, float x, float y);

#ifdef __cplusplus
}  // extern C
#endif
 
#endif
 