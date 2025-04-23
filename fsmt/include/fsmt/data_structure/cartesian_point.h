
/**
 * @file point.h
 * @brief Header file of fsmt point.
 * 
 * @date March 15, 2024
 * @author: Rômulo Rodrigues
 *
 *
 **/

#ifndef FSMT_DATA_STRUCTURE_POINT_H
#define FSMT_DATA_STRUCTURE_POINT_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct fsmt_cartesian_point_s{
    float x, y;
}fsmt_cartesian_point_t;

typedef struct fsmt_cartesian_point_array_s{
    fsmt_cartesian_point_t *points;
    size_t max_size;
    size_t size;
}fsmt_cartesian_point_array_t;

fsmt_cartesian_point_array_t* fsmt_cartesian_point_array_create(size_t max_size);

void fsmt_cartesian_point_array_destroy(fsmt_cartesian_point_array_t** array);

void fsmt_cartesian_point_array_reset(fsmt_cartesian_point_array_t* array);

int fsmt_cartesian_point_array_push(fsmt_cartesian_point_array_t* array, float x, float y);

#ifdef __cplusplus
}  // extern C
#endif
 
#endif
 