
/**
 * @file transform.h
 * @brief Header file of fsmt transform.
 * 
 * @date March 15, 2024
 * @author: Rômulo Rodrigues
 *
 *
 **/

#ifndef FSMT_DATA_STRUCTURE_TRANSFORM_H
#define FSMT_DATA_STRUCTURE_TRANSFORM_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct fsmt_transform_s{
    float cos_yaw, sin_yaw;
    float x, y;
}fsmt_transform_t;

#ifdef __cplusplus
}  // extern C
#endif

#endif


