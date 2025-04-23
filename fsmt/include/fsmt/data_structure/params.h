
/**
 * @file params.h
 * @brief Header file of fsmt params.
 * 
 * @date March 15, 2025
 * @author: Rômulo Rodrigues
 *
 *
 **/

#ifndef FSMT_DATA_STRUCTURE_PARAMS_H
#define FSMT_DATA_STRUCTURE_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct fsmt_vehicle_params_s{
    float width;
    float lenght;
    float distance_axle_to_front;
    float distance_axle_to_rear;
    float maximum_curvature;
}fsmt_vehicle_params_t;

typedef struct fsmt_sampling_params_s{
    float spatial_step;
}fsmt_sampling_params_t;

typedef struct fsmt_params_s{
    fsmt_sampling_params_t sampling;
    fsmt_vehicle_params_t vehicle;
}fsmt_params_t;

 #ifdef __cplusplus
 }  // extern C
 #endif
 
 #endif
 