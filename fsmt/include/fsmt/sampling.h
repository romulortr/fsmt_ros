
/**
 * @file sampling.h
 * @brief 
 * 
 * @date March 15, 2022
 * @author: Rômulo Rodrigues
 *
 * Detailed description here.
 *
 **/

#ifndef FSMT_SAMPLING_H
#define FSMT_SAMPLING_H

#include <stddef.h>
#include <stdbool.h>

#include <fsmt/data_structure/lidar.h>
#include <fsmt/data_structure/cartesian_point.h>
#include <fsmt/data_structure/pose.h>

#ifdef __cplusplus
extern "C" {
#endif

void fsmt_sample_circular_arc(fsmt_cartesian_point_array_t *samples, float radius, 
    float angular_displacement, float angular_offset, float spatial_step);

void fsmt_sample_line_segment(fsmt_cartesian_point_array_t *samples, 
    fsmt_cartesian_point_t *p_start, fsmt_cartesian_point_t *p_end, float spatial_step);

#ifdef __cplusplus
}  // extern C
#endif

#endif
