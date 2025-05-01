/**
 * @file utils.h
 * @brief Header file of fsmt utils.
 * 
 * @date March 15, 2024
 * @author: Rômulo Rodrigues
 *
 *
 **/

#ifndef FSMT_UTILS_H
#define FSMT_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <fsmt/data_structure/transform.h>
#include <fsmt/data_structure/cartesian_point.h>
#include <fsmt/data_structure/polar_point.h>

float fsmt_euclidean_distance(const fsmt_cartesian_point_t *p1, 
    const fsmt_cartesian_point_t *p2);
    
float fsmt_distance_point_to_segment(const fsmt_cartesian_point_t *p, 
    const fsmt_cartesian_point_t *v1, const fsmt_cartesian_point_t *v2);

int fsmt_point_within_rectangle(const fsmt_cartesian_point_t *p1, 
    const fsmt_cartesian_point_t *p2, const fsmt_cartesian_point_t *p3, 
    const fsmt_cartesian_point_t *p4, const fsmt_cartesian_point_t *ptest);

float fsmt_distance_to_rectangle(const fsmt_cartesian_point_t *p1, 
    const fsmt_cartesian_point_t *p2, const fsmt_cartesian_point_t *p3, 
    const fsmt_cartesian_point_t *p4, const fsmt_cartesian_point_t *ptest);

int fsmt_discrete_distance(float number, float reference, float delta);

int fsmt_circle_ray_intersection(const fsmt_cartesian_point_t *po, float angle, float r,
    fsmt_cartesian_point_t *p_circle);

void add_transformations(const fsmt_transform_t *t1, const fsmt_transform_t *t2,
    fsmt_transform_t *t_out);

void invert_transformation(const fsmt_transform_t *t_in, fsmt_transform_t *t_out);

void point_frame_transformation(const fsmt_transform_t *transform, 
    const fsmt_cartesian_point_t *in, fsmt_cartesian_point_t *out);

void fsmt_point_array_frame_transformation(const fsmt_transform_t *transform, 
    const fsmt_cartesian_point_array_t *in, fsmt_cartesian_point_array_t *out);

int cartesian_array_to_polar_array(const fsmt_cartesian_point_array_t *cartesian, 
    fsmt_polar_point_array_t *polar);

float fsmt_dot_product(const fsmt_cartesian_point_t *p1, 
    const fsmt_cartesian_point_t *p2);

float wrap_to_pi(float angle);

#ifdef __cplusplus
}  // extern C
#endif

#endif