
/**
 * @file polar_tube.h
 * @brief Header file of polar tube
 * 
 * @date March 15, 2022
 * @author: Rômulo Rodrigues
 *
 * Detailed description here.
 *
 **/

#ifndef FSMT_POLAR_TUBE_H
#define FSMT_POLAR_TUBE_H

#include <stddef.h>
#include <stdbool.h>

#include <fsmt/data_structure/maneuver.h>
#include <fsmt/data_structure/params.h>
#include <fsmt/data_structure/polar_point.h>
#include <fsmt/data_structure/transform.h>

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct fsmt_polar_tube_s{
    fsmt_transform_t transform;
    struct{
        float inner;
        float outer;
        float center;
    }radius;

    struct{
        float length;
        struct{
            float min, max, final;
        }angle;
    }limits;
}fsmt_polar_tube_t;

int fsmt_polar_tube_compute(fsmt_polar_tube_t *tube, 
    fsmt_maneuver_t *maneuver, fsmt_params_t *params);

#ifdef __cplusplus
}  // extern C
#endif
 
 #endif
 