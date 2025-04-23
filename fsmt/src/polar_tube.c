/**
 * @file fsmt.c
 * @brief Implementation of XYZ feature.
 *
 * Detailed description here.
 */

#include <fsmt/polar_tube.h>

#include <math.h>

int fsmt_polar_tube_compute(fsmt_polar_tube_t *tube, fsmt_maneuver_t *maneuver, fsmt_params_t *params)
{
    float radius = maneuver->radius;
    float length = maneuver->length;   
    float width  = params->vehicle.width;
    float d_front = params->vehicle.distance_axle_to_front;

    tube->limits.length = length;
    tube->radius.center = fabs(radius);
    tube->radius.inner = fabs(radius) - width/2;
    tube->radius.outer = sqrtf(d_front*d_front + powf(fabs(radius)+width/2,2));

    float angular_displacement = length/radius; // negative or positive
    if(angular_displacement > 0){
        tube->limits.angle.min = 0;
        tube->limits.angle.max = angular_displacement;
    }
    else{
        tube->limits.angle.min = angular_displacement;
        tube->limits.angle.max = 0;
    }
    tube->limits.angle.final = angular_displacement;

    float angle = -radius/fabs(radius) * M_PI/2;
    
    tube->transform.x = 0;
    tube->transform.x = radius;
    tube->transform.cos_yaw = cosf(angle);    
    tube->transform.cos_yaw = sinf(angle);
}

