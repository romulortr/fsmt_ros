
#include <fsmt/sampling.h>

#include <stdint.h>
#include <math.h>
#include <stdio.h>

// if (samples->number_of_points >= samples->max_number_of_points){   
//     break;
// }
// Transform to reference frame

/**
 * @param radius always positive
 * @param angular_displacement positive or negative
 * @param spatial_step always positive
 */
void fsmt_sample_circular_arc(fsmt_cartesian_point_array_t *samples, float radius, 
    float angular_displacement, float angular_offset, float spatial_step)
{
    int signal = angular_displacement/fabs(angular_displacement);
    float angular_step = signal*spatial_step/radius;   // can be positive or negative

    float number_of_samples_float = roundf(angular_displacement/angular_step);
    if(number_of_samples_float < 1) return;

    size_t number_of_samples = (size_t) roundf(angular_displacement/angular_step); 
    size_t *index = &samples->size;

    for (size_t i=0; i<number_of_samples; i++)
    {
        float angle = angular_step*i + angular_offset;
        // Described on the ICP frame.
        samples->points[*index].x = radius*cosf(angle);
        samples->points[*index].y = radius*sinf(angle);

        *index += 1;
    }     
}

void fsmt_sample_line_segment(fsmt_cartesian_point_array_t *samples, 
    fsmt_cartesian_point_t *p_start, fsmt_cartesian_point_t *p_end, float spatial_step)
{
    float length = sqrtf(powf(p_start->x - p_end->x,2) + powf(p_start->y - p_end->y,2));
    if(length < spatial_step){
        return;
    }

    float dx = (p_end->x - p_start->x)/length;
    float dy = (p_end->y - p_start->y)/length;
    size_t number_of_samples = (int) ceil(length/spatial_step) + 1;

    // Refine sample interval to contain end-points
    float sampling_step = length/(number_of_samples-1);

    size_t *index = &samples->size;
    for(int i=0; i<number_of_samples; i++){
        samples->points[*index].x = p_start->x + i*sampling_step*dx;
        samples->points[*index].y = p_start->y + i*sampling_step*dy;
        *index += 1;
    }    
}

