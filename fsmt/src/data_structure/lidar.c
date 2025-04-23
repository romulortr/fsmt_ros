#include <fsmt/data_structure/lidar.h>

#include <stdlib.h>
#include <errno.h>

fsmt_lidar_t* fsmt_lidar_create(size_t max_number_of_beams)
{
    fsmt_lidar_t *lidar = (fsmt_lidar_t *) malloc(sizeof(fsmt_lidar_t));

    if(lidar==NULL){
        // Set error code.
        errno = ENOMEM;  
        return NULL;
    }

    lidar->measurements.ranges = (float*) malloc(sizeof(float) * max_number_of_beams);
    
    if(lidar->measurements.ranges == NULL){
        // Release memory.
        fsmt_lidar_destroy(&lidar);
        // Set error code.
        errno = ENOMEM;  
        return NULL;
    }

    lidar->measurements.size = 0;
    lidar->config.size.max = max_number_of_beams;
    lidar->config.angle.max = 0;
    lidar->config.angle.min = 0;
    lidar->config.range.max = 0;
    lidar->config.range.min = 0;
    lidar->config.angular_step = 0;

    return lidar;
}

void fsmt_lidar_destroy(fsmt_lidar_t** lidar)
{
    if (lidar==NULL)
        return;
    
    free((*lidar)->measurements.ranges);
    (*lidar)->measurements.ranges = NULL;
    free(*lidar);
    *lidar = NULL;
}

int fsmt_lidar_config(fsmt_lidar_t* lidar, float angle_min, float angle_max, float range_min, float range_max, float angular_step)
{
    if (lidar == NULL){
        return -1;
    }

    lidar->config.angle.min = angle_min;
    lidar->config.angle.max = angle_max;
    lidar->config.range.min = range_min;
    lidar->config.range.max = range_max;
    lidar->config.angular_step = angular_step;

    return 1;
}
