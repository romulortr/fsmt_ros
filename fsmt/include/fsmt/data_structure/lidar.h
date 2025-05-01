
/**
 * @file lidar.h
 * @brief Header file for free space motion tube.
 * 
 * @date March 15, 2024
 * @author: Rômulo Rodrigues
 *
 *
 **/

 #ifndef FSMT_DATA_STRUCTURE_LIDAR_H
 #define FSMT_DATA_STRUCTURE_LIDAR_H

#include <stddef.h>
  
 #ifdef __cplusplus
 extern "C" {
 #endif


typedef struct fsmt_lidar_s{
    // measurement data.
    struct{
        float *ranges;
        size_t size;
    }measurements;
    // configuration data.
    struct{
        struct{
            float min, max;
        }angle, range;    
        struct{
            size_t max;
        }size;
        float angular_step;
    }config;
}fsmt_lidar_t;

/**
 * @brief Creates a variable of type fsmt_lidar_t.
 *
 * This function allocates memory and initializes the fields of an instantiation of fsmt_lidar_t. 
 * Memory allocation errors can be handled with errno.
 *
 * @return valid pointer on success, NULL on failure. 
 */
fsmt_lidar_t* fsmt_lidar_create(size_t number_of_beams);

/**
 * @brief Cleans up resources.
 *
 * This function frees any allocated memory and performs cleanup tasks.
 */
void fsmt_lidar_destroy(fsmt_lidar_t** fsmt);

int fsmt_lidar_config(fsmt_lidar_t *lidar, float angle_min, float angle_max, float range_min, float range_max, float angular_step);

#ifdef __cplusplus
}  // extern C
#endif
 
 #endif
 