
/**
 * @file cartesian_tube.h
 * @brief Header file for cartesian tube.
 * 
 * @date March 15, 2025
 * @author: Rômulo Rodrigues
 *
 * Detailed description here.
 *
 **/

#ifndef FSMT_NAVIGATION_H
#define FSMT_NAVIGATION_H

#include <fsmt/data_structure/lidar.h>
#include <fsmt/data_structure/control.h>

#include <fsmt/tube.h>


#ifdef __cplusplus
 extern "C" {
#endif

#define FSMT_STATE_NORMAL 0
#define FSMT_STATE_STOP_TO_RECOVER 1 
#define FSMT_STATE_START_RECOVER 2
#define FSMT_STATE_RECOVERY_MOVE_BACKWARD 3
#define FSMT_STATE_STOP_TO_ROTATE 4
#define FSMT_STATE_RECOVERY_ROTATE 5
#define FSMT_STATE_STOP_TO_NORMAL 6

typedef struct fsmt_navigation_s{
    // 
    fsmt_tube_array_t *tubes[5];
    fsmt_tube_array_t *nominal_horizon;
    fsmt_tube_array_t *long_horizon;
    struct{
        fsmt_tube_array_t *backward;    
        struct{
            fsmt_sensor_point_array_t *sensor;
            fsmt_cartesian_point_array_t *cartesian;            
        }rotate;
    }recovery;
    float length;
    fsmt_control_t control;
    fsmt_cartesian_point_t local_goal;
    int state;
    

    struct{
        // Stores arrays' indices to speed up access.
        struct{
            int horizon;
            int tube;
        }index;
        // Pointer to selected tube.
        fsmt_tube_t *tube;
        int number_of_feasible_tubes;   ///!< number of tubes within the free-space [0,..],[unit].
        float rate_of_feasible_tubes;   ///!< ratio between feasible tubes and total number of tubes [0,1],[unitless].
    }solution;

}fsmt_navigation_t;

fsmt_navigation_t* fsmt_navigation_create(size_t number_of_tubes, size_t number_of_samples);

void fsmt_navigation_destroy(fsmt_navigation_t **navigation);

void fsmt_navigation_reset(fsmt_navigation_t *navigation);

void fsmt_navigation_configure(fsmt_navigation_t *navigation, fsmt_params_t *params, fsmt_lidar_t *lidar);

int fsmt_evaluate(fsmt_navigation_t *navigation, fsmt_lidar_t *lidar, fsmt_cartesian_point_array_t *plan,
    float *orientation, fsmt_control_t *control);

#ifdef __cplusplus
}  // extern C
#endif
 
#endif
 