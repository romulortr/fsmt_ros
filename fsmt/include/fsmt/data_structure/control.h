
/**
 * @file control.h
 * @brief 
 * 
 * @date March 15, 2024
 * @author: Rômulo Rodrigues
 *
 *
 **/

 #ifndef FSMT_DATA_STRUCTURE_CONTROL_H
 #define FSMT_DATA_STRUCTURE_CONTROL_H

#include <stddef.h>
  
 #ifdef __cplusplus
 extern "C" {
 #endif


typedef struct fsmt_control_s{
    // measurement data.
    struct{
        float forward, lateral;
        float angular_rate;
    }velocity;
}fsmt_control_t;

#ifdef __cplusplus
}  // extern C
#endif
 
 #endif
 