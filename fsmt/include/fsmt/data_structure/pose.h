
/**
 * @file pose.h
 * @brief Header file of fsmt pose.
 * 
 * @date March 15, 2024
 * @author: Rômulo Rodrigues
 *
 *
 **/

#ifndef FSMT_DATA_STRUCTURE_POSE_H
#define FSMT_DATA_STRUCTURE_POSE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct fsmt_pose_s{
    float x, y, theta;
}fsmt_pose_t;

#ifdef __cplusplus
}  // extern C
#endif

#endif


