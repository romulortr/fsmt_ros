/**
 * @file fsmt.c
 * @brief Implementation of XYZ feature.
 *
 * Detailed description here.
 */

#include <fsmt/tube.h>
#include <fsmt/sampling.h>
#include <fsmt/data_structure/pose.h>
#include <fsmt/data_structure/sensor_point.h>

#include <stdint.h>
#include <stdio.h>

#include <stdlib.h>
#include <errno.h>

/**
 * @brief Creates and initializes values in a variable.
*/
fsmt_tube_t* fsmt_tube_create(size_t max_size){
    fsmt_tube_t *tube = (fsmt_tube_t*) malloc(sizeof(fsmt_tube_t));
    if(tube==NULL){
        // Set error code.
        errno = ENOMEM;  
        return NULL;
    }

    tube->cartesian = fsmt_cartesian_tube_create(max_size);
    tube->sensor = fsmt_sensor_tube_create(max_size);

    if(tube->cartesian==NULL || tube->sensor==NULL){
        fsmt_tube_destroy(&tube);
        // Set error code.
        errno = ENOMEM;  
        return NULL;
    }

    fsmt_tube_reset(tube);

    return tube;
}

void fsmt_tube_reset(fsmt_tube_t* tube){
    if (tube==NULL)
        return;

    fsmt_cartesian_tube_reset(tube->cartesian);
    fsmt_sensor_tube_reset(tube->sensor);
}

/**
 * @brief Releases memory
*/
void fsmt_tube_destroy(fsmt_tube_t** tube){
    if (tube==NULL)
        return;

    fsmt_cartesian_tube_destroy(&(*tube)->cartesian);
    fsmt_sensor_tube_destroy(&(*tube)->sensor);

    free(*tube);
    *tube = NULL;
}

/**
 * @brief Creates and initializes values in a variable.
*/
fsmt_tube_array_t* fsmt_tube_array_create(size_t number_of_tubes, size_t number_of_samples)
{
    fsmt_tube_array_t *array = (fsmt_tube_array_t*) malloc(sizeof(fsmt_tube_array_t));
    if(array==NULL)
    {
        // Set error code.
        errno = ENOMEM;  
        return NULL;
    }

    array->tube = (fsmt_tube_t**) malloc(sizeof(fsmt_tube_t*)*number_of_tubes);
    for(size_t i=0; i<number_of_tubes; i++)
    {
        array->tube[i] = fsmt_tube_create(number_of_samples);
    }

    array->max_size = number_of_tubes;
    array->size = 0;

    fsmt_tube_array_reset(array);

    return array;
}

void fsmt_tube_array_reset(fsmt_tube_array_t* array){
    printf("INSIDE RESET FUNC\n");
    if (array==NULL)
        return;
    printf("INSIDE RESET SIZE: %ld \n", array->max_size);
    for(size_t i=0; i<array->max_size; i++)
    {
        printf("INSIDE RESET index %ld \n", i);
        fsmt_tube_reset(array->tube[i]);
    }
}

/**
 * @brief Releases memor
*/
void fsmt_tube_array_destroy(fsmt_tube_array_t** array){
    if (array==NULL)
        return;

    for(size_t i=0; i<(*array)->max_size; i++)
    {
        fsmt_tube_destroy(&(*array)->tube[i]);
    }

    free(*array);
    *array = NULL;
}
