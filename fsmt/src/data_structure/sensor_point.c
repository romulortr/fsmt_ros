#include <fsmt/data_structure/sensor_point.h>

#include <stdlib.h>
#include <errno.h>

fsmt_sensor_point_array_t* fsmt_sensor_point_array_create(size_t max_size)
{
    fsmt_sensor_point_array_t *array = (fsmt_sensor_point_array_t*) malloc(sizeof(fsmt_sensor_point_array_t));
    if(array==NULL)
    {
        // Set error code.
        errno = ENOMEM;  
        return NULL;
    }

    array->points =  (fsmt_sensor_point_t*) malloc(sizeof(fsmt_sensor_point_t)*max_size);
    if(array->points == NULL)
    {
        fsmt_sensor_point_array_destroy(&array);
        // Set error code.
        errno = ENOMEM;  
        return NULL;
    }

    array->max_size = max_size;

    fsmt_sensor_point_array_reset(array);
    return array;    
}

void fsmt_sensor_point_array_destroy(fsmt_sensor_point_array_t** array)
{
    if(array==NULL)
    {
        return;
    }

    if(*array != NULL)
    {
        free((*array)->points);
        (*array)->points = NULL;   
    }
 
    free(*array);
}

void fsmt_sensor_point_array_reset(fsmt_sensor_point_array_t* array)
{
    array->size = 0;
}
