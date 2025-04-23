
#include <fsmt/data_structure/polar_point.h>

#include <stdlib.h>
#include <errno.h>

fsmt_polar_point_array_t* fsmt_polar_point_array_create(size_t max_size)
{
    fsmt_polar_point_array_t *array = (fsmt_polar_point_array_t*) malloc(sizeof(fsmt_polar_point_array_t));
    if(array==NULL)
    {
        // Set error code.
        errno = ENOMEM;  
        return NULL;
    }

    array->points =  (fsmt_polar_point_t*) malloc(sizeof(fsmt_polar_point_t)*max_size);
    if(array->points == NULL)
    {
        fsmt_polar_point_array_destroy(&array);
        // Set error code.
        errno = ENOMEM;  
        return NULL;
    }

    array->max_size = max_size;

    fsmt_polar_point_array_reset(array);
    return array;    
}

void fsmt_polar_point_array_destroy(fsmt_polar_point_array_t** array)
{
    if(array==NULL)
    {
        return;
    }

    free((*array)->points);
    (*array)->points = NULL;
 
    free(*array);
}

void fsmt_polar_point_array_reset(fsmt_polar_point_array_t* array)
{
    array->size = 0;
}

int fsmt_polar_point_array_push(fsmt_polar_point_array_t* array, float radius, float angle)
{
    if(array == NULL || array->points == NULL)
    {
        return -1;
    }

    size_t index = array->size;
    if(index >= array->max_size)
    {
        return 0;
    }

    array->points[index].radius = radius;
    array->points[index].angle = angle; 
    array->size += 1;

    return 1;
}
