#include <fsmt/data_structure/cartesian_point.h>

#include <stdlib.h>
#include <errno.h>

fsmt_cartesian_point_array_t* fsmt_cartesian_point_array_create(size_t max_size)
{
    fsmt_cartesian_point_array_t *array = (fsmt_cartesian_point_array_t*) malloc(sizeof(fsmt_cartesian_point_array_t));
    if(array==NULL)
    {
        // Set error code.
        errno = ENOMEM;  
        return NULL;
    }

    array->points =  (fsmt_cartesian_point_t*) malloc(sizeof(fsmt_cartesian_point_t)*max_size);
    if(array->points == NULL)
    {
        fsmt_cartesian_point_array_destroy(&array);
        // Set error code.
        errno = ENOMEM;  
        return NULL;
    }

    array->max_size = max_size;

    fsmt_cartesian_point_array_reset(array);
    return array;    
}

void fsmt_cartesian_point_array_destroy(fsmt_cartesian_point_array_t** array)
{
    if(array==NULL)
    {
        return;
    }

    if(*array!=NULL)
    {
        free((*array)->points);
        (*array)->points = NULL;    
    }
 
    free(*array);
}

void fsmt_cartesian_point_array_reset(fsmt_cartesian_point_array_t* array)
{
    array->size = 0;
}

int fsmt_cartesian_point_array_push(fsmt_cartesian_point_array_t* array, float x, float y)
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

    array->points[index].x = x;
    array->points[index].y = y; 
    array->size += 1;

    return 1;
}
