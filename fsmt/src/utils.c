#include <fsmt/utils.h>

#include <math.h>

// Compute Euclidean distance between two points
float fsmt_euclidean_distance(const fsmt_cartesian_point_t *p1, 
    const fsmt_cartesian_point_t *p2) 
{
    return sqrtf((p1->x - p2->x)*(p1->x - p2->x) + (p1->y - p2->y)*(p1->y - p2->y));
}

// Compute the shortest distance from point p to line segment v1v2
float fsmt_distance_point_to_segment(const fsmt_cartesian_point_t *p, 
    const fsmt_cartesian_point_t *p1, const fsmt_cartesian_point_t *p2) 
{
    float dx = p2->x - p1->x;
    float dy = p2->y - p1->y;

    if (dx == 0 && dy == 0)
    {
        // p1 and p2 are the same point
        return fsmt_euclidean_distance(p, p1);
    }

    // Project point p onto the line that contains p1p2, 
    // computing parameterized position t
    float t = ((p->x - p1->x) * dx + (p->y - p1->y) * dy) / (dx*dx + dy*dy);

    if (t < 0)
    {
        return fsmt_euclidean_distance(p, p1); // Closest to p1
    }else if (t > 1) 
    {
        return fsmt_euclidean_distance(p, p2); // Closest to p2
    }

    // Projection falls on the segment p1p2
    fsmt_cartesian_point_t projection = { 
        .x = p1->x + t*dx, 
        .y = p1->y + t*dy };
    return fsmt_euclidean_distance(p, &projection);
}

// Compute whether a test point is within a rectangle.
int fsmt_point_within_rectangle(const fsmt_cartesian_point_t *p1, 
    const fsmt_cartesian_point_t *p2, const fsmt_cartesian_point_t *p3, 
    const fsmt_cartesian_point_t *p4, const fsmt_cartesian_point_t *ptest)
{
    fsmt_cartesian_point_t v1 = {
        .x = p2->x - p1->x,
        .y = p2->y - p1->y 
    };
    fsmt_cartesian_point_t v2 = {
        .x = p4->x - p1->x,
        .y = p4->y - p1->y 
    }; 
    fsmt_cartesian_point_t vt = {
        .x = ptest->x - p1->x,
        .y = ptest->y - p1->y 
    };

    float dot_v1_v1 = fsmt_dot_product(&v1, &v1);
    float dot_v2_v2 = fsmt_dot_product(&v2, &v2);
    float dot_vp_v1 = fsmt_dot_product(&vt, &v1);
    float dot_vp_v2 = fsmt_dot_product(&vt, &v2);

    return  0 <= dot_vp_v1 && dot_vp_v1 <= dot_v1_v1 &&
        0 <= dot_vp_v2 && dot_vp_v2 <= dot_v2_v2;
}

// Compute shortest distance from point pt to any edge of the rectangle
float fsmt_distance_to_rectangle(const fsmt_cartesian_point_t *p1, 
    const fsmt_cartesian_point_t *p2, const fsmt_cartesian_point_t *p3, 
    const fsmt_cartesian_point_t *p4, const fsmt_cartesian_point_t *ptest) 
{
    float d1 = fsmt_distance_point_to_segment(ptest, p1, p2);
    float d2 = fsmt_distance_point_to_segment(ptest, p2, p3);
    float d3 = fsmt_distance_point_to_segment(ptest, p3, p4);
    float d4 = fsmt_distance_point_to_segment(ptest, p4, p1);

    // Return the minimum distance
    float min1 = fmin(d1, d2);
    float min2 = fmin(d3, d4);
    return fmin(min1, min2);
}

int fsmt_discrete_distance(float number, float reference, float delta) {
    if (delta <= 0) {
        // Delta must be positive to avoid division by zero or negative intervals
        return -1; // You could also choose to handle this differently
    }

    // Compute the absolute distance in terms of how many 'delta' intervals apart the numbers are
    float raw_distance = fabsf(number - reference);
    int discrete_steps = (int)(raw_distance / delta);

    return discrete_steps;
}

void add_transformations(const fsmt_transform_t *t1, const fsmt_transform_t *t2,
    fsmt_transform_t *t_out)
{
    t_out->x = t1->cos_yaw*t2->x - t1->sin_yaw*t2->y + t1->x;
    t_out->y = t1->sin_yaw*t2->x + t1->cos_yaw*t2->y + t1->y;
    t_out->cos_yaw = t1->cos_yaw*t2->cos_yaw - t1->sin_yaw*t2->sin_yaw;
    t_out->sin_yaw = t1->sin_yaw*t2->cos_yaw + t1->cos_yaw*t2->sin_yaw; 
}

void invert_transformation(const fsmt_transform_t *t_in, fsmt_transform_t *t_out)
{
    t_out->x = -(t_in->cos_yaw*t_in->x + t_in->sin_yaw*t_in->y); 
    t_out->y = -(-t_in->sin_yaw*t_in->x + t_in->cos_yaw*t_in->y);
    t_out->cos_yaw = t_in->cos_yaw;
    t_out->sin_yaw = -t_in->sin_yaw; 
}

void point_frame_transformation(const fsmt_transform_t *transform, 
    const fsmt_cartesian_point_t *in, fsmt_cartesian_point_t *out)
{
    float x = in->x;
    float y = in->y;
    float cos_yaw = transform->cos_yaw;
    float sin_yaw = transform->sin_yaw;

    out->x = cos_yaw*x - sin_yaw*y +  transform->x;
    out->y = sin_yaw*x + cos_yaw*y +  transform->y;
}

void fsmt_point_array_frame_transformation(const fsmt_transform_t *transform, 
    const fsmt_cartesian_point_array_t *in, fsmt_cartesian_point_array_t *out)
{
    float cos_yaw = transform->cos_yaw;
    float sin_yaw = transform->sin_yaw;

    fsmt_cartesian_point_t *in_points = in->points;
    fsmt_cartesian_point_t *out_points = out->points;
    for(size_t i=0; i < in->size; i++)
    {
        float x = in_points[i].x;
        float y = in_points[i].y;
        out_points[i].x = cos_yaw*x - sin_yaw*y +  transform->x;
        out_points[i].y = sin_yaw*x + cos_yaw*y +  transform->y;    
    }
}

int cartesian_array_to_polar_array(const fsmt_cartesian_point_array_t *cartesian, 
    fsmt_polar_point_array_t *polar)
{
    // check allocation.
    if(polar==NULL || cartesian==NULL) return -1;
    if(polar->points==NULL) return -1;
    if(cartesian->points==NULL) return -1;

    // check size.
    if(polar->max_size < cartesian->size) return -1;

    // dereference pointers.
    fsmt_polar_point_t *polar_points = polar->points;
    const fsmt_cartesian_point_t *cartesian_points = cartesian->points;

    // transform to polar coordinates.
    for(size_t i=0; i<cartesian->size; i++)
    {
        polar_points[i].radius = sqrtf(cartesian_points[i].x*cartesian_points[i].x + 
            cartesian_points[i].y*cartesian_points[i].y);
        polar_points[i].angle = atan2f(cartesian_points[i].y,
            cartesian_points[i].x);
    }
    polar->size = cartesian->size;

    return 1;
}

float fsmt_dot_product(const fsmt_cartesian_point_t *p1, 
    const fsmt_cartesian_point_t *p2)
{
    return p1->x*p2->x + p1->y*p2->y;
}

float wrap_to_pi(float angle){
    if(angle > M_PI)
    {
        return angle - 2*M_PI;
    }else if (angle < -M_PI)
    {
        return angle + 2*M_PI;
    }else{
        return angle;
    }
}
