#include <stdio.h>
#include <lapacke.h>
#include <math.h>

#include <fsmt/score.h>

#include <fsmt/tube.h>

#include <fsmt/utils.h>


int fsmt_availability(const fsmt_sensor_point_array_t* sensor, const fsmt_lidar_t *lidar){
    for(int i=0; i<sensor->size; i++){
        double measurement = lidar->measurements.ranges[sensor->points[i].index];
        // Check if measurements is valid (within bounds).
        if(measurement > lidar->config.range.min && measurement < lidar->config.range.max){
            // Check if there is free space along beam tangent_at_p_inner_final.
            if (measurement < sensor->points[i].range1){
                // Space is not free along beam, search can stop.
                return 0;
            }
        }
    }
    return 1;
}

int fsmt_availability_counter(const fsmt_sensor_point_array_t* sensor, const fsmt_lidar_t *lidar){
    int res = 0;
    for(int i=0; i<sensor->size; i++){
        double measurement = lidar->measurements.ranges[sensor->points[i].index];
        // Check if measurements is valid (within bounds).
        if(measurement > lidar->config.range.min && measurement < lidar->config.range.max){
            // Check if there is free space along beam tangent_at_p_inner_final.
            if (measurement < sensor->points[i].range1){
                // Space is not free along beam, search can stop.
                break;
            }
        }
        res++;
    }
    return res;
}

int fsmt_waypoints_within_tube(fsmt_tube_t* tube, fsmt_polar_point_array_t *plan)
{
    fsmt_polar_tube_t *polar_tube = &tube->polar;

    fsmt_polar_point_t *points = plan->points;
    size_t number_of_points = 0;

    float distance_travelled = 0;
    for(size_t i=0; i<plan->size; i++)
    {
        if(distance_travelled > polar_tube->limits.length) break;

        if(points[i].radius < polar_tube->radius.inner) continue;
        if(points[i].radius > polar_tube->radius.outer) continue;       
        if(points[i].angle < polar_tube->limits.angle.min) continue;
        if(points[i].angle > polar_tube->limits.angle.max) continue;

        number_of_points++;
    }
    return 1;
}

float fsmt_distance_to_local_goal(const fsmt_cartesian_tube_t *tube, fsmt_cartesian_point_t *test)
{
    fsmt_cartesian_point_t p1_front_right = tube->at_final_time.p1_front_right;
    fsmt_cartesian_point_t p2_front_left = tube->at_final_time.p2_front_left;
    fsmt_cartesian_point_t p3_rear_left = tube->at_final_time.p3_rear_left;
    fsmt_cartesian_point_t p4_rear_right = tube->at_final_time.p4_rear_right;

    int vehicle_reaches_goal = fsmt_point_within_rectangle(
        &p1_front_right,
        &p2_front_left,
        &p3_rear_left,
        &p4_rear_right,
        test
    );

    float distance_to_local_goal = fsmt_distance_to_rectangle(
        &p1_front_right,
        &p2_front_left,
        &p3_rear_left,
        &p4_rear_right,
        test       
    );

    if (vehicle_reaches_goal == 0)
        return distance_to_local_goal;
    else
        return -distance_to_local_goal;
}

float fsmt_orientation_to_local_goal(const fsmt_polar_tube_t *tube, float desired_final_orientation)
{
    float final_orientation = tube->limits.angle.final;
    printf("final orientation: %f.. desired orientation: %f\n",
        final_orientation*180/3.1415,
       desired_final_orientation*180/3.1415);
    float angle_difference = fabs(
        wrap_to_pi(desired_final_orientation - final_orientation));
    
    return angle_difference;
}

int fsmt_whisker_distance(const fsmt_sensor_tube_t *tube, const fsmt_lidar_t *lidar)
{
    int inner_whisker_counter = fsmt_availability_counter(tube->samples.whiskers.inner, lidar);
    int outer_whisker_counter = fsmt_availability_counter(tube->samples.whiskers.inner, lidar);

    int max_side_distance = outer_whisker_counter;
    if(outer_whisker_counter > inner_whisker_counter)
        max_side_distance = inner_whisker_counter;
}

int fsmt_circle_fitting_kasa(fsmt_cartesian_point_array_t* array, fsmt_circle_t *circle) {
    float A[200];
    float b[100];
    // Create A (n x 2) and b (n x 1)
    float A11 = 0.0, A12 = 0.0, A22 = 0.0;
    float b1 = 0.0, b2 = 0.0;

    size_t number_of_points = array->size;

    for (size_t i=0; i<array->size; i++) {
        float xi = array->points[i].x;
        float yi = array->points[i].y;
   
        float ri2 = xi*xi + yi*yi;

        A11 += yi * yi;
        A12 += yi;
        A22 += 1.0;

        b1 += -ri2 * yi;
        b2 += -ri2;
    }

    // Solve the 2x2 system [A] * [D; F] = [b]
    double det = A11 * A22 - A12 * A12;

    if (fabs(det) < 1e-12) {
        printf("Singular matrix, can't solve.\n");
        circle->radius = 100000000000000;
        return -1;
    }

    float D = (b1 * A22 - b2 * A12) / det;
    float F = (A11 * b2 - A12 * b1) / det;

    float yc = -D / 2.0;
    float r = sqrt(yc * yc - F);

    circle->xc = 0;
    circle->yc = yc;
    if(yc < 0){
        circle->radius = -r;        
    }else{
        circle->radius = r;
    }
    printf("radius: %f\n", circle->radius);
    return 1;
    // size_t number_of_points = array->size;
    
    // for (size_t i=0; i<array->size; i++) {
    //     float xi = array->points[i].x;
    //     float yi = array->points[i].y;
    //     A[i*2+0] = yi;
    //     A[i*2+1] = 1.0; //@TODO set this once at configuration.
    //     b[i] = -(xi*xi + yi*yi);
    //     printf("point %f, %f\n", xi, yi);
    // }

    // int m = number_of_points;
    // int ncols = 2;
    // int nrhs = 1;
    // int lda = 2;
    // int ldb = number_of_points > 2 ? number_of_points : 2;
    // float s[2];
    // int rank;
    // float rcond = -1;
    // int info;

    // info = LAPACKE_sgelsd(LAPACK_ROW_MAJOR, m, ncols, nrhs,
    //                       A, lda, b, ldb, s, rcond, &rank);

    // if (info != 0) {
    //     fprintf(stderr, "LAPACKE_sgelsd failed with error %d\n", info);
    //     return;
    // }

    // float center_x = 0.0;
    // float center_y = -b[0] / 2.0;
    // float radius = sqrtf(center_x * center_x + center_y * center_y - b[1]);

    // circle->xc = center_x;
    // circle->yc = center_y;
    // circle->radius = radius;
    // printf("Fitted circle with D = 0:\n");
    // printf("Center: (%.5f, %.5f)\n", center_x, center_y);
    // printf("Radius: %.5f\n", radius);

}

// #define N 8  // number of points
// #define P 2  // number of parameters (D, F)

// int main() {
//     float x[N] = {0.071352, 0.173863, 0.275245, 0.356040, 0.456806, 0.537333, 0.637925, 0.738477};
//     float y[N] = {0.092179, 0.163696, 0.236814, 0.295739, 0.369705, 0.428997, 0.503201, 0.577457};

//     float A[N * P];   // Column-major: A = [y_i, 1]
//     float b[N];       // b = -(x^2 + y^2)

//     for (int i = 0; i < N; ++i) {
//         float r2 = x[i]*x[i] + y[i]*y[i];
//         A[i]     = y[i];  // first column (y)
//         A[i + N] = 1.0f;  // second column (1)
//         b[i] = -r2;
//     }

//     lapack_int m = N;
//     lapack_int n = P;
//     lapack_int nrhs = 1;
//     lapack_int lda = N;
//     lapack_int ldb = N;
//     float s[P];
//     float rcond = -1.0;
//     lapack_int rank;
//     lapack_int info;

//     // LAPACKE_sgelsd overwrites b with solution
//     info = LAPACKE_sgelsd(LAPACK_COL_MAJOR, m, n, nrhs, A, lda, b, ldb, s, rcond, &rank);

//     if (info != 0) {
//         printf("LAPACKE_sgelsd failed with error %d\n", info);
//         return 1;
//     }

//     float D = b[0];
//     float F = b[1];

//     float yc = -D / 2.0f;
//     float r = sqrtf(yc * yc - F);

//     printf("Best-fit circle (x-center = 0) using LAPACKE_sgelsd:\n");
//     printf("  Center: (0.000, %.6f)\n", yc);
//     printf("  Radius: %.6f\n", r);
//     return 0;
// }