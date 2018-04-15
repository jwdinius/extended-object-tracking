#ifndef CONSTANTS_H
#define CONSTANTS_H

#define MAP_WIDTH           8000.0f
#define MAP_HEIGHT          8000.0f

#define MSECS_UDP           1000
#define MAX_DETS            64
#define P_LAMBDA            5

#define OBJECT_W            170.0f
#define OBJECT_H             40.0f

// initial covariance terms
#define INIT_VAR_LENGTH          200.0f
#define INIT_VAR_ALPHA            0.02f
#define INIT_VAR_POS             900.0f
#define INIT_VAR_VEL              16.0f

// process covariance terms
#define VAR_LENGTH          0.5f
#define VAR_ALPHA           0.04f
#define VAR_POS             100.0f
#define VAR_VEL             1.0f

// measurement covariance terms
#define VAR_H               0.25f
#define VAR_V1              2000.0f
#define VAR_V2              80.0f
#endif  // CONSTANTS_H
