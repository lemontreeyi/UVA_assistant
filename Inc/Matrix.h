#ifndef __Matirx_H
#define __Matirx_H

#include <stdint.h>
#include <stdbool.h>

//需注意后面计算的A不能为奇异矩阵!!!
#define STATION1_X 0.0
#define STATION1_Y 0.0
#define STATION1_Z 1.50
#define STATION2_X 4.95
#define STATION2_Y 0.0
#define STATION2_Z 1.30
#define STATION3_X 0.0
#define STATION3_Y 4.95
#define STATION3_Z 1.60
#define STATION4_X 4.95
#define STATION4_Y 4.95
#define STATION4_Z 1.47

typedef struct Matrix
{
    float** matrix;
    int line;
    int cols;
} Matrix;

static Matrix A_res;

void calculate_location(float d[], float location[], float height);
void calculate_cxof(float location[], short d_location[], float *speed, uint32_t Dtime);
void mid_filter(float raw_data, float* location_esm, float* array);
void limit_filter(float data, float* location_esm);
void init_A_matrix(void);

void BubbleSort(float* array, int len);
bool set_NextLocation(float* current_location, int* target_location, int *next_location);
#endif
