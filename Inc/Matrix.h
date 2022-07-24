#ifndef __Matirx_H
#define __Matirx_H

#include <stdint.h>
#include <stdbool.h>

//需注意后面计算的A不能为奇异矩阵!!!
#define STATION1_X 0
#define STATION1_Y 0.03
#define STATION1_Z 0.90
#define STATION2_X 4.85
#define STATION2_Y 0.02
#define STATION2_Z 1.55
#define STATION3_X 0.04
#define STATION3_Y 4.10
#define STATION3_Z 0.70
#define STATION4_X 4.90
#define STATION4_Y 4.10
#define STATION4_Z 1.45

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
