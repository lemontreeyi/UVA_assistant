#ifndef __Matirx_H
#define __Matirx_H

#include <stdint.h>
#include <stdbool.h>

//需注意后面计算的A不能为奇异矩阵!!!
#define STATION1_X 0
#define STATION1_Y 0
#define STATION1_Z 85
#define STATION2_X 490
#define STATION2_Y 2
#define STATION2_Z 120
#define STATION3_X 0
#define STATION3_Y 410
#define STATION3_Z 100
#define STATION4_X 490
#define STATION4_Y 410
#define STATION4_Z 125

typedef struct Matrix
{
    float** matrix;
    int line;
    int cols;
} Matrix;

static Matrix A, A_inv;

void calculate_location(float d[], float location[]);
void calculate_cxof(float location[], short d_location[]);
void mid_filter(float raw_data, float* location_esm, float* array);
void limit_filter(float data, float* location_esm);
void init_A_matrix(void);

void BubbleSort(float* array, int len);

#endif
