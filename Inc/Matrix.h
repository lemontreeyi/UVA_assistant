#ifndef __Matirx_H
#define __Matirx_H

#include <stdint.h>
#include <stdbool.h>

//需注意后面计算的A不能为奇异矩阵!!!
extern float STATION1_X;
extern float STATION1_Y;
extern float STATION1_Z;
extern float STATION2_X;
extern float STATION2_Y;
extern float STATION2_Z;
extern float STATION3_X;
extern float STATION3_Y;
extern float STATION3_Z;
extern float STATION4_X;
extern float STATION4_Y;
extern float STATION4_Z;

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

#endif
