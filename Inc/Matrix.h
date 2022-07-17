#ifndef __Matirx_H
#define __Matirx_H

#include <stdint.h>

//需注意后面计算的A不能为奇异矩阵!!!
#define STATION1_X 60
#define STATION1_Y 0
#define STATION1_Z 0
#define STATION2_X 0
#define STATION2_Y 500
#define STATION2_Z 35
#define STATION3_X 420
#define STATION3_Y 500
#define STATION3_Z 35
#define STATION4_X 420
#define STATION4_Y 0
#define STATION4_Z 35


typedef struct Matrix
{
    float** matrix;
    int line;
    int cols;
} Matrix;

void calculate_location(float d[], float location[]);
void calculate_cxof(float location[], short d_location[]);

#endif
