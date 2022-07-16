#ifndef __Matirx_H
#define __Matirx_H

#include <stdint.h>

//需注意后面计算的A不能为奇异矩阵!!!
#define STATION1_X 0
#define STATION1_Y 0
#define STATION1_Z 0
#define STATION2_X 200
#define STATION2_Y 0
#define STATION2_Z 50
#define STATION3_X 0
#define STATION3_Y 100
#define STATION3_Z 60
#define STATION4_X 200
#define STATION4_Y 100
#define STATION4_Z 100


typedef struct Matrix
{
    float** matrix;
    int line;
    int cols;
} Matrix;

void calculate_location(float d[], float location[]);
void calculate_cxof(float location[], short d_location[]);

#endif
