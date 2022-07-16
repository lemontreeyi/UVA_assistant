#ifndef __Matirx_H
#define __Matirx_H

#include <stdint.h>

#define STATION1_X 0
#define STATION1_Y 0
#define STATION2_X 200
#define STATION2_Y 0
#define STATION3_X 0
#define STATION3_Y 100
#define STATION4_X 200
#define STATION4_Y 100


typedef struct Matrix
{
    float** matrix;
    int line;
    int cols;
} Matrix;

void calculate_location(float d[], float location[]);
void calculate_cxof(float location[], uint16_t d_location[]);

#endif
