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

#define FIFO_LEN 8

typedef struct Matrix
{
    float** matrix;
    int line;
    int cols;
} Matrix;

typedef struct fifo
{
  uint8_t buf[FIFO_LEN];
  int front, rear;
} fifo;

void init_fifo(fifo* p);
bool is_empty_fifo(fifo* p);
bool is_full_fifo(fifo* p);
bool read_fifo(fifo* p, uint8_t* data);
bool write_fifo(fifo* p, uint8_t data);

void calculate_location(float d[], float location[]);
void calculate_cxof(float location[], short d_location[]);
void mid_filter(float raw_data, float* location_esm, float* array);

void BubbleSort(float* array, int len);

#endif
