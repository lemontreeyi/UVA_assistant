#ifndef _PATROL_
#define _PATROL_

#include <stdbool.h>
#include <stdint.h>
#include "flyControl.h"
#include "main.h"
#define FLYPATH_LEN 4

// void init_flypath();
void reset_path_flag();
bool set_NextLocation(float* current_location, int* target_location, int *next_location);
int getCurrentTarget(float* current_location, int* target_location, int Length, bool* path_flag, int path[][2], float Threshold);
bool Rectangle(int *start, int width_x, int width_y, float *current_location);
int Get_circle_1(float *current_location, int *target_location, int *begin, int *end);
int Get_circle_2(int current_height, int target_height);
bool takeoff(int height, float* current_location, bool* is_takeoff, bool* is_settarget);
void landon(float height, float* current_location);
bool shootphoto(float target_x, float target_y, float* current_location, USART_TypeDef* huart);
#endif