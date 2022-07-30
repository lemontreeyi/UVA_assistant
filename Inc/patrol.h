#ifndef _PATROL_
#define _PATROL_

#include <stdbool.h>
#include <stdint.h>
#include "flyControl.h"
#include "uart_api.h"
#include "main.h"
#define FLYPATH_LEN 4

extern uint8_t cmd_buf[4];
extern bool t1_path_flag[4];
extern bool t1_opt_flag;

// void init_flypath();
void reset_path_flag(bool path_flag[], int len);
bool set_NextLocation(float* current_location, int* target_location, int *next_location);
int getCurrentTarget(float* current_location, int* target_location, int Length, bool* path_flag, int path[][2], float Threshold, uint16_t Task1_Type1,  uint16_t Task1_Type2);
bool Rectangle(int *start, int width_x, int width_y, float *current_location);
int Get_circle_1(float *crrent_location, int *target_location, int *begin, int *end);
int Get_circle_2(int current_height, int target_height);
bool takeoff(int height, float* current_location, bool* is_takeoff, bool* is_settarget);
bool taskOne(float* cur_location, int tar1_x, int tar1_y, int tar2_x, int tar2_y, bool* is_SetStartPoint, uint16_t Task1_Type1,  uint16_t Task1_Type2);
bool taskOne_C(float* cur_location, int height, int tar1_x, int tar1_y, int tar2_x, int tar2_y, bool* is_SetStartPoint, uint16_t Task1_Type1,  uint16_t Task1_Type2);
bool Fly2Target(float *current_location,int *target_location);
bool landon(int height, float* current_location, bool *is_SetStartPoint);
bool landon_C(int height, float* current_location, bool *is_SetStartPoint);
bool shootphoto(float target_x, float target_y, float* current_location, USART_TypeDef* huart);
int Get_WeightedValue(int param1, int param2, float weight);
void Mix_PwmOut(int cur_x, int cur_y, int *target_location, uint16_t task1_type);
bool fixyaw(float yaw);
void Moto_Down(void);
void Moto_Up(void);
void Moto_stable(void);
bool Throw_Moto(bool *is_begin, bool *is_over, bool *drop_over);
bool rotato(float cur_yaw, float tar_yaw);

#endif
