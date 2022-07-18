#ifndef LIB_KALMAN_H
#define LIB_KALMAN_H
#include "math.h"
typedef struct{
    float lastPredval;          //上次的估计值
    float lastPredCovVal;       //上次估计协方差
    float lastRealCovVal;       //上次实际协方差
    float kg;                   //卡尔曼增益-kalman gain
}kalman_type;
extern kalman_type kalman_x;
extern kalman_type kalman_y;
extern kalman_type kalman_d[4];
//初始化卡尔曼滤波模块
void kalman_init(kalman_type *kalman);
/*
func:测量本次的输出值
val:本次的测量值
*/
float kalman_calc(kalman_type *kalman, float val);


#endif
