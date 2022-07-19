#ifndef LIB_KALMAN_H
#define LIB_KALMAN_H
#include "math.h"
typedef struct{
    double p_last;               //上次预估协方差
    double x_last;               //上次预测结果
    double kg;                   //卡尔曼增益-kalman gain
    double Q;                    //过程噪声协方差
    double R;                    //测量噪声协方差
}kalman_type;
extern kalman_type kalman_x;
extern kalman_type kalman_y;
extern kalman_type kalman_d[4];
//初始化卡尔曼滤波模块
void kalman_init(kalman_type *kalman, double Q_val, double R_val);
/*
func:测量本次的输出值
val:本次的测量值
*/
float kalman_calc(kalman_type *kalman, double val);


#endif
