#include "Kalman.h"

kalman_type kalman_x;
kalman_type kalman_y;
kalman_type kalman_d[4];
void kalman_init(kalman_type *kalman, double Q_val, double R_val)
{
    kalman->p_last = 0.2;
    kalman->x_last = 0;
    kalman->kg = 0;
    kalman->Q = Q_val;
    kalman->R = R_val;
}
float kalman_calc(kalman_type *kalman, double val)
{
    double x_mid = kalman->x_last;
    double x_now;
    double p_mid, p_now;
    p_mid = kalman->p_last + kalman->Q;
    kalman->kg = p_mid / (p_mid + kalman->R);
    x_now = x_mid + kalman->kg * (val - x_mid);     //求出预估最优值
    p_now = (1 - kalman->kg) * p_mid;               //求出最优解对应的协方差
    kalman->p_last = p_now;
    kalman->x_last = x_now;
    return (float)x_now;
}