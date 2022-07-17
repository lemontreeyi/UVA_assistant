#include "Kalman.h"

kalman_type kalman_x;
kalman_type kalman_y;
void kalman_init(kalman_type *kalman)
{
    kalman->lastPredval = 0;        //清零初始预估值
    kalman->lastPredCovVal = 0.1;
    kalman->lastRealCovVal = 0.1;
    kalman->kg = 0;
}
float kalman_calc(kalman_type *kalman, float val)
{
    float currPredVal = 0;              //本次预估值
    float currRealVal = val;            //本次测量值
    float currPredCovVal = kalman->lastPredCovVal;      //本次预估协方差值
    float currRealCovVal = kalman->lastRealCovVal;      //本次测量协方差值

    //计算本次预估值 & 更新到上次预估值
    currPredVal = kalman->lastPredval + kalman->kg * (currRealVal - kalman->lastPredval);
    kalman->lastPredval = currPredVal;

    //计算卡尔曼增益
    kalman->kg = sqrt(pow(kalman->lastPredCovVal,2) / (pow(kalman->lastPredCovVal,2) + pow(kalman->lastRealCovVal,2)));

    //计算下次预估协方差 & 测量
    kalman->lastPredCovVal = sqrt(1.0 - kalman->kg) * currPredCovVal;
    kalman->lastRealCovVal = sqrt(1.0 - kalman->kg) * currRealCovVal;

    return currPredVal;
}