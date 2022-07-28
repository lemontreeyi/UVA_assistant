#include "patrol.h"
#include <math.h>

uint8_t cmd_buf[3];
bool rec_path_flag[9] = {0};
bool t1_path_flag[4] = {0};
bool ld_path_flag[2] = {0};
int auto_next_target[2] = {0};
float init_yaw = 0;
int takeoff_location[2] = {0};

//用于重置路径点标志位
void reset_path_flag(bool path_flag[], int len)
{
    for(int i = 0; i < len; ++i)
        path_flag[i] = 0;
}

//用于自动更新下一个目标点，更新半径为35cm
bool set_NextLocation(float* current_location, int* target_location, int *next_location)
{
    //转化到m为单位
    float tar_x = (float)target_location[0] / 100.0;
    float tar_y = (float)target_location[1] / 100.0;

    float flag_y = (tar_y - current_location[1] > 0)?1.0:-1.0;
    float flag_x = (tar_x - current_location[0] > 0)?1.0:-1.0;

    if((current_location[0] - tar_x) * (current_location[0] - tar_x) + (current_location[1] - tar_y) * (current_location[1] - tar_y) < 0.25 * 0.25)
    {
        //在阈值内
        next_location[0] = current_location[0];
        next_location[1] = current_location[1];
        return true;
    }
    else
    {
        float tan = (tar_y - current_location[1]) / (tar_x - current_location[0]);
        float sin = sqrt(tan * tan / (1 + tan * tan));
        float cos = sqrt(1 / (1 + tan * tan));

        next_location[1] = (int)((current_location[1] + 0.35 * flag_y * sin) * 100);
        next_location[0] = (int)((current_location[0] + 0.35 * flag_x * cos) * 100);
        return false;
    }
}

/*
func:用于更新规划好路径的目标点
param:
Length:路径点的个数
Threshold:到目标点的更新阈值（cm为单位）
return:遍历过点的个数
*/
int getCurrentTarget(float* current_location, int* target_location, int Length, bool* path_flag, int path[][2], float Threshold)
{
    int have_arrive = 0;
    //转到cm为单位
    int cur_x = (int)(current_location[0]*100);
    int cur_y = (int)(current_location[1]*100);

    static uint32_t time = 0;
    static bool is_time = 0;
    //根据目标点顺序遍历flag
    for(have_arrive = 0; have_arrive < Length; ++have_arrive)
    {
        if(!path_flag[have_arrive])
            break;
    }
    //到达目标点阈值范围内，并且长达5秒,flag置1
    if((path[have_arrive][0] - cur_x) * (path[have_arrive][0] - cur_x) + (path[have_arrive][1] - cur_y) * (path[have_arrive][1] - cur_y) < (Threshold*Threshold))
    {
        if(!is_time)
        {
            is_time = 1;
            time = HAL_GetTick();
        }
        else
        {
            if((HAL_GetTick() - time) > 5000)
            {
                is_time = 0;
                path_flag[have_arrive] = true;
                BEEP_ON();
                HAL_Delay(100);
                BEEP_OFF();
                ++have_arrive;
            }
        } 
    }
    else
    {
        if(is_time)
        {
            is_time = 0;
        }
    }
    //设置目标点->根据flag打开的个数去设置
    if(have_arrive < Length)
    {
        target_location[0] = (int)(path[have_arrive][0]);
        target_location[1] = (int)(path[have_arrive][1]);
    }
    else
    {
        target_location[0] = (int)(path[Length - 1][0]);
        target_location[1] = (int)(path[Length - 1][1]);
    }
    return have_arrive;
}

int Get_WeightedValue(int param1, int param2, float weight)
{
    return (int)(param1 * weight + param2 * (1 - weight));
}

/*
func:完成任务一
参数：当前坐标，两个目标点坐标，任务开始初始点设置标志位
返回：1：已完成任务
*/
bool taskOne(float* cur_location, int tar1_x, int tar1_y, int tar2_x, int tar2_y, bool* is_SetStartPoint)
{
    static int start_location[2] = {0};
    int path[4][2];
    int index = 0, next_target[2];

    //设置任务的起始点
    if(!(*is_SetStartPoint))
    {
        start_location[0] = cur_location[0] * 100;
        start_location[1] = cur_location[1] * 100;
        *is_SetStartPoint = 1;
    }
    //设置任务的路径点
    path[0][0] = tar1_x; path[0][1] = start_location[1];
    path[1][0] = tar1_x; path[1][1] = tar1_y;
    path[2][0] = tar2_x; path[2][1] = tar1_y;
    path[3][0] = tar2_x; path[3][1] = tar2_y;
    printf("t1_x:%d t1_y:%d\r\n", path[0][0], path[0][1]);
    //得到当前路径中的目标点
    index = getCurrentTarget(cur_location, next_target, 4, t1_path_flag, path, 25);
    printf("target_x:%d, target_y:%d, index:%d\r\n", next_target[0], next_target[1], index);
    //得到以当前路径中目标点计算出的小目标点
    set_NextLocation(cur_location, next_target, auto_next_target);
    printf("mini_target_x:%d. mini_target_y:%d\r\n", auto_next_target[0], auto_next_target[1]);

    //计算UWB的PID数据
    Loiter_location((int)(cur_location[0] * 100), (int)(cur_location[1] * 100), auto_next_target[0], auto_next_target[1]);
    //混合UWB的PID和视觉定位PID,并输出
    Mix_PwmOut((int)(cur_location[0] * 100), (int)(cur_location[1] * 100),next_target);

    if(index == 4) return true;
    else return false;
}

/*
func:让无人机飞一个矩形路径,默认将起始点设在左下角，可按需求调整
param：width_x->x方向的边长，width_y->y方向的边长，start->起始点
6     5   4
7         3
0(8)  1   2
*/
bool Rectangle(int *start, int width_x, int width_y, float *current_location)
{
    int path[9][2];
    //printf("start_x:%d start_y:%d\r\n", start[0], start[1]);
    path[0][0] = start[0]; path[0][1] = start[1];
    path[1][0] = start[0] - (int)(width_x / 2.0); path[1][1] = start[1];
    path[2][0] = start[0] - width_x; path[2][1] = start[1];
    path[3][0] = path[2][0]; path[3][1] = start[1] - (int)(width_y / 2.0);
    path[4][0] = path[2][0]; path[4][1] = start[1] - width_y;
    path[5][0] = path[1][0]; path[5][1] = path[4][1];
    path[6][0] = start[0];   path[6][1] = path[4][1];
    path[7][0] = start[0];   path[7][1] = path[3][1];
    path[8][0] = start[0];   path[8][1] = start[1];
    int next_target[2], index = 0;
    // printf("path0_x:%d path0_y:%d\r\n", path[0][0], path[0][1]);
    index = getCurrentTarget(current_location,next_target,9,rec_path_flag,path,20);
    // printf("cur_x:%f cur_y:%f\r\n", current_location[0], current_location[1]);
    // printf("index:%d\r\n", index);
    // printf("target_x:%d target_y:%d\r\n", next_target[0], next_target[1]);
    set_NextLocation(current_location, next_target, auto_next_target);
    Loiter_location((int)(current_location[0]*100),(int)(current_location[1]*100),auto_next_target[0],auto_next_target[1]);
    if(index == 9) return true;
    else return false;
}

//朝目标点飞去
bool Fly2Target(float *current_location,int *target_location)
{
    static int next_target[2] = {0, 0};
    int cur_x = (int)(current_location[0]*100), cur_y = (int)(current_location[1]*100);
    set_NextLocation(current_location, target_location, next_target);
    printf("next_x:%d, next_y:%d\r\n", next_target[0], next_target[1]);
    Loiter_location((int)(current_location[0]*100),(int)(current_location[1]*100),next_target[0],next_target[1]);
    Mix_PwmOut(cur_x, cur_y, target_location);
    return true;
}

//根据坐标位置融合PWM输出
void Mix_PwmOut(int cur_x, int cur_y, int *target_location)
{
    if(((cur_x-target_location[0])*(cur_x-target_location[0]) + (cur_y-target_location[1])*(cur_y-target_location[1])) <= 85*85)
    {
        Loiter(Attitude.Position_x, Attitude.Position_y, Attitude.SetPoint_x, Attitude.SetPoint_y,0,0);
        //此处让坐标环PID占0.2的权重
        // printf("1pwm_roll_out:%d pwm_pitch_out:%d pwm_roll_SensorOut:%d pwm_pitch_SensorOut:%d\r\n", pwm_roll_out, pwm_pitch_out, pwm_roll_SensorOut, pwm_pitch_SensorOut);
        Set_PWM_Roll(Get_WeightedValue(pwm_roll_out, pwm_roll_SensorOut, 1.0));
        Set_PWM_Pitch(Get_WeightedValue(pwm_pitch_out, pwm_pitch_SensorOut, 1.0));
    }
    else
    {
        // printf("2pwm_roll_out:%d pwm_pitch_out:%d\r\n", pwm_roll_out, pwm_pitch_out);
        Set_PWM_Roll(pwm_roll_out);
        Set_PWM_Pitch(pwm_pitch_out);   
    }
}

/*
func1:呼啦圈垂直于地面摆放，无人机固定一个高度向指定方向前行一段距离
return:
返回1：目标点为起始点
返回2：正在穿越呼啦圈，将指针指向的值更新为线段上的目标点
返回3：目标点为结束位置
*/
int Get_circle_1(float *current_location, int *target_location, int *begin, int *end)
{
    //设置标志位
    bool circle_flag[4] = {0};
    int index;
    //根据呼啦圈的起始点和末尾点规划路径
    int path[4][2];
    path[0][0], path[0][1] = begin[0], begin[1];        //起始点
    int dpath_x = (int)((end[0] - begin[0]) / 3);
    int dpath_y = (int)((end[1] - begin[1]) / 3);
    //在中间设置2个过程目标点
    for(int i=1;i<3;i++)
    {
        path[i][0] = path[i-1][0] + dpath_x;
        path[i][1] = path[i-1][1] + dpath_y;
    }
    path[3][0] = end[0];
    path[3][1] = end[1];
    //传入处理函数，得出当前目标值
    index = getCurrentTarget(current_location,target_location,4,circle_flag,path,5);
    switch (index)
    {
    case 0:
        return 1; break;
    case 3:
        return 3; break;
    default: return 2; break;
    }
}

/*
func2:呼啦圈平行于地面摆放，飞机保持xy坐标状态，变化高度，钻过呼啦圈
*/
int Get_circle_2(int current_height, int target_height)
{
    //到时通过调用一键起飞Take_off函数实现，重点->如何加入其它手段使得xy更稳
}

//起飞
bool takeoff(int height, float* current_location, bool* is_takeoff, bool* is_settarget)
{
    static uint32_t takeoff_Time = 0;
    static int next_target[2] = {0};

    if(!(*is_settarget))
    {
        takeoff_location[0] = (int)(current_location[0]*100);
        takeoff_location[1] = (int)(current_location[1]*100);
        *is_settarget = 1;
    }

    set_NextLocation(current_location, takeoff_location, next_target);
    Loiter_location((int)(current_location[0]*100), (int)(current_location[1]*100), takeoff_location[0], takeoff_location[1]);
    Mix_PwmOut((int)(current_location[0] * 100), (int)(current_location[1] * 100),takeoff_location);

    // printf("height = %d\r\n", height);
    // printf("x:%f y:%f\r\n", current_location[0], current_location[1]);
    // printf("takeoff_x:%d takeoff_y:%d\r\n", takeoff_location[0], takeoff_location[1]);
    if(abs(height - 1500) > 100 && *is_takeoff==1)
    {
        Take_off(1500, height);
        takeoff_Time = HAL_GetTick();
    }
    else if(fabs(height - 1500) <= 100 && *is_takeoff==1)
    {
        *is_takeoff = ((HAL_GetTick() - takeoff_Time) > 3000)?0:1;
        if(*is_takeoff == 0)
        {
            Set_PWM_Thr(4500);
            return true;
        }
    }
    return false;
}

//降落
bool landon(int height, float* current_location, bool *is_SetStartPoint)
{
    static int start_location[2] = {0};
    int path[2][2], index = 0;
    int next_target[2];
    //设置任务的起始点
    if(!(*is_SetStartPoint))
    {
        start_location[0] = current_location[0];
        start_location[1] = current_location[1];
        *is_SetStartPoint = 1;
    }
    index = getCurrentTarget(current_location, next_target, 2, ld_path_flag, path, 25);
    if(set_NextLocation(current_location, next_target, auto_next_target) && index == 2)
        land(height);
    Loiter_location((int)(current_location[0] * 100), (int)(current_location[1]), auto_next_target[0], auto_next_target[1]);
    
	if(height < 80) return true;
	else return false;
}

//拍照片
bool shootphoto(float target_x, float target_y, float* current_location, USART_TypeDef* huart)
{
    Loiter_location(current_location[0], current_location[1], target_x, target_y);
    if(fabs(current_location[0] - target_x) < 0.01 && fabs(current_location[1] - target_y) < 0.01)
    {
        Pack_cmd_buf(1, cmd_buf);
        BSP_USART_SendArray_LL(huart, cmd_buf, 3);
        return true;
    }
    return false;
}

//纠正姿态
bool fixyaw(float yaw)
{
    static bool is_inityaw = 0;
    if(!is_inityaw)
    {
        init_yaw = yaw;
        if(yaw != 0)
        {
            printf("init_yaw:%f\r\n", init_yaw);
            is_inityaw = 1;
        }        
    }
    if(yaw < 0 && yaw > -150.0 / 180.0 * 3.14159 && is_inityaw)
    {
        if(yaw - init_yaw < -0.175f)
        {
            printf("pwm_yaw:%f\r\n", 4431 + 100 * (exp(init_yaw - yaw) - 1));
            Set_PWM_Yaw(min(4431 + 80 * exp(init_yaw - yaw), 4631));//右转
        }         
        else if(yaw - init_yaw > 0.175f)
        {
            printf("pwm_yaw:%f\r\n", 4431 - 100 * (exp(yaw - init_yaw) - 1));
            Set_PWM_Yaw(max(4431 - 80 * exp(yaw - init_yaw), 4231));//左转
        }
        else
        {
            Set_PWM_Yaw(4431);//回中
        }    
    }
    else
        Set_PWM_Yaw(4431);//回中
    return is_inityaw;
}