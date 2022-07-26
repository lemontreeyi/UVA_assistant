#include "patrol.h"
#include <math.h>

uint8_t cmd_buf[3];
bool path_flag[9] = {0};
int auto_next_target[2] = {0};
// float path[FLYPATH_LEN][2];
// bool path_flag[FLYPATH_LEN];
// void init_flypath()
// {
//     path[0][0] = 3.50; path[0][1] = 2.00;
//     path[1][0] = 2.80; path[1][1] = 2.00;
//     path[2][0] = 2.20; path[2][1] = 2.00;
//     path[3][0] = 1.50; path[3][1] = 2.00;

//     for (int i = 0; i < FLYPATH_LEN; ++i)
//     {
//         path_flag[i] = false;
//     }
    
// }

void reset_path_flag()
{
    for(int i = 0; i < 9; ++i)
        path_flag[i] = 0;
}

bool set_NextLocation(float* current_location, int* target_location, int *next_location)
{
    //转化到m为单位
    float tar_x = (float)target_location[0] / 100.0;
    float tar_y = (float)target_location[1] / 100.0;

    float flag_y = (tar_y - current_location[1] > 0)?1.0:-1.0;
    float flag_x = (tar_x - current_location[0] > 0)?1.0:-1.0;

    if((tar_x - current_location[0]) * (tar_x - current_location[0]) + (tar_y - current_location[1]) * (tar_y - current_location[1]) < 0.20*0.20)
    {

        next_location[0] = target_location[0];
        next_location[1] = target_location[1];
        return true;
    }
    else
    {
        float tan = (tar_y - current_location[1]) / (tar_x - current_location[0]);
        float sin = sqrt(tan * tan / (1 + tan * tan));
        float cos = sqrt(1 / (1 + tan * tan));
				//printf("tan:%f sin:%f cos:%f flag_x:%f flag_y:%f\r\n", tan, sin, cos, flag_x, flag_y);
        //转化到cm为单位
        //next_y
        next_location[1] = (int)((current_location[1] + 0.35 * flag_y * sin) * 100);
        //next_x
        next_location[0] = (int)((current_location[0] + 0.35 * flag_x * cos) * 100);
				//printf("%f %d\r\n", (current_location[1] + 0.35 * flag_y * sin) * 100, (int)((current_location[1] + 0.35 * flag_y * sin) * 100));
        return false;
    }
}

/*
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
    //到达目标点阈值范围内，flag置1
    if((path[have_arrive][0] - cur_x) * (path[have_arrive][0] - cur_x) + (path[have_arrive][1] - cur_y) * (path[have_arrive][1] - cur_y) < (Threshold*Threshold))
    {
        if(!is_time)
        {
            is_time = 1;
            time = HAL_GetTick();
        }
        else
        {
            if((HAL_GetTick() - time) > 2000)
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
    index = getCurrentTarget(current_location,next_target,9,path_flag,path,20);
    // printf("cur_x:%f cur_y:%f\r\n", current_location[0], current_location[1]);
    // printf("index:%d\r\n", index);
    printf("target_x:%d target_y:%d\r\n", next_target[0], next_target[1]);
    set_NextLocation(current_location, next_target, auto_next_target);
    printf("next_x:%d, next_y:%d\r\n", auto_next_target[0], auto_next_target[1]);
    Loiter_location((int)(current_location[0]*100),(int)(current_location[1]*100),auto_next_target[0],auto_next_target[1]);
    printf("roll_out:%d, pitch_out:%d\r\n",pwm_roll_out, pwm_pitch_out);
    if(index == 9) return true;
    else return false;
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
    static int target_location[2];

    if(!(*is_settarget))
    {
        target_location[0] = (int)(current_location[0]*100);
        target_location[1] = (int)(current_location[1]*100);
        *is_settarget = 1;
    }

    Loiter_location((int)(current_location[0]*100), (int)(current_location[1]*100), target_location[0], target_location[1]);
    //printf("height = %d\r\n", height);
    if(abs(height - 1500) > 100 && *is_takeoff==1)
    {
        Take_off(1500, height);
        takeoff_Time = HAL_GetTick();
    }
    else if(fabs(height - 1500) <= 100 && *is_takeoff==1)
    {
        *is_takeoff = ((HAL_GetTick() - takeoff_Time) > 1000)?0:1;
        if(*is_takeoff == 0)
        {
            Set_PWM_Thr(4500);
            return true;
        }
    }
    return false;
}

//降落
bool landon(int height, float* current_location, bool *is_settarget)
{
    static float target_location[2];

    if(!(*is_settarget))
    {
        target_location[0] = current_location[0];
        target_location[1] = current_location[1];
        *is_settarget = 1;
    }

    Loiter_location(current_location[0], current_location[1], target_location[0], target_location[1]);

    land(height);
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