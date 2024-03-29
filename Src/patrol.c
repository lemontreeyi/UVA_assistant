#include "patrol.h"
#include "main.h"
#include <math.h>
#include <stdlib.h>

uint8_t cmd_buf[4];
bool rec_path_flag[9] = {0};
bool t1_path_flag[4] = {0};
bool t1_opt_flag = 0;
bool ld_path_flag[2] = {0};
int auto_next_target[2] = {0};
float init_yaw = 0;
int takeoff_location[2] = {0};

bool is_near_target[4] = {0};
bool is_near_target_land[2] = {0};

bool drop_goods_flag[6] = {0};//分别代表下降，舵机下降货物，稳住5s，舵机上升，上升，前往下一个坐标六个步骤
bool is_start_drop_goods = 0;
uint32_t drop_time = 0;
uint32_t stable_time = 0;
bool is_count_time = 0;
bool is_begin = 0, is_over = 0, send_over = 0;

bool taskone_state = 0;
int count_time = 0;
bool time_over = 0;
bool start_time = 0;
int time_array[5][5];

//用于重置路径点标志位
void reset_path_flag(bool path_flag[], int len)
{
    for(int i = 0; i < len; ++i)
        path_flag[i] = 0;
}
//清除投放货物过程的标志位
void clear_drop_goods_flag()
{
    for(int i = 0; i < 6; ++i)
        drop_goods_flag[i] = 0;
    is_begin = 0;
    is_over = 0;
    send_over = 0;
    is_count_time = 0; 
}

//用于自动更新下一个目标点，更新半径为35cm
bool set_NextLocation(float* current_location, int* target_location, int *next_location)
{
    //转化到m为单位
    float tar_x = (float)target_location[0] / 100.0f;
    float tar_y = (float)target_location[1] / 100.0f;

    float flag_y = (tar_y - current_location[1] > 0)?1.0:-1.0;
    float flag_x = (tar_x - current_location[0] > 0)?1.0:-1.0;

    if((current_location[0] - tar_x) * (current_location[0] - tar_x) + (current_location[1] - tar_y) * (current_location[1] - tar_y) < 0.25 * 0.25)
    {
        //在阈值内
        next_location[0] = target_location[0];
        next_location[1] = target_location[1];
        return true;
    }
    else
    {
        float tan = (tar_y - current_location[1]) / (tar_x - current_location[0]);
        float sin = sqrt(tan * tan / (1 + tan * tan));
        float cos = sqrt(1 / (1 + tan * tan));

        next_location[1] = (int)((current_location[1] + 0.35f * flag_y * sin) * 100.0f);
        next_location[0] = (int)((current_location[0] + 0.35f * flag_x * cos) * 100.0f);
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
int getCurrentTarget(float* current_location, int* target_location, int Length, bool* path_flag, int path[][2], float Threshold, uint16_t Task1_Type1,  uint16_t Task1_Type2)
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
        if(have_arrive == 1 || have_arrive == 3)
        {
            if(drop_goods_flag[0] == 0 && !is_start_drop_goods)
            {
                clear_drop_goods_flag();
                drop_goods_flag[0] = 1;//开启下降
                is_start_drop_goods = 1;
            }
            if(drop_goods_flag[5] == 1)
            {
                if(have_arrive == 1 && t1_opt_flag)
                {
                    Pack_cmd_buf(Task1_Type1, 0, cmd_buf);
                    BSP_USART_SendArray_LL(USART2, cmd_buf, 4);
                    t1_opt_flag = 0;
                }
                if(have_arrive == 3 && t1_opt_flag)
                {
                    Pack_cmd_buf(Task1_Type2, 0, cmd_buf);
                    BSP_USART_SendArray_LL(USART2, cmd_buf, 4);
                    t1_opt_flag = 0;
                }
                clear_drop_goods_flag();//
                is_start_drop_goods = 0;
                path_flag[have_arrive] = true;
                ++have_arrive;
            }
        }
        else
        {
            path_flag[have_arrive] = true;
            BEEP_ON();
            HAL_Delay(100);
            BEEP_OFF();
            ++have_arrive;
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
bool taskOne(float* cur_location, int tar1_x, int tar1_y, int tar2_x, int tar2_y, bool* is_SetStartPoint, uint16_t Task1_Type1,  uint16_t Task1_Type2)
{
    static int start_location[2] = {0};
    int path[4][2];
    int next_target[2];
    static int index = 0, old_index = 0;
    //设置任务的起始点
    if(!(*is_SetStartPoint))
    {
        start_location[0] = cur_location[0] * 100;
        start_location[1] = cur_location[1] * 100;
        *is_SetStartPoint = 1;
        //请求识别type4,即红色三角形
        // Pack_cmd_buf(Task1_Type1,1,cmd_buf);
        // BSP_USART_SendArray_LL(USART2, cmd_buf, 4);
    }
    //设置任务的路径点
    path[0][0] = tar1_x; path[0][1] = start_location[1];
    path[1][0] = tar1_x; path[1][1] = tar1_y;       //目标点1
    path[2][0] = tar2_x; path[2][1] = tar1_y;
    path[3][0] = tar2_x; path[3][1] = tar2_y;       //目标点2
    printf("t1_x:%d t1_y:%d\r\n", path[0][0], path[0][1]);
    old_index = index;
    //得到当前路径中的目标点
    index = getCurrentTarget(cur_location, next_target, 4, t1_path_flag, path, 25, Task1_Type1, Task1_Type2);
    // if(index ==2 && old_index != index)
    // {
    //     //请求识别type3,即蓝色三角形
    //     Pack_cmd_buf(Task1_Type2,1,cmd_buf);
    //     BSP_USART_SendArray_LL(USART2, cmd_buf, 4);
    // }
    printf("target_x:%d, target_y:%d, index:%d\r\n", next_target[0], next_target[1], index);
    //得到以当前路径中目标点计算出的小目标点
    // set_NextLocation(cur_location, next_target, auto_next_target);
    // printf("mini_target_x:%d. mini_target_y:%d\r\n", auto_next_target[0], auto_next_target[1]);

    //计算UWB的PID数据
    Loiter_location((int)(cur_location[0] * 100), (int)(cur_location[1] * 100), auto_next_target[0], auto_next_target[1]);
    //混合UWB的PID和视觉定位PID,并输出
    if(index == 1)
        Mix_PwmOut((int)(cur_location[0] * 100), (int)(cur_location[1] * 100), next_target, Task1_Type1);
    else if(index == 3)
        Mix_PwmOut((int)(cur_location[0] * 100), (int)(cur_location[1] * 100), next_target, Task1_Type2);
    else
        Mix_PwmOut((int)(cur_location[0] * 100), (int)(cur_location[1] * 100), next_target, 8);//无效视觉
    if(index == 4) return true;
    else return false;
}

/*
func:完成任务一
参数：当前坐标，两个目标点坐标，任务开始初始点设置标志位
返回：1：已完成任务
*/
bool taskOne_B(float* cur_location, int tar1_x, int tar1_y, int tar2_x, int tar2_y, bool* is_SetStartPoint, uint16_t Task1_Type1,  uint16_t Task1_Type2)
{
    static int start_location[2] = {0};
    int path[4][2];
    int next_target[2];
    static int index = 0, old_index = 0;
    //设置任务的起始点
    if(!(*is_SetStartPoint))
    {
        start_location[0] = cur_location[0] * 100;
        start_location[1] = cur_location[1] * 100;
        *is_SetStartPoint = 1;
        //请求识别type4,即红色三角形
        // Pack_cmd_buf(Task1_Type1,1,cmd_buf);
        // BSP_USART_SendArray_LL(USART2, cmd_buf, 4);
    }
    //设置任务的路径点
    path[0][0] = tar1_x; path[0][1] = start_location[1];
    path[1][0] = tar1_x; path[1][1] = tar1_y;       //目标点1
    path[2][0] = tar2_x; path[2][1] = tar1_y;
    path[3][0] = tar2_x; path[3][1] = tar2_y;       //目标点2
    printf("t1_x:%d t1_y:%d\r\n", path[0][0], path[0][1]);
    old_index = index;
    //得到当前路径中的目标点
    index = getCurrentTarget(cur_location, next_target, 4, t1_path_flag, path, 25, Task1_Type1, Task1_Type2);
    printf("target_x:%d, target_y:%d, index:%d\r\n", next_target[0], next_target[1], index);

    //计算UWB的PID数据
    Loiter_location((int)(cur_location[0] * 100), (int)(cur_location[1] * 100), auto_next_target[0], auto_next_target[1]);
    //混合UWB的PID和视觉定位PID,并输出
    if(index == 1)
        Mix_PwmOut((int)(cur_location[0] * 100), (int)(cur_location[1] * 100), next_target, Task1_Type1);
    else if(index == 3)
        Mix_PwmOut((int)(cur_location[0] * 100), (int)(cur_location[1] * 100), next_target, Task1_Type2);
    else
        Mix_PwmOut((int)(cur_location[0] * 100), (int)(cur_location[1] * 100), next_target, 8);//无效视觉
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
    // index = getCurrentTarget(current_location,next_target,9,rec_path_flag,path,20);
    // printf("cur_x:%f cur_y:%f\r\n", current_location[0], current_location[1]);
    // printf("index:%d\r\n", index);
    // printf("target_x:%d target_y:%d\r\n", next_target[0], next_target[1]);
    set_NextLocation(current_location, next_target, auto_next_target);
    Loiter_location((int)(current_location[0]*100),(int)(current_location[1]*100),auto_next_target[0],auto_next_target[1]);
    if(index == 9) return true;
    else return false;
}

// //朝目标点飞去
// bool Fly2Target(float *current_location,int *target_location)
// {
//     static int next_target[2];
//     next_target[0] = target_location[0]
//     int cur_x = (int)(current_location[0]*100), cur_y = (int)(current_location[1]*100);

//     // Mix_PwmOut(cur_x, cur_y, target_location);
//     return true;
// }

//根据坐标位置融合PWM输出
void Mix_PwmOut(int cur_x, int cur_y, int *target_location, uint16_t task1_type)
{
    if(((cur_x-target_location[0])*(cur_x-target_location[0]) + (cur_y-target_location[1])*(cur_y-target_location[1])) <= 40*40)
    {
        if(task1_type != 8)
        {
            if(!t1_opt_flag)
            {
                Pack_cmd_buf(task1_type, 1, cmd_buf);
                BSP_USART_SendArray_LL(USART2, cmd_buf, 4);
                Attitude.Position_x = 0;
                Attitude.Position_y = 0;
                Attitude.SetPoint_x = 0;
                Attitude.SetPoint_y = 0;
                t1_opt_flag = 1;
            }
        }
        
        Loiter(Attitude.Position_x, Attitude.Position_y, Attitude.SetPoint_x, Attitude.SetPoint_y,0,0);
        //此处让坐标环PID占0.2的权重
        printf("1pwm_roll_out:%d pwm_pitch_out:%d pwm_roll_SensorOut:%d pwm_pitch_SensorOut:%d\r\n", pwm_roll_out, pwm_pitch_out, pwm_roll_SensorOut, pwm_pitch_SensorOut);
        Set_PWM_Roll(Get_WeightedValue(pwm_roll_out, pwm_roll_SensorOut, 1.0));
        Set_PWM_Pitch(Get_WeightedValue(pwm_pitch_out, pwm_pitch_SensorOut, 1.0));
        //printf("roll_out:%d, pitch_out:%d\r\n",Get_WeightedValue(pwm_roll_out, pwm_roll_SensorOut, 0.2),Get_WeightedValue(pwm_pitch_out, pwm_pitch_SensorOut, 0.2));
    }
    else
    {
        printf("2pwm_roll_out:%d pwm_pitch_out:%d\r\n", pwm_roll_out, pwm_pitch_out);
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
    // index = getCurrentTarget(current_location,target_location,4,circle_flag,path,5);
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
    // static int next_target[2] = {0};

    if(!(*is_settarget))
    {
        takeoff_location[0] = (int)(current_location[0]*100);
        takeoff_location[1] = (int)(current_location[1]*100);
        *is_settarget = 1;
    }

    // set_NextLocation(current_location, takeoff_location, next_target);
    // Loiter_location((int)(current_location[0]*100), (int)(current_location[1]*100), takeoff_location[0], takeoff_location[1]);
    // Mix_PwmOut((int)(current_location[0] * 100), (int)(current_location[1] * 100),takeoff_location, 0);

    // printf("height = %d\r\n", height);
    // printf("x:%f y:%f\r\n", current_location[0], current_location[1]);
    // printf("takeoff_x:%d takeoff_y:%d\r\n", takeoff_location[0], takeoff_location[1]);
    // printf("mini_tar_x:%d mini_tar_y:%d\r\n", next_target[0], next_target[1]);

    if(abs(height - 1500) > 100 && *is_takeoff==1)
    {
        Take_off(1500, height);
        // if(height < 450) Set_PWM_Roll(4500 + 150);
        takeoff_Time = HAL_GetTick();
    }
    else if(height > 1400 && *is_takeoff==1)
    {
        *is_takeoff = ((HAL_GetTick() - takeoff_Time) > 3000)?0:1;
        if(*is_takeoff == 0)
        {
            // Pack_cmd_buf(0, 0, cmd_buf);
            // BSP_USART_SendArray_LL(USART2, cmd_buf, 4);
            // t1_opt_flag = 0;

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
    index = getCurrentTarget(current_location, next_target, 2, ld_path_flag, path, 25, 0, 8);
    if(set_NextLocation(current_location, next_target, auto_next_target) && index == 2)
        land(height);
    Loiter_location((int)(current_location[0] * 100), (int)(current_location[1]), auto_next_target[0], auto_next_target[1]);
    if(index == 1)
        Mix_PwmOut((int)(current_location[0] * 100), (int)(current_location[1] * 100), next_target, 0);
    else
        Mix_PwmOut((int)(current_location[0] * 100), (int)(current_location[1] * 100), next_target, 8);
	if(height < 80) return true;
	else return false;
}

//拍照片
bool shootphoto(float target_x, float target_y, float* current_location, USART_TypeDef* huart)
{
    Loiter_location(current_location[0], current_location[1], target_x, target_y);
    if(fabs(current_location[0] - target_x) < 0.01 && fabs(current_location[1] - target_y) < 0.01)
    {
        //Pack_cmd_buf(1, cmd_buf);
        BSP_USART_SendArray_LL(huart, cmd_buf, 3);
        return true;
    }
    return false;
}

bool rotato(float cur_yaw, float tar_yaw)
{
    if(abs(cur_yaw - tar_yaw) < 3.1415926 / 2.0)
    {
        if(cur_yaw - tar_yaw < -0.125f)
        {
            Set_PWM_Yaw(min((int)(4500 + 80 * exp(tar_yaw - cur_yaw)), 4500 + 200));//右转
            // printf("pwm_yaw:%d\r\n", min((int)(4500 + 120 * exp(tar_yaw - cur_yaw)), 4500 + 200));
        }
            
        else if(cur_yaw - tar_yaw > 0.125f)
        {
            Set_PWM_Yaw(max((int)(4500 - 80 * exp(cur_yaw - tar_yaw)), 4500 - 200));
            // printf("pwm_yaw:%d\r\n", max((int)(4500 - 120 * exp(cur_yaw - tar_yaw)), 4500 - 200));
        }
        else
        {
            Set_PWM_Yaw(4500);
            return true;
        }  
    }
    else
    {
        if(cur_yaw - tar_yaw < 0)
        {
            printf("pwm1_yaw:%d\r\n", max((int)(4500 - 80 * exp(tar_yaw - cur_yaw + 3.1415926)), 4500 - 200));
            Set_PWM_Yaw(max((int)(4500 - 80 * exp(tar_yaw - cur_yaw + 3.1415926)), 4500 - 200));
        }     
        else
        {
            printf("pwm1_yaw:%d\r\n", min((int)(4431 + 30 * exp(tar_yaw - cur_yaw - 3.1415926)), 4431 + 200));
            Set_PWM_Yaw(min((int)(4500 + 80 * exp(tar_yaw - cur_yaw - 3.1415926)), 4500 + 200));
        }
            
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
    if(is_inityaw)
        rotato(yaw, init_yaw);
    return is_inityaw;
}

void to_one_point(float* cur_location, int* target_location, uint16_t task1_type, bool* is_near_target, bool axis)
{
    int cur_x = (int)(cur_location[0] * 100);
    int cur_y = (int)(cur_location[1] * 100);
    int target_x = target_location[0];
    int target_y = target_location[1];

    Loiter_location(cur_x, cur_y, target_x, target_y);//必须每次丢调用这个函数，不然突然切换到位置PID会derror会产生突变

    if((cur_x * target_x) * (cur_x * target_x) + (cur_y - target_y) * (cur_y - target_y) < 30 * 30 || *is_near_target == 1)
    {
        Mix_PwmOut(cur_x, cur_y, target_location, task1_type);
        *is_near_target = 1;
    }
    else
    {
        int pwm_x = pwm_roll_out, pwm_y = pwm_pitch_out;
        if(!axis) //axis = 0,沿x轴飞
        {
            pwm_y = calculate_noPID_pwm(cur_y, target_y, 0);
        }
        else
        {
            pwm_x = calculate_noPID_pwm(cur_x, target_x, 1);
        }
        Set_PWM_Roll(pwm_x);
        Set_PWM_Pitch(pwm_y);
    }
}

int calculate_noPID_pwm(int cur_point, int tar_point, bool is_x)
{
    float d_flag = ((cur_point - tar_point > 0)? 1.0: -1.0);
    if(abs(cur_point - tar_point) < 100)
    {
        int out = (cur_point - tar_point) * 7.0 - 350;
        if(is_x)
            return 4500 + out;
        else
            return 4500 - out;
    }
    else
    {
        if(is_x)
            return 4500 + d_flag * 450;
        else
            return 4500 - d_flag * 450;
    }
}

bool taskOne_C(float* cur_location, int height, int tar1_x, int tar1_y, int tar2_x, int tar2_y, bool* is_SetStartPoint, uint16_t Task1_Type1,  uint16_t Task1_Type2)
{
    static int start_location[2] = {0};
    int path[4][2];
    int next_target[2];
    static int index = 0;
    //设置任务的起始点
    if(!(*is_SetStartPoint))
    {
        start_location[0] = cur_location[0] * 100;
        start_location[1] = cur_location[1] * 100;
        *is_SetStartPoint = 1;
    }
    //设置任务的路径点
    path[0][0] = tar1_x; path[0][1] = start_location[1];
    path[1][0] = tar1_x; path[1][1] = tar1_y;       //目标点1
    path[2][0] = tar2_x; path[2][1] = tar1_y;
    path[3][0] = tar2_x; path[3][1] = tar2_y;       //目标点2

    //得到当前路径中的目标点
    index = getCurrentTarget(cur_location, next_target, 4, t1_path_flag, path, 25, Task1_Type1, Task1_Type2);

    printf("height = %d\r\n", height);
    printf("x:%f y:%f\r\n", cur_location[0], cur_location[1]);
    printf("target_x:%d, target_y:%d, index:%d\r\n", next_target[0], next_target[1], index);
    printf("flag:%d %d %d %d %d %d\r\n", drop_goods_flag[0], drop_goods_flag[1], drop_goods_flag[2], drop_goods_flag[3], drop_goods_flag[4],drop_goods_flag[5]);
    //flag0 -> 开始降到80cm
    if(drop_goods_flag[0] == 1)
    {
        if(abs(height - 800) > 100)
        {
            Take_off(800, height);
            drop_time = HAL_GetTick();
        }
        else if(abs(height - 800) <= 100)
        {
            if(HAL_GetTick() - drop_time > 3000)
            {
                Set_PWM_Thr(4500);
                drop_goods_flag[1] = 1;
                drop_goods_flag[0] = 0;
                is_begin = 1;
            }
        }
    }
    //flag1 -> 开始投下快递，投递5s后停止下降
    else if(drop_goods_flag[1] == 1)//舵机下降
    {
        Throw_Moto(&is_begin, &is_over, &send_over);
    }
    //flag2 -> 投递完成后，计时5s，将send_over打开
    else if(drop_goods_flag[2] == 1)
    {
        if(!is_count_time)
        {
            stable_time = HAL_GetTick();
            is_count_time = 1;
        }
        if(HAL_GetTick() - stable_time > 5000)
        {
            drop_goods_flag[3] = 1;
            drop_goods_flag[2] = 0;
            is_over = 1;
            BEEP_ON();
            HAL_Delay(100);
            BEEP_OFF();
        }
    }
    //flag3 -> 直接收上来快递,收完后蜂鸣器提醒
    else if(drop_goods_flag[3] == 1)
    {
        if(Throw_Moto(&is_begin, &is_over, &send_over))
        {
            is_over = 0;
            drop_goods_flag[4] = 1;
            drop_goods_flag[3] = 0;
            BEEP_ON();
            HAL_Delay(100);
            BEEP_OFF();
            printf("get up finished...\r\n");
        }
    }
    //flag4 -> 高度升到1.5m
    else if(drop_goods_flag[4] == 1)
    {
        if(abs(height - 1500) > 100)
        {
            Take_off(1500, height);
            drop_time = HAL_GetTick();
        }
        else if(abs(height - 1500) <= 100)
        {
            if(HAL_GetTick() - drop_time > 3000)
            {
                Set_PWM_Thr(4500);
                drop_goods_flag[5] = 1;
                drop_goods_flag[4] = 0;
            }
        }
    }
    //混合UWB的PID和视觉定位PID,并输出
    if(index == 1)
        to_one_point(cur_location, next_target, Task1_Type1, is_near_target + index, 1);
    else if(index == 3)
        to_one_point(cur_location, next_target, Task1_Type2, is_near_target + index, 1);
    else if(index == 0 || index == 2)
        to_one_point(cur_location, next_target, 8, is_near_target +index, 0);//无效视觉
    if(index == 4) return true;
    else return false;
}

bool landon_C(int height, float* current_location, bool *is_SetStartPoint)
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
    path[0][0] = takeoff_location[0]; path[0][1] = Task1_Point2_y;
    path[1][0] = takeoff_location[0]; path[1][1] = takeoff_location[1];
    index = getCurrentTarget(current_location, next_target, 2, ld_path_flag, path, 10, 0, 8);

    printf("height = %d\r\n", height);
    printf("x:%f y:%f\r\n", current_location[0], current_location[1]);
    printf("target_x:%d, target_y:%d, index:%d\r\n", next_target[0], next_target[1], index);
    printf("flag:%d %d %d %d %d\r\n", drop_goods_flag[0], drop_goods_flag[1], drop_goods_flag[2], drop_goods_flag[3], drop_goods_flag[4]);
    
    if(drop_goods_flag[0] == 1)
        land(height);
    if(index == 1)
        to_one_point(current_location, next_target, 0, is_near_target_land + index, 1);
    else
        to_one_point(current_location, next_target, 8, is_near_target_land + index, 0);
	if(height < 80) 
    {
        clear_drop_goods_flag();
        return true;
    }
	else return false;
}


void Moto_Down()
{
  HAL_GPIO_WritePin(GPIOC, Moto_down_Pin, GPIO_PIN_SET);  
  HAL_GPIO_WritePin(GPIOC, Moto_up_Pin, GPIO_PIN_RESET);  
}

void Moto_Up()
{
  HAL_GPIO_WritePin(GPIOC, Moto_down_Pin, GPIO_PIN_RESET);  
  HAL_GPIO_WritePin(GPIOC, Moto_up_Pin, GPIO_PIN_SET);  
}

void Moto_stable()
{
  HAL_GPIO_WritePin(GPIOC, Moto_down_Pin, GPIO_PIN_RESET);  
  HAL_GPIO_WritePin(GPIOC, Moto_up_Pin, GPIO_PIN_RESET);  
}

/*
func:调用时，进行投递货物任务
param1->is_begin：开始投递货物，放下砝码，滴答定时器计时到一定时间后停止下降
param2->is_over：结束投递货物，开始上升砝码，读到开关电平时保持静止
param3->drop_over：下降完成后置1
*/
bool Throw_Moto(bool *is_begin, bool *is_over, bool *drop_over)
{
    static uint32_t down_time;
    if(*is_begin)
    {
        down_time = HAL_GetTick();
        Moto_Down();
        *is_begin  = false;
    }
    if (HAL_GetTick() - down_time >= 7000 && *drop_over == false)
    {
        Moto_stable();
        drop_goods_flag[1] = 0;
        drop_goods_flag[2] = 1;
        *drop_over = true;
    }
    if(*is_over)
    {
        Moto_Up();
        if(HAL_GPIO_ReadPin(GPIOC,Moto_Pin_Pin) == GPIO_PIN_SET)
        {
            Moto_stable();
            return true;
        }
    }
    return false;
}

//direction为飞行方向，1为右飞，0为左飞,time为飞行时间
bool fly_x(bool direction, int time)
{
    if(!start_time)
    {
        count_time = time;//计时时间
        TIM10_Set(1);
    }
		printf("tim10_cnt:%d\r\n", tim10_1ms);
    if(time_over)
    {
        Set_PWM_Roll(4500);
        printf("fly time over\r\n");
        return true;
    }
    else
    {
        Set_PWM_Roll(4500 + (direction?400:-400));
        printf("pwm_roll:%d\r\n", 4500 + (direction?400:-400));
        return false;
    }
}

//direction为飞行方向，1为后飞，0为前飞,time为飞行时间
bool fly_y(bool direction, int time)
{
    if(!start_time)
    {
        count_time = time;//计时时间
        TIM10_Set(1);
    }
    if(time_over)
    {
        Set_PWM_Pitch(4500);
        return true;
    }
    else
    {
        Set_PWM_Pitch(4500 + (direction?300:-300));
        return false;
    }
}

//mode为飞行模式,0为定高模型,1为悬停模式,time为切换模式延时时间
bool switch_mode(bool mode, int time)
{
    if(!start_time)
    {
        count_time = time;//计时时间
        TIM10_Set(1);
    }
    if(time_over)
    {
        if(mode)
        {
            Set_PWM_Mode(4500);
            return true;
        }
        else
        {
            Set_PWM_Mode(3000);
            return true;
        }
    }
    return false;
}

//direction为飞行方向，1为下飞，0为上飞,time为飞行时间
bool fly_z(bool direction, int time)
{
    if(!start_time)
    {
        count_time = time;//计时时间
        TIM10_Set(1);
    }
    if(time_over)
    {
        Set_PWM_Thr(4500);
        return true;
    }
    else
    {
        Set_PWM_Thr(4500 + (direction?-600:600));
        printf("pwm_thr:%d\r\n", 4500 + (direction?-600:600));
        return false;
    }
}

//direction为舵机方向，1为下放，0为上拉,time为控制时间
bool drop_goods(bool direction, int time)
{
    if(!start_time)
    {
        if(direction)
        {
            Moto_Down();
        }
        else
        {
            Moto_Up();
        }
        count_time = time;//计时时间
        TIM10_Set(1);
    }
    if(time_over)
    {
        Moto_stable();
        return true;
    }
    if(direction == 0 && HAL_GPIO_ReadPin(GPIOC,Moto_Pin_Pin) == GPIO_PIN_SET)
    {
        Moto_stable();
        return true;
    }
    return false;
}
//time是蜂鸣器项的时间
bool beep_on(int time)
{
    if(!start_time)
    {
        BEEP_ON();
        count_time = time;//计时时间
        TIM10_Set(1);
    }
    if(time_over)
    {
        BEEP_OFF();
        return true;
    }
    return false;
}

void Open_view(uint16_t TaskType)
{
    Pack_cmd_buf(TaskType, 1, cmd_buf);
    BSP_USART_SendArray_LL(USART2, cmd_buf, 4);
    t1_opt_flag = 1;
}

void Close_view(uint16_t TaskType)
{
    Pack_cmd_buf(TaskType, 0, cmd_buf);
    BSP_USART_SendArray_LL(USART2, cmd_buf, 4);
    t1_opt_flag = 0;
}

void init_time_array()
{
    for(int i = 0; i < 5; ++i)
        for(int j = 0; j < 5; ++j)
            time_array[i][j] = 0;
    time_array[1][1] = 1000;
}

bool view_location(uint16_t TaskType, int time)
{
    if(!start_time)
    {
        Open_view(TaskType);
        count_time = time;//计时时间
        TIM10_Set(1);
    }
    if(time_over)
    {
        Close_view(TaskType);
        return true;
    }
    return false;
}

bool TaskOne_D()
{
    printf("takeone_state:%d\r\n", taskone_state);
    switch (taskone_state)
    {
    case 0: 
        if(switch_mode(0, 1500)){
            taskone_state = 1;
            start_time = 0;
        }
        break;
    case 1:
        if(fly_x(1, time_array[Task1_index_x1][Task1_index_y1]))
        {
            taskone_state = 2;
            start_time = 0;
        }
        break;
    case 2:
        if(switch_mode(1, 1500)){
            taskone_state = 3;
            start_time = 0;
        }
        break;
    case 3:
        Open_view(Task1_Type1);
        taskone_state = 4;
        break;
    case 4:
        if(fly_z(1, 200)){
            taskone_state = 5;
            start_time = 0;
        }
        break;
    case 5:
        if(drop_goods(1, 7000)){
            taskone_state = 6;
            start_time = 0;
        }
        break;
    case 6:
        if(beep_on(5000)){
            taskone_state = 7;
            start_time = 0;
        }
        break;
    case 7:
        if(drop_goods(0, 7000)){
            taskone_state = 8;
            start_time = 0;
        }
        break;
    case 8:
        if(fly_z(0, 200)){
            taskone_state = 9;
            start_time = 0;
        }
        break;
    case 9:
        Close_view(Task1_Type1);
        taskone_state = 10;
        break;
    case 10:
        if(switch_mode(0, 1500)){
            taskone_state = 11;
            start_time = 0;
        }
        break;
    case 11:
        if(fly_x(0, time_array[Task1_index_x1][Task1_index_x2])){
            taskone_state = 10;
            start_time = 0;
        }
        break;
    case 12:
        if(switch_mode(1, 1500)){
            taskone_state = 13;
            start_time = 0;
        }
        break;
    case 13:
        if(view_location(Task1_Type1, 5000)){
            taskone_state = 14;
            start_time = 0;
        }
        break;
    default:
        return true;
    }
    if(t1_opt_flag)
    {
        Loiter(Attitude.Position_x, Attitude.Position_y, Attitude.SetPoint_x, Attitude.SetPoint_y, 0, 0);
        Set_PWM_Pitch(pwm_pitch_SensorOut);
        Set_PWM_Pitch(pwm_roll_SensorOut);
    }
    return false;
}