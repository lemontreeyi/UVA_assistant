#include "patrol.h"

float path[FLYPATH_LEN][2];
bool path_flag[FLYPATH_LEN];
void init_flypath()
{
    path[0][0] = 350; path[0][1] = 200;
    path[1][0] = 280; path[1][1] = 200;
    path[2][0] = 220; path[2][1] = 200;
    path[3][0] = 150; path[3][1] = 200;

    for (int i = 0; i < FLYPATH_LEN; ++i)
    {
        path_flag[i] = false;
    }
    
}

void getCurrentTarget(float* current_location, float* target_location)
{
    int have_arrive = 0;
    for(have_arrive = 0; have_arrive < FLYPATH_LEN; ++have_arrive)
    {
        if(!path_flag[have_arrive])
            break;
    }
    if((path[have_arrive][0] - current_location[0]) * (path[have_arrive][0] - current_location[0]) + (path[have_arrive][1] - current_location[1]) * (path[have_arrive][1] - current_location[1]) < 0.01)
    {
        path_flag[have_arrive] = true;
        ++have_arrive;
    }
    if(have_arrive < FLYPATH_LEN)
    {
        target_location[0] = path[have_arrive][0];
        target_location[1] = path[have_arrive][1];
    }
    else
    {
        target_location[0] = path[FLYPATH_LEN - 1][0];
        target_location[1] = path[FLYPATH_LEN - 1][1];
    }
}