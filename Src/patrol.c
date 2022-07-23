#include "patrol.h"

float path[FLYPATH_LEN][2];
bool path_flag[FLYPATH_LEN];
void init_flypath()
{
    path[0][0] = 3.50; path[0][1] = 2.00;
    path[1][0] = 2.80; path[1][1] = 2.00;
    path[2][0] = 2.20; path[2][1] = 2.00;
    path[3][0] = 1.50; path[3][1] = 2.00;

    for (int i = 0; i < FLYPATH_LEN; ++i)
    {
        path_flag[i] = false;
    }
    
}

void getCurrentTarget(float* current_location, int* target_location)
{
    int have_arrive = 0;
    for(have_arrive = 0; have_arrive < FLYPATH_LEN; ++have_arrive)
    {
        if(!path_flag[have_arrive])
            break;
    }
    if((path[have_arrive][0] - current_location[0]) * (path[have_arrive][0] - current_location[0]) + (path[have_arrive][1] - current_location[1]) * (path[have_arrive][1] - current_location[1]) < 0.0225)
    {
        path_flag[have_arrive] = true;
        ++have_arrive;
    }
    if(have_arrive < FLYPATH_LEN)
    {
        target_location[0] = (int)(path[have_arrive][0]*100);
        target_location[1] = (int)(path[have_arrive][1]*100);
    }
    else
    {
        target_location[0] = (int)(path[FLYPATH_LEN - 1][0]*100);
        target_location[1] = (int)(path[FLYPATH_LEN - 1][1]*100);
    }
}