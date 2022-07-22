#ifndef _PATROL_
#define _PATROL_

#include <stdbool.h>

#define FLYPATH_LEN 4

void init_flypath();
void getCurrentTarget(float* current_location, float* target_location);

#endif