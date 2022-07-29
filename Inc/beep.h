#ifndef  __BEEP_H__
#define  __BEEP_H__
#include "stm32f4xx_hal.h"
#include "main.h"

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Initialization
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

typedef struct Beep_time_t
{
    uint32_t last_time;
    uint32_t interval;
} Beep_time_t;

void BEEP_ON(void);
void BEEP_OFF(void);
void init_Beeptim(Beep_time_t* beep_tim);
void BEEP_timing_on(Beep_time_t* beep_tim, uint32_t interval);
void BEEP_timing_off(Beep_time_t* beep_tim);

#endif


