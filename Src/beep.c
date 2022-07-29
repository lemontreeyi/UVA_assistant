
#include "beep.h"

void BEEP_ON(void)
{
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
}

void BEEP_OFF(void)
{
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET );
}

void init_Beeptim(Beep_time_t* beep_tim)
{
  beep_tim->interval = 0;
  beep_tim->last_time = 0;
}
//启动蜂鸣器
void BEEP_timing_on(Beep_time_t* beep_tim, uint32_t interval)
{
  beep_tim->last_time = HAL_GetTick();
  beep_tim->interval = interval;
  BEEP_ON();
}
//检查是否到达定时时间，如果到达就关闭蜂鸣器
void BEEP_timing_off(Beep_time_t* beep_tim)
{
  if((beep_tim->interval != 0) && (HAL_GetTick() - beep_tim->last_time > beep_tim->interval))
  {
    BEEP_OFF();
    beep_tim->interval = 0;
  }
}
