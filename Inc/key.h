#ifndef  __KEY_H__
#define  __KEY_H__
#include "stm32h7xx_hal.h"
#include "beep.h"
#include "led.h"
#include "uart_api.h"

#define KEY1_PRES 1
#define KEY2_PRES 2
#define KEY3_PRES 3
#define KEY4_PRES 4

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Initialization
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
uint8_t KeyScanning(uint8_t mode);
void GetKey_NUM(void);

#endif


