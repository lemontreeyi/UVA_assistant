#ifndef  __KEY_H__
#define  __KEY_H__
#include "stm32f4xx_hal.h"
#include "beep.h"
#include "led.h"
#include "uart_api.h"
#include "stm32f4xx.h"

/*???????????????????IO*/
#define KEY1   GPIO_ReadInputDataBit(GPIOB,GPIO_PIN_5) //PB5
#define KEY2   GPIO_ReadInputDataBit(GPIOB,GPIO_PIN_4) //PB4 
#define KEY3   GPIO_ReadInputDataBit(GPIOB,GPIO_PIN_8) //PB8
#define KEY4   GPIO_ReadInputDataBit(GPIOB,GPIO_PIN_9) //PB9


#define KEY1_PRES 	1
#define KEY2_PRES		2
#define KEY3_PRES 	3
#define KEY4_PRES		4

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Initialization
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void KEY_Init(void); //IO???
uint8_t KeyScanning(uint8_t mode);
void GetKey_NUM(void);

#endif


