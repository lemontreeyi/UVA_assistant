#ifndef  __LED_H__
#define  __LED_H__
#include "stm32f4xx_hal.h"

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Initialization
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void LED1_Flash(void);
void LED2_Flash(void); 
void LED3_Flash(void); 

void LED1_Slow_Flash(void);
void LED2_Slow_Flash(void); 
void LED3_Slow_Flash(void); 

//*****иак╦********
void LED_R_Flash(void);
void LED_G_Flash(void);
void LED_B_Flash(void);

void LED_R_Off(void);
void LED_G_Off(void);
void LED_B_Off(void);



#endif


