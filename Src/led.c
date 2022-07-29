#include "led.h"
#include "main.h"

#define LED1_ON   HAL_GPIO_WritePin(GPIOE, LED_B_Pin, GPIO_PIN_RESET);
#define LED1_OFF  HAL_GPIO_WritePin(GPIOE, LED_B_Pin, GPIO_PIN_SET);
#define LED2_ON   HAL_GPIO_WritePin(GPIOE, LED_G_Pin, GPIO_PIN_RESET);
#define LED2_OFF  HAL_GPIO_WritePin(GPIOE, LED_G_Pin, GPIO_PIN_SET);
#define LED3_ON   HAL_GPIO_WritePin(GPIOE, LED_R_Pin, GPIO_PIN_RESET);
#define LED3_OFF  HAL_GPIO_WritePin(GPIOE, LED_R_Pin, GPIO_PIN_SET);

// void Moto_Down()
// {
//   HAL_GPIO_WritePin(GPIOC, Moto_down_GPIO_Port, GPIO_PIN_SET);  //?????????
//   HAL_GPIO_WritePin(GPIOC, Moto_up_GPIO_Port, GPIO_PIN_RESET);  //?????????
// }

// void Moto_Up()
// {
//   HAL_GPIO_WritePin(GPIOC, Moto_down_GPIO_Port, GPIO_PIN_RESET);  //?????????
//   HAL_GPIO_WritePin(GPIOC, Moto_up_GPIO_Port, GPIO_PIN_SET);  //?????????
// }



void LED1_Flash(void)
{
  HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
}

void LED2_Flash(void)
{
  HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
}

void LED3_Flash(void)
{
  HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
}

void LED1_Slow_Flash(void)
{
  LED1_ON;
  HAL_Delay(100);
  LED1_OFF;
  HAL_Delay(100);
}

void LED2_Slow_Flash(void)
{
  LED2_ON;
  HAL_Delay(100);
  LED2_OFF;
  HAL_Delay(10);
}

void LED3_Slow_Flash(void)
{
  LED3_ON;
  HAL_Delay(100);
  LED3_OFF;
  HAL_Delay(10);
}

void LED_R_Off(void)
{
  LED3_OFF;
}
void LED_G_Off(void)
{
  LED2_OFF;
}
void LED_B_Off(void)
{
  LED1_OFF;
}
//*****************∂•≤„”¶”√****************************
void LED_R_Flash(void)
{
  LED3_Flash();
}

void LED_G_Flash(void)
{
  LED2_Flash();
}

void LED_B_Flash(void)
{
  LED1_Flash();
}
