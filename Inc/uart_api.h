#ifndef  __UART_API__
#define  __UART_API__

#include "stm32h7xx_hal.h"
#include "stdio.h"


void BSP_USART_SendData_LL(USART_TypeDef* huart, uint8_t Data);
void BSP_USART_SendArray_LL(USART_TypeDef* huart, uint8_t* pdata, uint32_t size);
//void delay_ms(int32_t nms);
void BSP_USART_StartIT_LL(USART_TypeDef* huart);
void USART_Send_Check(USART_TypeDef* huart, uint8_t *Buffer, uint8_t len);
void USART_Send_out(USART_TypeDef* huart, uint16_t *data, uint8_t len, uint8_t send);

#endif
