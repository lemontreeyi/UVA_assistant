#ifndef  __UART_API__
#define  __UART_API__

#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "main.h"
#include "string.h"

extern uint8_t cxof_buf[9];
void BSP_USART_SendData_LL(USART_TypeDef* huart, uint8_t Data);
void BSP_USART_SendArray_LL(USART_TypeDef* huart, uint8_t* pdata, uint32_t size);
//void delay_ms(int32_t nms);
void BSP_USART_StartIT_LL(USART_TypeDef* huart);
void USART_Send_Check(USART_TypeDef* huart, uint8_t *Buffer, uint8_t len);
void USART_Send_out(USART_TypeDef* huart, uint16_t *data, uint8_t len, uint8_t send);

void Pack_cxof_buf(float *speed, uint8_t quality, uint8_t *cxof_buf);
void Send_cxof_buf(USART_TypeDef* huart, uint8_t *buf, uint32_t size);

void Pack_cmd_buf(uint8_t cmd, uint8_t status, uint8_t* buf);

#endif
