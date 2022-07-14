#include "stm32h7xx_hal.h"
#include "stm32h7xx_ll_usart.h"
#include "uart_api.h"


void BSP_USART_SendData_LL(USART_TypeDef* huart, uint8_t Data)
{
    /* Check the parameters */
    assert_param( huart );

    /* Wait for TXE flag to be raised */
    while (!LL_USART_IsActiveFlag_TXE(huart))
    {
    }

    /* Write character in Transmit Data register.
    TXE flag is cleared by writing data in DR register */
    LL_USART_TransmitData8(huart, Data);


    /* Wait for TC flag to be raised for last char */
    while (!LL_USART_IsActiveFlag_TC(huart))
    {
    }  
}


void BSP_USART_SendArray_LL(USART_TypeDef* huart, uint8_t* pdata, uint32_t size)
{
    /* Check the parameters */
    assert_param( huart );

    for( uint32_t i=0; i<size; i++)
    {
        /* Wait for TXE flag to be raised */
        while (!LL_USART_IsActiveFlag_TXE(huart))
        {
        }

        /* Write character in Transmit Data register.
        TXE flag is cleared by writing data in DR register */
        LL_USART_TransmitData8(huart, *pdata++);


        /* Wait for TC flag to be raised for last char */
        while (!LL_USART_IsActiveFlag_TC(huart))
        {
        }  
    }
}


void BSP_USART_StartIT_LL(USART_TypeDef* huart)
{
	/* Check the parameters */
	assert_param( huart );
	
	/* Clear Overrun flag, Enable RXNE and Error interrupts */
	LL_USART_ClearFlag_ORE(huart);  
	LL_USART_EnableIT_RXNE(huart);
	//LL_USART_EnableIT_ERROR(huart);
}

/*
发送多字节数据 + 校验和->主要用于测距模块的串口通信
*/
void USART_Send_Check(USART_TypeDef* huart, uint8_t *Buffer, uint8_t len)
{
    uint8_t i = 0;
    while(i < len)
    {
        if(i < (len-1)) Buffer[len-1] += Buffer[i];
        LL_USART_TransmitData8(huart, Buffer[i++]);
    }
}

/*
发送一帧数据
*/
void USART_Send_out(USART_TypeDef* huart, uint16_t *data, uint8_t len, uint8_t send)
{
    uint8_t Tx_Data[30] = {0}, i = 0, k = 0;
    memset(Tx_Data, 0, (2*len+5));
    Tx_Data[i++] = 0x5A;        //Byte0:帧头
    Tx_Data[i++] = 0x5A;        //Byte1:帧头
    Tx_Data[i++] = send;        //Byte2:功能字节，本帧数据类型
    Tx_Data[i++] = 2 * len;     //Byte3:数据个数
    for(k=0;k<len;k++)          //存入数据到缓存Tx_Data
    {
        Tx_Data[i++] = (uint8_t)data[k]>>8;
        Tx_Data[i++] = (uint8_t)data[k];
    }
    USART_Send_Check(huart, Tx_Data, 2 * len + 5);

}


//void delay_ms(int32_t nms)
// {
//  int32_t temp;
//  SysTick->LOAD = 8000*nms;
//  SysTick->VAL=0X00;//清空计数器
//  SysTick->CTRL=0X01;//使能，减到0是无动作，采用外部时钟源
//  do
//  {
//       temp=SysTick->CTRL;//读取当前倒计数值
//  }
//     while((temp&0x01)&&(!(temp&(1<<16))));//等待时间到达
//     
//     SysTick->CTRL=0x00; //关闭计数器
//     SysTick->VAL =0X00; //清空计数器
// }
 
