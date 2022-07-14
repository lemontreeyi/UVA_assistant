#include "key.h"
#include "main.h"
#include "stdio.h"
#include "beep.h"

#define S1  HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)
#define S2  HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin)
#define S3  HAL_GPIO_ReadPin(KEY3_GPIO_Port,KEY3_Pin)
#define S4  HAL_GPIO_ReadPin(KEY4_GPIO_Port,KEY4_Pin)

//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//0，没有任何按键按下
//1，WKUP按下 WK_UP
//注意此函数有响应优先级,KEY0>KEY1>KEY2>WK_UP!!
uint8_t KeyScanning(uint8_t mode)
{
    static uint8_t key_up=1;     //按键松开标志
    if(mode==1)key_up=1;         //支持连按
    if(key_up&&(S1==GPIO_PIN_RESET||S2==GPIO_PIN_RESET))
    {
			  //printf(" KEY_Scan \r\n");
        HAL_Delay(30);
        key_up=0;	//表示按键已按下
        if(S1==GPIO_PIN_RESET)        
				{
					printf("KEY1_PRES \r\n");
					return  KEY1_PRES;
				}
        else if(S2==GPIO_PIN_RESET)
				{
					printf("KEY2_PRES \r\n");
					return  KEY2_PRES;
				}
    }
	else if(S1==GPIO_PIN_SET&&S2==GPIO_PIN_SET)
		{
			key_up=1;
			printf("no key pres...");
		}
    return 0;   //无按键按下
}			



void GetKey_NUM(void)
{
	uint8_t key;
	key=KeyScanning(0);            //按键扫描
	switch(key)
	{				 
		case  KEY1_PRES:	//控制LED0,LED1互斥点亮
			LED2_Flash();
			BEEP_ON();
			break;
		case  KEY2_PRES:	//控制LED0翻转
			LED2_Flash();
			BEEP_ON();
			break;
	}
}
	
