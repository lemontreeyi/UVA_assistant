#include "key.h"
#include "main.h"
#include "stdio.h"
#include "beep.h"

#define S1  HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)
#define S2  HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin)
#define S3  HAL_GPIO_ReadPin(KEY3_GPIO_Port,KEY3_Pin)
#define S4  HAL_GPIO_ReadPin(KEY4_GPIO_Port,KEY4_Pin)

void  KEY_Init() 
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitStructure.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8; //KEY1 KEY2 KEY3对应引脚
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;                //普通输入模式
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;     //100M
  GPIO_InitStructure.Pull = GPIO_PULLUP;                    //上拉
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);                //初始化GPIOB4,5,8
  GPIO_InitStructure.Pin = GPIO_PIN_9;                      //KEY4对应引脚PB9
  GPIO_InitStructure.Pull = GPIO_PULLDOWN ;                 //下拉
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);                //初始化GPIOB9
} 
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
    if(key_up&&(S1==0||S2==0||S3==0||S4==1))
    {
			  printf(" KEY_Scan \r\n");
        HAL_Delay(10);
        key_up=0;	//表示按键已按下
        if(S1==0)        
				{
					printf("KEY1_PRES \r\n");
					return  KEY1_PRES;
				}
        else if(S2==0)
				{
					printf("KEY2_PRES \r\n");
					return  KEY2_PRES;
				}
        else if(S3==0)   
				{
					printf("KEY3_PRES \r\n");
					return  KEY3_PRES;
				}					
        else if(S4==1)   
				{
					printf("KEY4_PRES \r\n");
				  return  KEY4_PRES;  
				}  
    }else if(S1==1&&S2==1&&S3==1&&S4==0)
		{
			key_up=1;
			//printf("key_up=1 \r\n");
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
	
