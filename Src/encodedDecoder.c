#include "coderDecoder.h"
#include "stdio.h"
#include "uart_api.h"
#include "flyControl.h"

#define BUFFSIZE 11  

extern uint8_t encodeAnswer[BUFFSIZE];
ATTITUDE    Attitude; 

uint8_t encodeDecode_Analysis(uint8_t *inBuf,uint8_t *outBuf,uint16_t Buflen)
{
	//printf("\r\n");  
	if(Buflen>=2)
	{
		//**********************#*****消息号**位置XH*****位置XL***位置YH****位置YL*****目标位置H***目标位置L***尾*******
    //********************* #头***消息号，数据位1L，数据位1H,数据位2L,数据位2H,数据位3L,数据位3H,数据位4L,数据位4H*****帧尾* *********
		if((inBuf[0]==0x23) && (inBuf[10]==0x2A))
		{
			//开始
			if(inBuf[1]=='1')		//悬停到目标点
			{
				//printf("*****第一号消息************ \r\n"); 
				Attitude.SetPoint_x= inBuf[3]<<8|inBuf[2];
				Attitude.SetPoint_y = inBuf[5]<<8|inBuf[4];
				Attitude.Position_x = inBuf[7]<<8|inBuf[6];
				Attitude.Position_y = inBuf[9]<<8|inBuf[8];
				outBuf[2]=inBuf[2]; //接收到的有效数据1
				outBuf[3]=inBuf[3]; //接收到的有效数据2
				outBuf[4]=inBuf[4]; //接收到的有效数据3
				outBuf[5]=inBuf[5]; //接收到的有效数据4
				outBuf[6]=inBuf[6]; //接收到的有效数据5
				outBuf[7]=inBuf[7]; //接收到的有效数据6
				outBuf[8]=inBuf[8]; //接收到的有效数据7
				outBuf[9]=inBuf[9]; //接收到的有效数据8
				//printf("x: %d, y: %d\r\n", Attitude.Position_x, Attitude.Position_y);
				//printf("Attitude Position x = %d, Attitude Position y = %d, SetPoint x = %d, SetPoint y = %d \r\n", Attitude.Position_x, Attitude.Position_y, Attitude.SetPoint_x, Attitude.SetPoint_y);
				return 1;
			}
			/* 以下指令可能会用于完成要求的飞行动作 */
			else if(inBuf[1] == '2')		//向右飞
			{
				return 2;
			}
			else if(inBuf[1] == '3')		//向左飞
			{
				return 3;
			}
			else if(inBuf[1] == '4')		//向右飞
			{
				return 4;
			}
			else if(inBuf[1] == '5')		//向右飞
			{
				return 5;
			}
			else return 0;
		}
		else return 0;
	}
	else return 0;
}


uint8_t encodeDecode_Analysis_UWB(uint8_t *inBuf,uint8_t *outBuf,uint16_t Buflen)
{
	//printf("\r\n");  
	if(Buflen>=2)
	{
		if((inBuf[0]==0x23) && (inBuf[10]==0x2A))
		{
			//开始
			if(inBuf[1]=='1')		//悬停到目标点
			{
				//printf("*****第一号消息************ \r\n"); 
				Attitude.SetPoint_x= inBuf[3]<<8|inBuf[2];
				Attitude.SetPoint_y = inBuf[5]<<8|inBuf[4];
				Attitude.Position_x = inBuf[7]<<8|inBuf[6];
				Attitude.Position_y = inBuf[9]<<8|inBuf[8];
				outBuf[2]=inBuf[2]; //接收到的有效数据1
				outBuf[3]=inBuf[3]; //接收到的有效数据2
				outBuf[4]=inBuf[4]; //接收到的有效数据3
				outBuf[5]=inBuf[5]; //接收到的有效数据4
				outBuf[6]=inBuf[6]; //接收到的有效数据5
				outBuf[7]=inBuf[7]; //接收到的有效数据6
				outBuf[8]=inBuf[8]; //接收到的有效数据7
				outBuf[9]=inBuf[9]; //接收到的有效数据8
				//printf("x: %d, y: %d\r\n", Attitude.Position_x, Attitude.Position_y);
				return 1;
			}
		}
	}
	else return 0;
}


/*
解析uwb串口发过来的通信包，仅适用于distance不超过10m的情况！
paragram:
inbuf->串口收到来自四个基站的通信包数组
dist ->四个基站的距离数据数组
*/
void encodeDecode_UWBpacket(uint8_t inbuf[][20], float *dist)
{
	int s_id;
	uint8_t integer, decimal;
	for(int i=0;i<4;i++)
	{
		s_id = inbuf[i][10];
		integer = inbuf[i][12];							//m为单位
		decimal = inbuf[i][14] * 10 + inbuf[i][15];		//cm为单位
		*(dist + s_id - 1) = (float)(integer * 100 + decimal);	//归一化到cm为单位
	}
}


