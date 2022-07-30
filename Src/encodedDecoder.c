#include "coderDecoder.h"
#include "stdio.h"
#include "uart_api.h"
#include "flyControl.h"
#include "Matrix.h"

#define BUFFSIZE 11  

extern uint8_t encodeAnswer[BUFFSIZE];
ATTITUDE    Attitude; 

uint8_t encodeDecode_Analysis(uint8_t *inBuf,uint8_t *outBuf,uint16_t Buflen)
{
	//printf("\r\n");  
	if(Buflen>=2)
	{
		//**********************#*****��Ϣ��**λ��XH*****λ��XL***λ��YH****λ��YL*****Ŀ��λ��H***Ŀ��λ��L***β*******
    //********************* #ͷ***��Ϣ�ţ�����λ1L������λ1H,����λ2L,����λ2H,����λ3L,����λ3H,����λ4L,����λ4H*****֡β* *********
		if((inBuf[0]==0x23) && (inBuf[26]==0x2A))
		{
			//1����Ϣ�����ڽ����Ӿ����λ����Ϣ
			if(inBuf[1]=='1')		//��ͣ��Ŀ���
			{
				//printf("*****��һ����Ϣ************ \r\n"); 
				Attitude.SetPoint_x= inBuf[3]<<8|inBuf[2];
				Attitude.SetPoint_y = inBuf[5]<<8|inBuf[4];
				Attitude.Position_x = inBuf[7]<<8|inBuf[6];
				Attitude.Position_y = inBuf[9]<<8|inBuf[8];
				outBuf[2]=inBuf[2]; //���յ�����Ч����1
				outBuf[3]=inBuf[3]; //���յ�����Ч����2
				outBuf[4]=inBuf[4]; //���յ�����Ч����3
				outBuf[5]=inBuf[5]; //���յ�����Ч����4
				outBuf[6]=inBuf[6]; //���յ�����Ч����5
				outBuf[7]=inBuf[7]; //���յ�����Ч����6
				outBuf[8]=inBuf[8]; //���յ�����Ч����7
				outBuf[9]=inBuf[9]; //���յ�����Ч����8
				printf("x: %d, y: %d\r\n", Attitude.Position_x, Attitude.Position_y);
				//printf("Attitude Position x = %d, Attitude Position y = %d, SetPoint x = %d, SetPoint y = %d \r\n", Attitude.Position_x, Attitude.Position_y, Attitude.SetPoint_x, Attitude.SetPoint_y);
				return 1;
			}
			/*****�޸Ļ�վλ����Ϣ*****/
			else if(inBuf[1] == '2')
			{
				STATION1_X = (float)(inBuf[3]<<8 | inBuf[2]);
				STATION1_Y = (float)(inBuf[5]<<8 | inBuf[4]);
				STATION1_Z = (float)(inBuf[7]<<8 | inBuf[6]);
				STATION2_X = (float)(inBuf[9]<<8 | inBuf[8]);
				STATION2_Y = (float)(inBuf[11]<<8 | inBuf[10]);
				STATION2_Z = (float)(inBuf[13]<<8 | inBuf[12]);
				STATION3_X = (float)(inBuf[15]<<8 | inBuf[14]);
				STATION3_Y = (float)(inBuf[17]<<8 | inBuf[16]);
				STATION3_Z = (float)(inBuf[19]<<8 | inBuf[18]);
				STATION4_X = (float)(inBuf[21]<<8 | inBuf[20]);
				STATION4_Y = (float)(inBuf[23]<<8 | inBuf[22]);
				STATION4_Z = (float)(inBuf[25]<<8 | inBuf[24]);
				BEEP_ON();
				HAL_Delay(400);
				BEEP_OFF();
				HAL_Delay(200);
				
				BEEP_ON();
				HAL_Delay(1000);
				BEEP_OFF();
				printf("%f %f %f %f %f %f %f %f %f %f %f %f\r\n", STATION1_X, STATION1_Y, STATION1_Z, STATION2_X, STATION2_Y, STATION2_Z, STATION3_X, STATION3_Y, STATION3_Z, STATION4_X, STATION4_Y, STATION4_Z);

				return 2;
			}
			//����Ŀ���λ����Ϣ
			else if(inBuf[1] == '3')		
			{
				Task1_Point1_x = inBuf[3]<<8 | inBuf[2];
				Task1_Point1_y = inBuf[5]<<8 | inBuf[4];
				Task1_Point2_x = inBuf[7]<<8 | inBuf[6];
				Task1_Point2_y = inBuf[9]<<8 | inBuf[8];
				Task1_Type1 = inBuf[11]<<8 | inBuf[10];
				Task1_Type2 = inBuf[13]<<8 | inBuf[12];
				printf("%d %d %d %d %d %d", Task1_Point1_x, Task1_Point1_y, Task1_Point2_x, Task1_Point2_y, Task1_Type1, Task1_Type2);
				return 3;
			}
			else if(inBuf[1] == '4')		//���ҷ�
			{
				return 4;
			}
			else if(inBuf[1] == '5')		//���ҷ�
			{
				return 5;
			}
		}
		else return 0;
	}
	else return 0;
}


bool encodeDecode_Analysis_UWB(uint8_t *inBuf,float *outBuf,uint16_t Buflen)
{
	static bool flag = 0;
	static int head = 0;
	static int uwb_id = 0;
	//printf("ok\r\n");
	for(int i = 0; i < Buflen; ++i)
	{
		//printf("%c", inBuf[i]);
		if(inBuf[i] == 0x24)
		{
			head = i;
			flag = 1;
		}
		else
		{
			if(flag)
			{
				if(i == head + 10)
					uwb_id = inBuf[i] - '1' + 1;
				else if(i - head > 10)
				{
					if(inBuf[i] == 0x0D)
					{
						outBuf[uwb_id - 1] = 0;
						for(int j = i - head - 12; j > 0; --j)
						{
							//printf("%c", inBuf[i - j]);
							if(inBuf[i - j] == 0x2E)
								continue;
							outBuf[uwb_id - 1] *= 10;
							outBuf[uwb_id - 1] += inBuf[i - j] - '0';
							//printf("%f ", outBuf[uwb_id - 1]);
						}
						outBuf[uwb_id - 1] /= 100.00;
						flag = 0;
						// printf("distance:%f, uwb_id:++%d\r\n", outBuf[uwb_id - 1], uwb_id);
					}
				}
			}	
		}
	}
	flag = 0;
	for(int i = 0; i < 4; ++i)
		if(outBuf[i] == 0)
			return false;
	return true;
}

bool encodeDecode_Analysis_SecondBoard(uint8_t *inBuf, uint16_t Buflen)
{
	uint8_t Data_buf[4];
	if(Buflen >= 7)
	{
		//ͨ�Ű���0xFE ID yaw*4 0xFD
		if(inBuf[0] == 0xFE && inBuf[1] == 0x01)
		{
			for(int i=0;i<4;i++)
				Data_buf[i] = inBuf[2+i];
			yaw = *((float *)Data_buf);
		}
		return true;
	}
	else return false;
}

/*
����uwb���ڷ�������ͨ�Ű�����������distance������10m�������
paragram:
inbuf->�����յ������ĸ���վ��ͨ�Ű�����
dist ->�ĸ���վ�ľ�����������
*/
void encodeDecode_UWBpacket(uint8_t inbuf[][20], float *dist)
{
	int s_id;
	uint8_t integer, decimal;
	for(int i=0;i<4;i++)
	{
		s_id = inbuf[i][10];
		integer = inbuf[i][12];							//mΪ��λ
		decimal = inbuf[i][14] * 10 + inbuf[i][15];		//cmΪ��λ
		*(dist + s_id - 1) = (float)(integer * 100 + decimal);	//��һ����cmΪ��λ
	}
}


