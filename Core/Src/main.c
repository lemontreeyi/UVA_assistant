/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brif          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h" 
#include "arm_math.h"
#include <stdarg.h>
#include <string.h> 
#include <stdbool.h>
#include "mavlink.h"
#include "mavlink_types.h"

#include "led.h"
#include "key.h"
#include "uart_api.h"
#include "beep.h"
#include "coderDecoder.h"
#include "flyControl.h"
#include "Matrix.h"
#include "Kalman.h"
#include "MAV_Altitude_Decode.h"
#define TAKEOFF

/* 定义例程名和例程发布日期 */
#define ABS(x) ((x<0)?-x:x)
#define DEMO_VER "V-1.0"

  
#define Flag_SendToUsart  1 

#define CH_NUM	1	/* 8通道 */
#define N 3
#define  voltage_ratio   204  //2.04


#define USART1_MAX_RECV_LEN		256				//最大接收缓存字节数
#define USART2_MAX_RECV_LEN		256	
#define USART3_MAX_RECV_LEN		256	
#define USART5_MAX_RECV_LEN		256	

#define BUFFSIZE 5 

//#define TAKEOFF
bool is_takeoff = 1;
int takeoff_Time;

int CHANNEL_1_RISE=0,CHANNEL_1_FALL=0,CHANNEL_1_PULSE_WIDE=0;
int CHANNEL_2_RISE=0,CHANNEL_2_FALL=0,CHANNEL_2_PULSE_WIDE=0;
int CHANNEL_3_RISE=0,CHANNEL_3_FALL=0,CHANNEL_3_PULSE_WIDE=0;
int CHANNEL_4_RISE=0,CHANNEL_4_FALL=0,CHANNEL_4_PULSE_WIDE=0;
int CHANNEL_5_RISE=0,CHANNEL_5_FALL=0,CHANNEL_5_PULSE_WIDE=0;
int CHANNEL_6_RISE=0,CHANNEL_6_FALL=0,CHANNEL_6_PULSE_WIDE=0;
int CHANNEL_7_RISE=0,CHANNEL_7_FALL=0,CHANNEL_7_PULSE_WIDE=0;
int CHANNEL_8_RISE=0,CHANNEL_8_FALL=0,CHANNEL_8_PULSE_WIDE=0;

int ICFLAG_1 = 1,ICFLAG_2 = 1,ICFLAG_3 = 1, ICFLAG_4 = 1, ICFLAG_5 = 1, ICFLAG_6 = 1, ICFLAG_7 = 1, ICFLAG_8 = 1;

int PWM_Mode_N1 =2000;
int PWM_Mode_N2 =4500;
int PWM_Mode_N3 =5000;
int PWM_Mode_N4 =7000;

uint16_t    USART1_RX_STA=0; 
uint16_t    USART2_RX_STA=0;
uint16_t    USART3_RX_STA=0; 
uint16_t    USART5_RX_STA=0; 

static int Recv_Cnt_UART1 = 0;
static int Recv_Cnt_UART2 = 0;
static int Recv_Cnt_UART3 = 0;
static int UART1_Frame_Flag = 0;
static int UART2_Frame_Flag = 0;
static int UART3_Frame_Flag = 0;
static int heartbeat = 0;
static int MAVLink_message_length = 0;
static mavlink_distance_sensor_t packet;
static float height = 0;

uint8_t USART1_RX_BUF [USART1_MAX_RECV_LEN]; 
uint8_t USART2_RX_BUF [USART2_MAX_RECV_LEN]; 
uint8_t USART3_RX_BUF [USART3_MAX_RECV_LEN];
uint8_t USART5_RX_BUF [USART5_MAX_RECV_LEN];

uint8_t FreeBuffer_Encode [USART1_MAX_RECV_LEN];
uint8_t FreeBuffer_Encode_3[USART3_MAX_RECV_LEN];
uint8_t FreeBuffer_Encode_5[USART3_MAX_RECV_LEN];
uint8_t MAVLink_RECV_BUF[USART2_MAX_RECV_LEN];
uint8_t MAVLink_TX_BUF [MAVLINK_MAX_PACKET_LEN];
uint8_t MAVLink_RECV_BUF_FAKE [USART2_MAX_RECV_LEN] = {0};

//回应 头 消息号 数据1  数据2 数据3 数据4 尾
uint8_t encodeAnswer[11]    ={'#','1',0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,'*'};

float old_value;

static      int16_t s_dat[CH_NUM];
float       s_volt[CH_NUM];      

uint32_t    ADC_ReadBuffer[CH_NUM]; 
float  OutData_Test[4]; 
float  OutData[4]; 
char   menu=0;

PID         PID_Control_Att;           //  PID Control Structure 

int mode_flag = 0;

void  OutPut_Data(void);
float ADC_CvtVolt(void);
float DataProcessing(float IN_Data);
float ADC_CvtVolt_and_filter(void);
float get_adc(char adc_id);
float filter_av(char filter_id);
void TIM3_Set(uint8_t sta);
void TIM2_Set(uint8_t sta);
int Filter_pwm(int position, int old, int max);
void TIM12_Set(uint8_t sta);
void TIM13_Set(uint8_t sta);
void TIM15_Set(uint8_t sta);
void TIM17_Set(uint8_t sta);

void Data_to_VisualScope(void);
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);

mavlink_message_t msg;
mavlink_status_t status;


// //vl53l1x
uint16_t distance = 0;
uint8_t re_buf_Data[8] = {0}, Receive_ok = 0;
uint8_t sum = 0, i = 0;
uint8_t RangeStatus = 0, Time = 0, Mode = 0;
uint16_t data = 0;

//mavlink
void MANUAL_CONTROL_Send(int16_t xpoint,int16_t ypoint);
void RC_CHANNELS_OVERRIDE_Send(int16_t xpoint,int16_t ypoint);
void heartbeat_Mavlink(void);
int  RC_Read(void);
void Back_to_Center(void);
	
//光流
uint8_t uwb_buf[4][20] = {0};
bool uwb_receive_ready = 1;
bool get_uwb_ready = 0;
bool is_cxof_ready = 1;
float distance_to_station[4] = {0, 0, 0, 0};
float distance_to_station_esm[4] = {0, 0, 0, 0};
float location[3] = {0, 0, 0};
float x_array[7] = {0, 0, 0, 0, 0, 0, 0};
float y_array[7] = {0, 0, 0, 0, 0, 0, 0};
float location_esm[3] = {0, 0, 0};
float location_esm_limit[3] = {0, 0, 0};
float location_esm_kalma[3] = {0, 0, 0};
short d_location[2] = {0 ,0};
bool Get_UWB_distance(float distance[]);


void end(void)
{
	printf("%c",0xff);
	printf("%c",0xff);
	printf("%c",0xff);
}

/* 仅允许本文件内调用的函数声明 */
static void PrintfHardInfo(void);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__  
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf 
	 set to 'Yes') calls __io_putchar() */  
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)  
#else  
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)  
#endif /* __GNUC__ */  

PUTCHAR_PROTOTYPE
{
    BSP_USART_SendData_LL( USART1, ch);
	  return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint16_t i1=0,i2=0,i3=0,i5=0;
	uint16_t rxlen_usart_1;
	uint16_t rxlen_usart_2;
  uint16_t rxlen_usart_3;
	uint16_t rxlen_usart_5;
	
	uint8_t cmd=0;
	uint8_t key;
	uint32_t RunTime=0;
  uint32_t ContrlTime_x = 0;
  uint32_t ContrlTime_y = 0;
  uint32_t ContriGetDataTime = 0;
  uint32_t ctrlstart_x = HAL_GetTick();
  uint32_t ctrlstart_y = HAL_GetTick();
  uint32_t ContriGetDataStart = HAL_GetTick();
  uint32_t Ctrl_flag_time = 0;
  uint32_t Ctrl_flag_start = HAL_GetTick();
	uint32_t decodetimer;
	uint32_t tickstart = HAL_GetTick();
	uint32_t ticklast = HAL_GetTick();
  int flag_rx2 = 0;
	mavlink_message_t msg1;
	mavlink_message_t msg2;
	mavlink_message_t msg_tmp;
	mavlink_message_t msg_altitude;
	mavlink_message_t msg_request_data_stream;
	uint32_t len = 0;
  uint8_t str[] = "hello world\r\n";
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
	Device_Init();
	PrintfHardInfo();	/* 打印硬件接线信息 */
	//init_fifo(&mav_fifo);
	/*
		由于ST固件库的启动文件已经执行了CPU系统时钟的初始化，所以不必再次重复配置系统时钟。
		启动文件配置了CPU主时钟频率、内部Flash访问速度和可选的外部SRAM FSMC初始化。
	*/

		//HAL_ADC_Init(&hadc1);
		//HAL_ADC_Start(&hadc1);		
		
		//开启PWM
		HAL_TIM_PWM_Init(&htim4);
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
		
		HAL_TIM_PWM_Init(&htim5);
		HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
		

		/// 使能定时器输入捕获。
		HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
		HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
		HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_3);
		HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_4);
		
		HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
		HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);
		HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_3);
		HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_4);

		PIDInit(&PID_Control_Att); 
		LED1_Slow_Flash();
		//BEEP_ON();
		HAL_Delay(1000);              
		BEEP_OFF();
		HAL_Delay(1000);   
		
		BSP_USART_StartIT_LL(USART1);
	  BSP_USART_StartIT_LL(USART2);
    BSP_USART_StartIT_LL(USART3);
		BSP_USART_StartIT_LL(UART5);
	
	  USART1_RX_STA=0;		//清零
	  USART2_RX_STA=0;		//清零
	
		//TIM3_Set(0);			  
	  //TIM5_Set(0);	
		

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(300);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	printf("test begin!!!\r\n");
  
  //先计算一次，舍弃掉此次
  kalman_init(&kalman_x);
  kalman_init(&kalman_y);
  for(int i = 0; i < 4; ++i)
    kalman_init(&kalman_d[i]);
  init_A_matrix();

  while (1)
  {
    // if(is_cxof_ready)
    // {
    //   if(Get_UWB_distance(distance_to_station)) is_cxof_ready = 0;
    // }
    // else
    // {
    //   calculate_location(distance_to_station, location);
    //   calculate_cxof(location, d_location);
    //   Pack_cxof_buf(d_location[0], d_location[1], 100, cxof_buf);
    //   Send_cxof_buf(UART5, cxof_buf, 9);
    //   is_cxof_ready = 1;
    // }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)printf("key1 pres...\r\n");
    if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)printf("key2 pres...\r\n");
		if(InitedFlag)  //确保设备已初始化
		{
			/********************************UART1接收并处理数据**********************************/
			if(USART1_RX_STA & 0X8000)		  //接收到一次数据了
			{
        printf("USART1 revd ...\r\n");
        //printf("USART1 INT =%d \r\n",USART1_RX_STA);				 
				rxlen_usart_1 = USART1_RX_STA & 0x7FFF;	//得到数据长度
        printf("%s\r\n", USART1_RX_BUF);
				//printf("This is a USART1 test rxlen_usart_1 = %d USART1_RX_STA= %d\r\n" ,rxlen_usart_1 ,USART1_RX_STA);
				for(i1=0;i1<rxlen_usart_1;i1++)
				{
					FreeBuffer_Encode[i1]=USART1_RX_BUF[i1];	 //将串口2接收到的数据传输给自由缓冲区
					//BSP_USART_SendArray_LL( USART1,&FreeBuffer_Encode[i1],1);
				}
				//cmd=encodeDecode_Analysis(FreeBuffer_Encode,encodeAnswer,rxlen_usart_1); //分析字符串
				BSP_USART_StartIT_LL( USART1 );
				rxlen_usart_1=0;
				USART1_RX_STA=0;	 //启动下一次接收
			}
			/********************************UART2接收并处理数据***********************************/
			if(USART2_RX_STA & 0X8000)		  //接收到一次数据，且超过了预设长度
			{			 
        //printf("USART2 revd ...\r\n");
				rxlen_usart_2 = USART2_RX_STA & 0x7FFF;	//得到数据长度
				//printf("len = %d\r\n",rxlen_usart_2);
        //BSP_USART_SendArray_LL(USART1, USART2_RX_BUF, 11);
				//printf("This is a USART1 test rxlen_usart_1 = %d USART1_RX_STA= %d\r\n" ,rxlen_usart_1 ,USART1_RX_STA);
				for(i2=0;i2<rxlen_usart_2;i2++)
				{
					FreeBuffer_Encode[i2] = USART2_RX_BUF[i2];					//将串口2接收到的数据传输给自由缓冲区
				}

				//BSP_USART_SendArray_LL(USART1, FreeBuffer_Encode, 11);
        if(rxlen_usart_2 == 11) {
          cmd = encodeDecode_Analysis(FreeBuffer_Encode,encodeAnswer,rxlen_usart_2);
        } //分析字符串
				rxlen_usart_2=0;
				USART2_RX_STA=0;
				BSP_USART_StartIT_LL( USART2 ); //启动下一次接收
			}
      /*****************************USART3接收&处理数据***********************************/
      if(USART3_RX_STA & 0x8000)    //接收满一次数据
      {
				//printf("USART3 revd ...\r\n");
        rxlen_usart_3 = USART3_RX_STA & 0x7FFF;   //得到数据长度
				//printf("%s",USART3_RX_BUF);
        for(i3=0;i3<rxlen_usart_3;i3++)
        {
          FreeBuffer_Encode_3[i3] = USART3_RX_BUF[i3];    //将串口3接收的数据传输到自由缓冲区
        }
        rxlen_usart_3 = 0;
        USART3_RX_STA = 0;
        BSP_USART_StartIT_LL( USART3 );   //启动下一次接收
      }
			/********************************UART5接收并处理数据***********************************/
			if(USART5_RX_STA & 0X0080)		  //接收到一次数据，且超过了预设长度
			{			 
        if(USART5_RX_STA > 128)
          rxlen_usart_5 = USART5_RX_STA & 0xFF7F;
        else
          rxlen_usart_5 = USART5_RX_STA;
				for(i5=0;i5<rxlen_usart_5;i5++)
				{
					FreeBuffer_Encode_5[i5] = USART5_RX_BUF[i5];					//将串口2接收到的数据传输给自由缓冲区
				}
        if(encodeDecode_Analysis_UWB(FreeBuffer_Encode_5,distance_to_station,rxlen_usart_5))
        {
          for(int i = 0; i < 4; ++i)
            distance_to_station_esm[i] = kalman_calc(&kalman_d[i], distance_to_station[i]);
          calculate_location(distance_to_station_esm, location);
          // printf("raw_x:%f raw_y:%f\r\n", location[0], location[1]);
          // mid_filter(location[0], location_esm, x_array);
          // mid_filter(location[1], location_esm + 1, y_array);
          // // printf("mid_x:%f mid_y:%f\r\n", location_esm[0], location_esm[1]);
          // limit_filter(location_esm[0], location_esm_limit);
          // limit_filter(location_esm[1], location_esm_limit + 1);

          // location_esm_kalma[0] = kalman_calc(&kalman_x, location_esm_limit[0]);
          // location_esm_kalma[1] = kalman_calc(&kalman_y, location_esm_limit[1]);
          // printf("kal_x:%f kal_y:%f\r\n", kalman_calc(&kalman_x, location[0]), kalman_calc(&kalman_y, location[1]));
          printf("kal_x:%f kal_y:%f\r\n", location[0], location[1]);

          calculate_cxof(location, d_location);
          Pack_cxof_buf(d_location[0], d_location[1], 100, cxof_buf);
          Send_cxof_buf(UART5, cxof_buf, 9);
        }
				rxlen_usart_5=0;
				USART5_RX_STA=0;
				BSP_USART_StartIT_LL(UART5); //启动下一次接收
			}
      //处理激光雷达传来的数据
      if(Receive_ok == 1)
      {
        for(sum=0,i=0;i<(re_buf_Data[3] + 4);i++)
        {
          sum += re_buf_Data[i];
        }
        if(sum == re_buf_Data[i])
        {
          distance = re_buf_Data[4]<<8 | re_buf_Data[5];
          RangeStatus = (re_buf_Data[6]>>4)&0x0f;
          Time = (re_buf_Data[6]>>2)&0x03;
          Mode = re_buf_Data[6] & 0x03;
          //printf("distance = %d, Mode = %d", distance, Mode);
        }
        Receive_ok = 0;
        BSP_USART_StartIT_LL(USART3);
      }
			
			//执行指令的当作
			if(2==RC_Read())//读取是否直通
			{ 
        if(mode_flag == 0) mode_flag = 1;
#ifdef TAKEOFF
        if(abs(distance - 1500) > 100 && is_takeoff==1)
        {
          Take_off(1500, distance);
          takeoff_Time = HAL_GetTick();
        }
        else if(abs(distance - 1500) <= 100 && is_takeoff==1)
        {
          is_takeoff = ((HAL_GetTick() - takeoff_Time) > 1000)?0:1;
          if(is_takeoff == 0) Set_PWM_Thr(4500);
        }
#endif
				//更新状态 与 时间控制分开执行
        ContriGetDataTime = HAL_GetTick() - ContriGetDataStart;
        if(ContriGetDataTime >= 80)
        {
          ContriGetDataStart = HAL_GetTick();
          Loiter(Attitude.Position_x,Attitude.Position_y,112,112, 0.0, 0.0);    //224x224
          pwm_pitch_time = PID_GetTime(&PID_Pitch_Time, Attitude.Position_y, 112);
          pwm_roll_time  = PID_GetTime(&PID_Roll_Time, Attitude.Position_x, 112);
					printf("pitch_time:%d, roll_time:%d \r\n", pwm_pitch_time, pwm_roll_time);
          
        }

        ContrlTime_x = HAL_GetTick() - ctrlstart_x;
        ContrlTime_y = HAL_GetTick() - ctrlstart_y;
        //Set_PWM_Thr(4500);
		    // if(ContrlTime_y>=680)
		    // {
		    // 	ctrlstart_y = HAL_GetTick();
		    // }
        // else if(ContrlTime_y <= pwm_pitch_time)
        // {
        //   //Set_PWM_Roll(pwm_roll_out);
        //   Set_PWM_Pitch(pwm_pitch_out);
        // }
        // else
        // {
        //   Set_PWM_Pitch(4500);
        // }

        // if(ContrlTime_x>=680)
        // {
        //   ctrlstart_x = HAL_GetTick();
        // }
        // else if(ContrlTime_x <= pwm_roll_time)
        // {
        //   Set_PWM_Roll(pwm_roll_out);
        // }
        // else
        // {
        //   Set_PWM_Roll(4500);
        // }

			}
			else if(3==RC_Read())
			{
        if(mode_flag == 1)
        {
          Set_PWM_Thr(4500);
				  Set_PWM_Pitch(4500);
				  Set_PWM_Roll(4500);
				  Set_PWM_Yaw(4500);
          mode_flag = 0;
        }
				//printf("remote ctrl...\r\n");
				 //桥接模式，即遥控器控制
				RC_bridge();
			}
			else if(1==RC_Read())
			{
				//Set_PWM_Mode(6000);
				Set_PWM_Thr(4500);
				Set_PWM_Pitch(4500);
				Set_PWM_Roll(4500);
				Set_PWM_Yaw(4500);
				
			}
			//printf("channel3 pulsewidth = %d \r\n", CHANNEL_3_PULSE_WIDE);
			 //LED_G_Flash(); 
			//RC_bridge_Test(); 
		}
		//指示灯
		RunTime = HAL_GetTick() - tickstart;
		if(RunTime>1000)
		{
			//printf("RunTime>1000%d \r\n",RunTime); 
			tickstart = HAL_GetTick();
      //此处为调试所需添加的注释
			LED_G_Flash(); 
			HAL_Delay(10);
			// LED_R_Flash();
			// HAL_Delay(20);
			// LED_B_Flash(); 
		}
    //控制部分
    
  }
  /*******************************从该处开始为cubemx配置*****************/
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 54;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 8;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20808DD4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 60000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 60000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 60000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 3000;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 4000;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 5000;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 72-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 60000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 2000;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 2500;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 3000;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 72-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 3000;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 0;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 3000;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 3000;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  LL_USART_InitTypeDef UART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART5;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
  /**UART5 GPIO Configuration
  PB12   ------> UART5_RX
  PB13   ------> UART5_TX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12|LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_14;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* UART5 interrupt Init */
  NVIC_SetPriority(UART5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),7, 0));
  NVIC_EnableIRQ(UART5_IRQn);

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  UART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  UART_InitStruct.BaudRate = 115200;
  UART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  UART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  UART_InitStruct.Parity = LL_USART_PARITY_NONE;
  UART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  UART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  UART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(UART5, &UART_InitStruct);
  LL_USART_DisableFIFO(UART5);
  LL_USART_SetTXFIFOThreshold(UART5, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(UART5, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_ConfigAsyncMode(UART5);

  /* USER CODE BEGIN WKUPType UART5 */

  /* USER CODE END WKUPType UART5 */

  LL_USART_Enable(UART5);

  /* Polling UART5 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(UART5))) || (!(LL_USART_IsActiveFlag_REACK(UART5))))
  {
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
  /**USART1 GPIO Configuration
  PB14   ------> USART1_TX
  PB15   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_14|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),4, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART1);
  LL_USART_ConfigAsyncMode(USART1);

  /* USER CODE BEGIN WKUPType USART1 */

  /* USER CODE END WKUPType USART1 */

  LL_USART_Enable(USART1);

  /* Polling USART1 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  {
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);
  /**USART2 GPIO Configuration
  PD5   ------> USART2_TX
  PD6   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART2);
  LL_USART_ConfigAsyncMode(USART2);

  /* USER CODE BEGIN WKUPType USART2 */

  /* USER CODE END WKUPType USART2 */

  LL_USART_Enable(USART2);

  /* Polling USART2 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
  {
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);
  /**USART3 GPIO Configuration
  PD8   ------> USART3_TX
  PD9   ------> USART3_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USART3 interrupt Init */
  NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),6, 0));
  NVIC_EnableIRQ(USART3_IRQn);

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART3);
  LL_USART_ConfigAsyncMode(USART3);

  /* USER CODE BEGIN WKUPType USART3 */

  /* USER CODE END WKUPType USART3 */

  LL_USART_Enable(USART3);

  /* Polling USART3 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART3))) || (!(LL_USART_IsActiveFlag_REACK(USART3))))
  {
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_B_Pin|LED_G_Pin|LED_R_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_B_Pin LED_G_Pin LED_R_Pin */
  GPIO_InitStruct.Pin = LED_B_Pin|LED_G_Pin|LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY1_Pin KEY2_Pin KEY3_Pin KEY4_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin|KEY2_Pin|KEY3_Pin|KEY4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*
*********************************************************************************************************
*	函 数 名: PrintfHardInfo
*	功能说明: 打印硬件接线信息
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void PrintfHardInfo(void)
{
  printf("*****************************************************************\r\n");
	printf("  接线方法: \r\n");
  printf("  +5V       <------   5.0V      5V供电\r\n");
  printf("  GND       -------   GND       地\r\n");
	printf("  PB0       ------>   PWM3      与遥控器的pitch通道相连\r\n");
	printf("  PB1       ------>   PWM4      与遥控器的接收机的第五通道相连 \r\n");
	printf("  PA6       ------>   PWM1      与遥控器接收机油门相连 \r\n");
	printf("  PB7       ------>   PWM2      与遥控器的偏航通道(YAW)相连\r\n");
	printf("  PA0       ------>   ADC_CH1   模拟超声波高度测量\r\n");
	printf("  打印采集数据: \r\n");
	printf("*****************************************************************\r\n");
}



unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++){      
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}

//*********************OutPut_Data****************************************//
void OutPut_Data(void)
{
  int temp[4] = {0};
  unsigned int temp1[4] = {0};
  unsigned char databuf[10] = {0};
  unsigned char i;
  unsigned short CRC16 = 0;
  for(i=0;i<4;i++)
   {
    
    temp[i]  = (int16_t)OutData[i];
    temp1[i] = (int16_t)temp[i];
    
   }
   
  for(i=0;i<4;i++) 
  {
    databuf[i*2]   = (int8_t)(temp1[i]%256);
    databuf[i*2+1] = (int8_t)(temp1[i]/256);
  }
  
  CRC16 = CRC_CHECK(databuf,8);
  databuf[8] = CRC16%256;
  databuf[9] = CRC16/256;
  
  for(i=0;i<10;i++)
	{
		BSP_USART_SendArray_LL( USART1,databuf,sizeof(databuf));
		//HAL_UART_Transmit(&huart1,(uint8_t *)&databuf[i],1,10);       //串口发送
	}
}



//****************************Data_to_VisualScope***************************************//
void Data_to_VisualScope(void)
{	
		int16_t i;	
    for(i = 0; i < CH_NUM; i++)
    {
      OutData[i] = s_volt[i];
      OutPut_Data();
    }
}


/*
*********************************************************************************************************
*	函 数 名: AD转化函数
*	功能说明: 处理采样后的数据
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
float ADC_CvtVolt(void)
{
	float volt=0;
	uint8_t i;
	int16_t adc;
	for(i=0;i<CH_NUM;i++)
	{

		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,0xffff);

		ADC_ReadBuffer[i] = HAL_ADC_GetValue(&hadc1);
		s_dat[i] = ADC_ReadBuffer[i];
		adc = s_dat[i];
		s_volt[i] = (adc * 3.30f) / 4096;
	}	
	HAL_ADC_Stop(&hadc1);
	volt=s_volt[0]*voltage_ratio ; 
	return  volt;
}


/*
*********************************************************************************************************
*	函 数 名: AD7606_Mak
*	功能说明: 处理采样后的数据
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
float ADC_CvtVolt_and_filter(void)
{
	float ADC_read=0.0;
	ADC_read=filter_av(1);
	return  ADC_read;
}


//***********读取ADC****************//
float get_adc(char adc_id)
{
	float volt=0;
	int16_t adc;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,0xffff);

	adc=HAL_ADC_GetValue(&hadc1);
	volt = ((adc * 3.30f) / 4096)*voltage_ratio;
	HAL_ADC_Stop(&hadc1);
	HAL_Delay(1);	
	//printf("V=%2.2f V Data_after_Deal=%2.2f cm  H=%2.2f cm  (count0=%4.1d) pwm_duty_out=%2.2f pwm_out=%d control_val=%2.2d \r\n",volt_0/voltage_ratio,Data_after_Deal,volt_0,count0,pwm_duty_out,PWM_Out,control_val); 
	return  volt;
}

//********算数平均滤波法**************//
float filter_av(char filter_id)
{
	char count=0;
	float sum = 0.0;
	for ( count=0;count<N;count++)
	{
	 sum  = sum + get_adc(1);
	}
	return sum/N;
}

/*根据串口接收的数据获取4个UWB基站的距离*/
bool Get_UWB_distance(float distance[])
{
  if(get_uwb_ready)
  {
    encodeDecode_UWBpacket(uwb_buf, distance_to_station);   //送入解析函数，得到4个基站的距离
    get_uwb_ready = 0;                                      //清除uwb标志位，串口再次开始接收
    BSP_USART_StartIT_LL(UART5);                            //清除串口5的相关标志位&开启接收中断
    for(int i=0;i<4;i++) memset(uwb_buf[i], 0, 20);         //重置接收数组
    return true;
  }
  else return false;
}

/****************************串口接收中断回调*****************************/
void   USART_RxCallback(USART_TypeDef *huart)
{ 
	if(LL_USART_IsActiveFlag_RXNE(huart) && LL_USART_IsEnabledIT_RXNE(huart))
  { 
		//  ***********串口1中断**********************************
		if (huart == USART1)
		{
			printf("USART1 INT \n");
			uint8_t data = LL_USART_ReceiveData8(huart);
		}
	 //***********串口2中断*********************
		else if(huart == USART2)
		{
			uint8_t data = LL_USART_ReceiveData8(huart);
			//printf("%c",data);
			if(data == 0x23)    //根据自定义通信包格式
			{
				UART2_Frame_Flag = 1;
			}
      if(((USART2_RX_STA  & (1<<15))==0) && (UART2_Frame_Flag == 1))		//还可以接收数据 ,最高位不为1.
			{
				TIM15->CNT=0;				//计数器15清空
        if(USART2_RX_STA == 0)
				{
					TIM15_Set(1);	 	                //使能定时器15的中断
					Recv_Cnt_UART2 = 0;
				}
				USART2_RX_BUF[USART2_RX_STA++] = data;
				Recv_Cnt_UART2 ++;			
				if(Recv_Cnt_UART2>=11)
				{
					Recv_Cnt_UART2 = 0;
					UART2_Frame_Flag = 0;
					USART2_RX_STA |= 1<<15;					//强制标记接收完成
					LL_USART_DisableIT_RXNE(USART2);
				}
      }	
      
		}
    //*******************串口3中断****************
    else if(huart == USART3)
    {
      uint8_t data = LL_USART_ReceiveData8(huart);  //串口接收一个字节
      //printf("%c", data);
      static uint8_t usart3_index = 0, rebuf[20] = {0};
      rebuf[usart3_index++] = data;
      if(rebuf[0] != 0x5a) usart3_index = 0;    //帧头不对
      if(usart3_index==2 && rebuf[1]!=0x5a) usart3_index = 0; //帧头不对
      if(usart3_index > 3)    //表示已经收到byte3->数据量
      {
        if(usart3_index == rebuf[3] + 5){  //接收完一帧数据
          if(Receive_ok == 0){                    //当数据在main中处理后才接收新数据
            //printf("receive a full\r\n");
            memcpy(re_buf_Data, rebuf, 8);    //拷贝接收到的数据
            Receive_ok = 1;                   //标志已经接收完成
          }
          usart3_index = 0;   //清空接收缓存数量
          Receive_ok = 1;
          LL_USART_DisableIT_RXNE(USART3);
        }
      }
    }
    /**************串口5中断待添加，调试成功后将串口1(type-c)替换成串口5即可********/
		else if (huart == UART5)
		{
			uint8_t data = LL_USART_ReceiveData8(huart); //串口接收一个字节
			// printf("%c",data);
			if ((USART5_RX_STA & (1 << 7)) == 0)  //缓冲区还没满，继续接收数据。
			{
				// TIM12->CNT = 0;			    //定时器12清空
				if (USART5_RX_STA == 0) //新一轮接收开始
				{
					TIM12_Set(1);
				}
				USART5_RX_BUF[USART5_RX_STA++] = data; //存入接收缓冲区
				// printf("USART5 INT =%d \r\n",USART5_RX_STA);
			}
			else
			{
				USART5_RX_STA |= 1 << 7; //强制标记接收完成
        LL_USART_DisableIT_RXNE(USART2);
			}
		}
	}
}



//*******定时器中断服务程序	*************************************//
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	  static uint16_t tim12_1ms= 0;//中断次数计数器
		static uint16_t tim13_1ms= 0;//中断次数计数器
	  static uint16_t tim15_1ms= 0;//中断次数计数器
	  //static uint16_t tim17_1ms= 0;//中断次数计数器

		/*****定时器12中断服务函数->在串口1中使用到更新中断*********/
		if (htim->Instance == htim12.Instance) //是更新中断
		{
			tim12_1ms++;
			if(tim12_1ms==500)		    //100ms内无CNT清空，则停止接收数据
			{
        //printf("o");
				// USART5_RX_STA|= (1<<9);	//标记接收完成
				TIM12->SR&=~(1<<0);		//清除中断标志位		   
				TIM12_Set(0);			    //关闭TIM3
				tim12_1ms=0;
				//printf("TIME 4 INT \r\n");
			} 
		}
		//*****定时器13中断服务函数->用于串口3*********************
		if (htim->Instance == htim13.Instance) //是更新中断
		{
			tim13_1ms++;
			if(tim13_1ms==500)		    //20x2ms内无CNT清空，则停止接收数据
			{
				USART3_RX_STA |= (1<<7);	//标记接收完成
				TIM13->SR&=~(1<<0);		//清除中断标志位		   
				TIM13_Set(0);			    //关闭TIM5
				tim13_1ms=0;
				//printf("TIME 2 INT \r\n");
			} 
		}
		//*****定时器15中断服务函数->用于串口2*********************
		if (htim->Instance == htim15.Instance) //是更新中断
		{
			tim15_1ms++;
			if(tim15_1ms==8)		    //40ms内无CNT清空，则停止接收数据
			{
				USART2_RX_STA |= (1<<15);	//标记接收完成
				TIM15->SR&=~(1<<0);		//清除中断标志位		   
				TIM15_Set(0);			    //关闭TIM5
				tim15_1ms=0;
				//printf("TIME 5 INT \r\n");
			} 
		}
		// //*****定时器17中断服务函数->用于串口2*********************
		// if (htim->Instance == htim17.Instance) //是更新中断
		// {
		// 	tim17_1ms++;
		// 	if(tim17_1ms==200)		    //200ms内无CNT清空，则停止接收数据
		// 	{
		// 		USART5_RX_STA |= (1<<15);	//标记接收完成
		// 		TIM17->SR&=~(1<<0);		//清除中断标志位		   
		// 		TIM17_Set(0);			    //关闭TIM5
		// 		tim17_1ms=0;
		// 		//printf("TIME 17 INT \r\n");
		// 	} 
		// }
}

//定时器12
void TIM12_Set(uint8_t sta)
{
	if(sta)
	{
		TIM12->CNT=0;                   //计数器清空
		HAL_TIM_Base_Start_IT(&htim12); //使能定时器12
	}else 
		HAL_TIM_Base_Stop_IT(&htim12);  //关闭定时器12
}

//定时器13
void TIM13_Set(uint8_t sta)
{
	if(sta)
	{
		TIM13->CNT=0;                   //计数器清空
		HAL_TIM_Base_Start_IT(&htim13); //使能定时器13
	}else 
		HAL_TIM_Base_Stop_IT(&htim13);  //关闭定时器13
}

//定时器15
void TIM15_Set(uint8_t sta)
{
	if(sta)
	{
		TIM15->CNT=0;                   //计数器清空
		HAL_TIM_Base_Start_IT(&htim15); //使能定时器15
	}else 
		HAL_TIM_Base_Stop_IT(&htim15);  //关闭定时器15
}

// //定时器17
// void TIM17_Set(uint8_t sta)
// {
// 	if(sta)
// 	{
// 		TIM17->CNT=0;                   //计数器清空
// 		HAL_TIM_Base_Start_IT(&htim17); //使能定时器17
// 	}else 
// 		HAL_TIM_Base_Stop_IT(&htim17);  //关闭定时器17
// }


//定时器3
//********************************
//采用定时器轮询的方式实现延时。
//		for(int q=0;q<1000;q++)
//		{
//				Delay_us(1000);
//		}
//调用示例
//********************************

// 定时器输入捕获PWM
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	//printf("TIM_IC \r\n");
  TIM_IC_InitTypeDef sConfigIC;
	//定时器2输入捕获
	if(htim->Instance == htim2.Instance)
	{
		switch(htim->Channel)
		{
      /*****************通道1********************************************/
			case HAL_TIM_ACTIVE_CHANNEL_1:    //AUX2_In
				if(ICFLAG_1){
					CHANNEL_1_RISE = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_1);
					
					sConfigIC.ICPolarity  = TIM_INPUTCHANNELPOLARITY_FALLING;//下降沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim2,TIM_CHANNEL_1);
					HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1);
					
					__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC1);
					if(HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1)!= HAL_OK)
					{
						printf("ERROR\r\n");
					}
					
					ICFLAG_1 = 0;
				}
				else{
					CHANNEL_1_FALL = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_1);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;//上升沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim2,TIM_CHANNEL_1);
					HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1);
					
					__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC1);
					if(HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1)!= HAL_OK)
					{
						printf("ERROR\r\n");
					}

					
					CHANNEL_1_PULSE_WIDE = (CHANNEL_1_FALL > CHANNEL_1_RISE ? (CHANNEL_1_FALL - CHANNEL_1_RISE):(CHANNEL_1_FALL - CHANNEL_1_RISE + 60000));
          printf("AUX2_Input:%d\r\n",CHANNEL_1_PULSE_WIDE);
					ICFLAG_1 = 1;
          //__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, CHANNEL_1_PULSE_WIDE);
				}
				//CH1_PWM_test(CHANNEL_1_PULSE_WIDE);
//				printf("channel1 pulsewidth = %d \r\n", CHANNEL_1_PULSE_WIDE);
				break;
	//	 /***************通道2**********************************************************/   
			case HAL_TIM_ACTIVE_CHANNEL_2:    //AUX1_In
					if(ICFLAG_2){
					CHANNEL_2_RISE = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_2);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;//下降沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim2,TIM_CHANNEL_2);
					HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2);
					
					__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC2);
					if(HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2)!= HAL_OK)
					{
						printf("ERROR\r\n");
					}  
					
					ICFLAG_2 = 0;
				}
				else{
					CHANNEL_2_FALL = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_2);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;//上升沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim2,TIM_CHANNEL_2);
					HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2);
					
					__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC2);
					if(HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2)!= HAL_OK)
					{
						printf("ERROR\r\n");
					} 
					
					CHANNEL_2_PULSE_WIDE = (CHANNEL_2_FALL > CHANNEL_2_RISE ? CHANNEL_2_FALL - CHANNEL_2_RISE:CHANNEL_2_FALL - CHANNEL_2_RISE + 60000);
					ICFLAG_2 = 1;
          //printf("AUX1_Input:%d\r\n",CHANNEL_2_PULSE_WIDE);
          //__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, CHANNEL_2_PULSE_WIDE);
				}
				//CH2_PWM_test(CHANNEL_2_PULSE_WIDE);
//				printf("channel2 pulsewidth = %d \r\n", CHANNEL_2_PULSE_WIDE);
				break;
	//	/***************通道3***********************************************************/    
			case HAL_TIM_ACTIVE_CHANNEL_3:    //Ctrl_In
				if(ICFLAG_3){
					CHANNEL_3_RISE = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_3);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;//下降沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim2,TIM_CHANNEL_3);
					HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3);
					
					__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC3);
					if(HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_3)!= HAL_OK)
					{
						printf("ERROR\r\n");
					}  
					
					ICFLAG_3 = 0;
				}
				else{
					CHANNEL_3_FALL = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_3);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;//上升沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim2,TIM_CHANNEL_3);
					HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3);
					
					__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC3);
					if(HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_3)!= HAL_OK)
					{
						printf("ERROR\r\n");
					}  
					
					CHANNEL_3_PULSE_WIDE = (CHANNEL_3_FALL > CHANNEL_3_RISE ? CHANNEL_3_FALL - CHANNEL_3_RISE:CHANNEL_3_FALL - CHANNEL_3_RISE + 60000);
					ICFLAG_3 = 1;
          //__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, CHANNEL_3_PULSE_WIDE);      
				}
				//CH3_PWM_test(CHANNEL_3_PULSE_WIDE);
//				printf("channel3 pulsewidth = %d \r\n", CHANNEL_3_PULSE_WIDE);
				break;
	//			/*************通道4*************************************************************/    
			case HAL_TIM_ACTIVE_CHANNEL_4:    //Mode_In
				if(ICFLAG_4){
					CHANNEL_4_RISE = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_4);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;//下降沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim2,TIM_CHANNEL_4);
					HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4);
					
					__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC4);
					if(HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_4)!= HAL_OK)
					{
						printf("ERROR\r\n");
					}    
					
					ICFLAG_4 = 0;
				}
				else{
					CHANNEL_4_FALL = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_4);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;//上升沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim2,TIM_CHANNEL_4);
					////////////////同上
					HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4);
					
					__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC4);
					if(HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_4)!= HAL_OK)
					{
						printf("ERROR\r\n");
					}  
					
					CHANNEL_4_PULSE_WIDE = (CHANNEL_4_FALL > CHANNEL_4_RISE ? CHANNEL_4_FALL - CHANNEL_4_RISE:CHANNEL_4_FALL - CHANNEL_4_RISE + 60000);
					ICFLAG_4 = 1;
          //__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, CHANNEL_4_PULSE_WIDE);
				}
				//CH4_PWM_test(CHANNEL_3_PULSE_WIDE);
//				printf("channel4 pulsewidth = %d \r\n", CHANNEL_4_PULSE_WIDE);
				break;
			default: break;
		}
	}
	
	//****************************定时器3捕获****************************************
	if(htim->Instance == htim3.Instance)
	{
		switch(htim->Channel)
		{
			case HAL_TIM_ACTIVE_CHANNEL_1:    //Yaw_In
				if(ICFLAG_5){
					CHANNEL_5_RISE = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;//下降沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_1); 
					HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1);
					
					__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC1);
					HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);  
					
					ICFLAG_5 = 0;
				}
				else{
					CHANNEL_5_FALL = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;//上升沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_1); 
					HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1);
					
				__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC1);
					HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1); 
					
					CHANNEL_5_PULSE_WIDE = (CHANNEL_5_FALL > CHANNEL_5_RISE ? CHANNEL_5_FALL - CHANNEL_5_RISE:CHANNEL_5_FALL - CHANNEL_5_RISE + 60000);
					ICFLAG_5 = 1;
          //__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, CHANNEL_5_PULSE_WIDE);
				}
				//CH1_PWM_test(CHANNEL_1_PULSE_WIDE);
//				printf("channel5 pulsewidth = %d \r\n", CHANNEL_5_PULSE_WIDE);
				break;
			case HAL_TIM_ACTIVE_CHANNEL_2:    //Roll_In
				if(ICFLAG_6){
					CHANNEL_6_RISE = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_2);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;//下降沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_2); 
					HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2);
					
					__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC2);
					HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);  
					
					ICFLAG_6 = 0;
				}
				else{
					CHANNEL_6_FALL = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_2);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;//上升沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_2); 
					HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2);
					
				__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC2);
					HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2); 
					
					CHANNEL_6_PULSE_WIDE = (CHANNEL_6_FALL > CHANNEL_6_RISE ? CHANNEL_6_FALL - CHANNEL_6_RISE:CHANNEL_6_FALL - CHANNEL_6_RISE + 60000);
					ICFLAG_6 = 1;
          //printf("roll pulsewidth = %d \r\n", CHANNEL_6_PULSE_WIDE);
          //__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, CHANNEL_6_PULSE_WIDE);
				}
				//CH1_PWM_test(CHANNEL_1_PULSE_WIDE);
//				printf("channel5 pulsewidth = %d \r\n", CHANNEL_5_PULSE_WIDE);
				break;
			case HAL_TIM_ACTIVE_CHANNEL_3:    //Pitch_In
				if(ICFLAG_7){
					CHANNEL_7_RISE = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_3);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;//下降沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_3); 
					HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3);
					
					__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC3);
					HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_3);  
					
					ICFLAG_7 = 0;
				}
				else{
					CHANNEL_7_FALL = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_3);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;//上升沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_3); 
					HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3);
					
				__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC3);
					HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_3); 
					
					CHANNEL_7_PULSE_WIDE = (CHANNEL_7_FALL > CHANNEL_7_RISE ? CHANNEL_7_FALL - CHANNEL_7_RISE:CHANNEL_7_FALL - CHANNEL_7_RISE + 60000);
					ICFLAG_7 = 1;
          //__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, CHANNEL_7_PULSE_WIDE);
				}
				//CH1_PWM_test(CHANNEL_1_PULSE_WIDE);
//				printf("channel5 pulsewidth = %d \r\n", CHANNEL_5_PULSE_WIDE);
				break;
		 /*************************************************************************/   
			case HAL_TIM_ACTIVE_CHANNEL_4:    //Thr_In
					if(ICFLAG_8){
					CHANNEL_8_RISE = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_4);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;//下降沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_4);
					HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4);
					
					__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC4);
					HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_4);  
					
					ICFLAG_8 = 0;
				}
				else{
					CHANNEL_8_FALL = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_4);
					
					sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;//上升沿
					sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
					sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
					sConfigIC.ICFilter = 0;
					HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_4);
					HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4);
					
					__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC4);
					HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_4); 
					
					CHANNEL_8_PULSE_WIDE = (CHANNEL_8_FALL > CHANNEL_8_RISE ? CHANNEL_8_FALL - CHANNEL_8_RISE:CHANNEL_8_FALL - CHANNEL_8_RISE + 60000);
					ICFLAG_8 = 1;
          //__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, CHANNEL_8_PULSE_WIDE);
				}
				//CH2_PWM_test(CHANNEL_2_PULSE_WIDE);
				//printf("channel8_thr pulsewidth = %d \r\n", CHANNEL_8_PULSE_WIDE);
				break;
			default: break;
		}
	}
}


//手动发送mavlink信号
void MANUAL_CONTROL_Send(int16_t xpoint,int16_t ypoint)
{
	uint8_t system_id = 255;          // 发送本条消息帧的设备的系统编号（sys）   
	uint8_t component_id = 0;         // 发送本条消息帧的设备的单元编号（comp）
	uint8_t target = 0x01;            //目标系统
	int16_t x=0;
	int16_t y=ypoint;
	int16_t z=0; 
	int16_t r=0; 
	uint16_t buttons=0;
	mavlink_message_t msg;             // msg The MAVLink message to compress the data into
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;	

	mavlink_msg_manual_control_pack(system_id, component_id, &msg, target,x,y,z,r,buttons);
	len = mavlink_msg_to_send_buffer(buf, &msg);        
	//UART_Send_Str(buf,len);	
	BSP_USART_SendArray_LL( USART1,buf,sizeof(buf));
}



void RC_CHANNELS_OVERRIDE_Send(int16_t xpoint,int16_t ypoint)
{
	uint8_t system_id=255;
	uint8_t component_id=0;
	mavlink_message_t msg; 
	uint8_t target_system=1;
	uint8_t target_component=0;
	uint16_t chan1_raw=ypoint;
	uint16_t chan2_raw=65535;
	uint16_t chan3_raw=65535;
	uint16_t chan4_raw=65535; 
	uint16_t chan5_raw=65535;
	uint16_t chan6_raw=65535;
	uint16_t chan7_raw=65535;
	uint16_t chan8_raw=65535;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;	

	mavlink_msg_rc_channels_override_pack(system_id,component_id,&msg,target_system,target_component,
		chan1_raw,chan2_raw,chan3_raw,chan4_raw,chan5_raw,chan6_raw,chan7_raw,chan8_raw);
	len = mavlink_msg_to_send_buffer(buf, &msg); 
	BSP_USART_SendArray_LL( USART1,buf,sizeof(buf));
	//UART_Send_Str(buf,len);	


	mavlink_msg_rc_channels_override_pack(system_id,component_id,&msg,target_system,target_component,
		0,chan2_raw,chan3_raw,chan4_raw,chan5_raw,chan6_raw,chan7_raw,chan8_raw);

	len = mavlink_msg_to_send_buffer(buf, &msg); 
	//UART_Send_Str(buf,len);	
	BSP_USART_SendArray_LL( USART1,buf,sizeof(buf));
}


//*****心跳信号*************************
void heartbeat_Mavlink(void)
{
	uint8_t system_id=255;
	uint8_t component_id=0;
	mavlink_message_t heart_msg;
	uint8_t type=0x06;
	uint8_t autopilot=0x08;
	uint8_t base_mode=0xc0;
	uint32_t custom_mode=0x0000; 
	uint8_t system_status=0x04;

	uint8_t buf_head[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;

	mavlink_msg_heartbeat_pack( system_id,component_id, &heart_msg,type, autopilot,base_mode,custom_mode,system_status);
	len = mavlink_msg_to_send_buffer(buf_head, &heart_msg);
	//UART_Send_Str(buf_head,len);
	BSP_USART_SendArray_LL( USART1,buf_head,sizeof(buf_head));
}



//读取工作模式，即读取Ctrl_In的脉宽
int RC_Read(void)
{
	// //3000~4000
	// if(CHANNEL_3_PULSE_WIDE>=PWM_Mode_N1 && CHANNEL_3_PULSE_WIDE<=PWM_Mode_N2)
	// {
		
	//   return 1;
	// }
	//4000~5000
  if(CHANNEL_3_PULSE_WIDE>=PWM_Mode_N1 && CHANNEL_3_PULSE_WIDE<=PWM_Mode_N2)
	{
	  return 2;
	}
	//5000~6000
	else if(CHANNEL_3_PULSE_WIDE>=PWM_Mode_N3 && CHANNEL_3_PULSE_WIDE<=PWM_Mode_N4)
	{
	  return 3;
	}
	else return 1;
}


//各通道回中

void Back_to_Center(void)
{
	Set_PWM_Thr(4500);
	Set_PWM_Pitch(4500);
	Set_PWM_Roll(4500);
	Set_PWM_Yaw(4500);
	//Set_PWM_Mode(4500);
}
/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
