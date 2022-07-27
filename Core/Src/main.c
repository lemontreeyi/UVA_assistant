/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "arm_math.h"
#include <stdarg.h>
#include <string.h>
#include "mavlink.h"
#include "mavlink_types.h"

#include "led.h"
#include "key.h"
#include "uart_api.h"
#include "beep.h"
#include "coderDecoder.h"
#include "flyControl.h"
#include "MAV_Altitude_Decode.h"
#include "mpu6050.h"
#include "imu.h"
#include "Matrix.h"
#include "Kalman.h"
#include "patrol.h"
#include "vl53l1_api.h"
#include "vl53l1_platform.h"
/* 定义例程名和例程发布日期 */
#define EXAMPLE_NAME "V4梦创飞控室内自动飞行例程"
#define EXAMPLE_DATE "2021-04-7 "
#define DEMO_VER "V-1.0"

#define Flag_SendToUsart 1
#define A 60
#define CH_NUM 1 /* 8通道 */
#define N 3
#define voltage_ratio 204 // 2.04

#define USART1_MAX_RECV_LEN 256 //�?大接收缓存字节数
#define USART2_MAX_RECV_LEN 256
#define USART3_MAX_RECV_LEN 256
#define UART5_MAX_RECV_LEN 256

int CHANNEL_1_RISE = 0, CHANNEL_1_FALL = 0, CHANNEL_1_PULSE_WIDE = 0; //辅助通道2
int CHANNEL_2_RISE = 0, CHANNEL_2_FALL = 0, CHANNEL_2_PULSE_WIDE = 0; //辅助通道1
int CHANNEL_3_RISE = 0, CHANNEL_3_FALL = 0, CHANNEL_3_PULSE_WIDE = 0; // Ctrl通道
int CHANNEL_4_RISE = 0, CHANNEL_4_FALL = 0, CHANNEL_4_PULSE_WIDE = 0; // Mode通道
int CHANNEL_5_RISE = 0, CHANNEL_5_FALL = 0, CHANNEL_5_PULSE_WIDE = 0; // Yaw通道
int CHANNEL_6_RISE = 0, CHANNEL_6_FALL = 0, CHANNEL_6_PULSE_WIDE = 0; // Roll通道
int CHANNEL_7_RISE = 0, CHANNEL_7_FALL = 0, CHANNEL_7_PULSE_WIDE = 0; // Pitch通道
int CHANNEL_8_RISE = 0, CHANNEL_8_FALL = 0, CHANNEL_8_PULSE_WIDE = 0; // Thr通道

int ICFLAG_1 = 1, ICFLAG_2 = 1, ICFLAG_3 = 1, ICFLAG_4 = 1, ICFLAG_5 = 1, ICFLAG_6 = 1, ICFLAG_7 = 1, ICFLAG_8 = 1;

//控制直�?�或桥接模式
int PWM_Ctrl_N1 = 2500;
int PWM_Ctrl_N2 = 4500;
int PWM_Ctrl_N3 = 5000;
int PWM_Ctrl_N4 = 6500;

uint16_t USART1_RX_STA = 0;
uint16_t USART2_RX_STA = 0;
uint16_t USART3_RX_STA = 0;
uint16_t UART5_RX_STA = 0;

//static int Recv_Cnt_UART1 = 0;
static int Recv_Cnt_UART2 = 0;
//static int Recv_Cnt_UART3 = 0;
//static int Recv_Cnt_UART5 = 0;
//static int UART1_Frame_Flag = 0;
static int UART2_Frame_Flag = 0;
//static int UART3_Frame_Flag = 0;
//static int UART5_Frame_Flag = 0;

static int heartbeat = 0;
// static int MAVLink_message_length = 0;
// static mavlink_distance_sensor_t packet;

uint8_t USART1_RX_BUF[USART1_MAX_RECV_LEN];
uint8_t USART2_RX_BUF[USART2_MAX_RECV_LEN];
uint8_t USART3_RX_BUF[USART3_MAX_RECV_LEN];
uint8_t UART5_RX_BUF[UART5_MAX_RECV_LEN];

uint8_t FreeBuffer_Encode[USART2_MAX_RECV_LEN];
uint8_t FreeBuffer_Encode_3[USART3_MAX_RECV_LEN];
uint8_t FreeBuffer_Encode_5[UART5_MAX_RECV_LEN];
uint8_t MAVLink_RECV_BUF[USART2_MAX_RECV_LEN];
uint8_t MAVLink_TX_BUF[MAVLINK_MAX_PACKET_LEN];
uint8_t MAVLink_RECV_BUF_FAKE[USART2_MAX_RECV_LEN] = {0};

uint8_t encodeAnswer[11] = {'#', '1', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, '*'};

float old_value;

static 	int16_t s_dat[CH_NUM];
float 	s_volt[CH_NUM];

uint32_t 	ADC_ReadBuffer[CH_NUM];
float 		OutData_Test[4];
float 		OutData[4];
char 			menu = 0;

PID PID_Control_Att; //  PID Control Structure

void  OutPut_Data(void);
float ADC_CvtVolt(void);
float DataProcessing(float IN_Data);
float ADC_CvtVolt_and_filter(void);
float get_adc(char adc_id);
float filter_av(char filter_id);
float filter(float new_value);

void Data_to_VisualScope(void);
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);

// mavlink
void MANUAL_CONTROL_Send(int16_t xpoint, int16_t ypoint);
void RC_CHANNELS_OVERRIDE_Send(int16_t xpoint, int16_t ypoint);
void heartbeat_Mavlink(void);
int  RC_Read(void);
void Back_to_Center(void);
void RC_Week_Bridge(void);

//vl53L1x
int32_t height;
VL53L1_Dev_t Dev;
VL53L1_RangingMeasurementData_t result_data;
VL53L1_CalibrationData_t save;
VL53L1_Error vl53l1x_init(VL53L1_DEV pDev);
VL53L1_Error vl53l1x_Cali(VL53L1_DEV pDev, VL53L1_CalibrationData_t* save);
VL53L1_Error vl53l1x_GetDistance(VL53L1_DEV pDev);

void end(void)
{
	printf("%c", 0xff);
	printf("%c", 0xff);
	printf("%c", 0xff);
}

//UWB to optical_flow
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
int   target_location[2] = {0, 0};	//cm为单�?
int   next_location[2] = {0, 0};
short d_location[2] = {0 ,0};
float speed[2] = {0.0, 0.0};
bool  patrol_flag = 0;

float height_esm = 0;

int task = 0;
bool is_takeoff = 1;
bool is_settarget = 0;
bool is_SetStartPoint = 0;

//校准坐标数据
float kx = 1.0078536923765482;
float dx = -0.091023393141628;
float ky = 0.9807397784555684;
float dy = 0.07220428891557518;

//bool Get_UWB_distance(float distance[]);

/* 仅允许本文件内调用的函数声明 */
//static void PrintfLogo(void);
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
	BSP_USART_SendData_LL(USART1, ch);
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
	uint16_t i1 = 0, i2 = 0, i3 = 0, i5 = 0;
	uint16_t len = 0;
	uint16_t rxlen_usart_1;
	uint16_t rxlen_usart_2;
	uint16_t rxlen_usart_3;
	uint16_t rxlen_uart_5;
	
	uint8_t cmd = 0;
	uint8_t key;
	uint32_t RunTime = 0;
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
	uint32_t Cxof_Time = HAL_GetTick();
	uint32_t Cxof_Wait = HAL_GetTick();
	uint32_t Dtime = 0;
	int flag_rx2 = 0;
//	mavlink_message_t msg1;
//	mavlink_message_t msg2;
//	mavlink_message_t msg_tmp;
//	mavlink_message_t msg_altitude;
//	mavlink_message_t msg_request_data_stream;
//	uint32_t len = 0;
  uint8_t str[] = "hello world\r\n";

  /* USER CODE END 1 */

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM11_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_TIM10_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	Device_Init();
	PrintfHardInfo(); /* 打印硬件接线信息 */
	KEY_Init();

	/*
		 由于ST固件库的启动文件已经执行了CPU系统时钟的初始化，所以不必再次重复配置系统时钟�??
	启动文件配置了CPU主时钟频率，内部Flash访问速度和可选的外部SRAM FSMC初始化�??
	*/
	// tag: ADC init
	//printf("ADC init\n");
	HAL_ADC_Init(&hadc1);
	HAL_ADC_Start(&hadc1);

	// tag:PWM
	printf("PWM init\n");
	HAL_TIM_PWM_Init(&htim4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	//
	HAL_TIM_PWM_Init(&htim5);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

	// 使能定时器输入捕�?
	printf("TIM 2 and 3 init\n");
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);

	// tag: pid init
	printf("PID init\n");
	PIDInit(&PID_Control_Att);
	LED1_Slow_Flash();
	BEEP_ON();
	HAL_Delay(300);
	BEEP_OFF();
	HAL_Delay(100);

	// tag: usart start
	printf("USART Start\n");
	BSP_USART_StartIT_LL(USART1);
	BSP_USART_StartIT_LL(USART2);
	BSP_USART_StartIT_LL(USART3);
	BSP_USART_StartIT_LL(UART5);
	
	//kalman init
	kalman_init(&kalman_x, 0.4, 0.5);
  	kalman_init(&kalman_y, 0.4, 0.5);
	kalman_init(&kalman_h, 0.3, 0.0016);
  	for(int i = 0; i < 4; ++i)
  		kalman_init(kalman_d+i, 0.3, 0.01);
  	init_A_matrix();
	
	//vl53L1x init
	VL53L1_Error Status = VL53L1_ERROR_NONE;
	Status = vl53l1x_init(&Dev);
	if(Status != VL53L1_ERROR_NONE)
	{
		printf("%d\r\n", Status);
	}

	//Attitude init
	Attitude_init(&Attitude);

	//patrol init
	//init_flypath();

	TIM11_Set(0);
	TIM13_Set(0);
	TIM14_Set(0);

	BEEP_ON();
	HAL_Delay(300);
	BEEP_OFF();
	
	// tag: init finish
	printf("init finish\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Set_PWM_Ctrl(CHANNEL_3_PULSE_WIDE);		//便于在地面站观察直�?�或桥接模式
		//读取�?光雷达测距数�?
		vl53l1x_GetDistance(&Dev);
		//读取MPU6050
		float inner_loop_time = 0.001;
		float outer_loop_time = 0.001;
		MPU6050_Read();						   //读取mpu6轴传感器
		MPU6050_Data_Prepare(inner_loop_time); // mpu6轴传感器数据处理
		/*IMU更新姿�?�输入：半个执行周期，三轴陀螺仪数据（转换到度每秒），三轴加速度计数据（4096--1G）；输出：ROLPITYAW姿�?�角*/
		IMUupdate(0.5f * outer_loop_time, mpu6050.Gyro_deg.x, mpu6050.Gyro_deg.y, mpu6050.Gyro_deg.z, mpu6050.Acc.x, mpu6050.Acc.y, mpu6050.Acc.z, &Roll, &Pitch, &Yaw);

		key = KeyScanning(0); //按键扫描
		switch (key)
		{
		case KEY1_PRES: //控制LED0，LED1互斥点亮
			LED_B_Flash();
			BEEP_ON();
			Unlock();
			break;
		case KEY2_PRES: 		//给v831发�?�指令进行扫�?
			LED1_Slow_Flash();
			Pack_cmd_buf(1, cmd_buf);
			BSP_USART_SendArray_LL(USART2, cmd_buf, 3);
			break;
		}
		
		if (InitedFlag) //确保设备已初始化
		{
			/********************************UART1接收并处理数�?**********************************/
			if (USART1_RX_STA & 0X8000) //接收到一次数据了
			{
				// printf("USART1 INT =%d \r\n",USART1_RX_STA);
				rxlen_usart_1 = USART1_RX_STA & 0x7FFF; //得到数据长度
				for (i1 = 0; i1 < rxlen_usart_1; i1++)
				{
					FreeBuffer_Encode[i1] = USART1_RX_BUF[i1]; //将串�?1接收到的数据传输给自由缓冲区
															   // BSP_USART_SendArray_LL( USART1,&FreeBuffer_Encode[i1],1);
				}
				cmd = encodeDecode_Analysis(FreeBuffer_Encode, encodeAnswer, rxlen_usart_1); //分析字符�?
				BSP_USART_StartIT_LL(USART1);
				rxlen_usart_1 = 0;
				USART1_RX_STA = 0; //启动下一次接�?
			}
			//********************************UART2接收并处理数�?***********************************/
			if (USART2_RX_STA & 0X8000) //接收到一次数据，且超过了预设长度
			{
        	//printf("USART2 revd ...\r\n");
				rxlen_usart_2 = USART2_RX_STA & 0x7FFF;	//得到数据长度
				for(i2=0;i2<rxlen_usart_2;i2++)
				{
					FreeBuffer_Encode[i2] = USART2_RX_BUF[i2];	//将串�?2接收到的数据传输给自由缓冲区
				}
        		if(rxlen_usart_2 == 27) {
					cmd = encodeDecode_Analysis(FreeBuffer_Encode,encodeAnswer,rxlen_usart_2);
        		} //分析字符
					rxlen_usart_2=0;
					USART2_RX_STA=0;
					BSP_USART_StartIT_LL( USART2 ); //启动下一次接�?
			}
			/*****************************USART3接收&处理数据***********************************/
			if (USART3_RX_STA & 0x8000) //接收满一次数�?
			{
				
			}
			/*****************************USART5接收&处理数据***********************************/
			if (UART5_RX_STA & 0x0080) //接收满一次数�?
			{
				if(UART5_RX_STA > 128)
        		rxlen_uart_5 = UART5_RX_STA & 0xFF7F;
        		else
          			rxlen_uart_5 = UART5_RX_STA;
				for(i5=0;i5<rxlen_uart_5;i5++)
				{
					FreeBuffer_Encode_5[i5] = UART5_RX_BUF[i5];					
					//将串�?5接收到的数据传输给自由缓冲区
				}
				if(encodeDecode_Analysis_UWB(FreeBuffer_Encode_5,distance_to_station,rxlen_uart_5))
        		{
					// printf("dis_real1=%f, dis_real2=%f, dis_real3=%f, dis_real4=%f\r\n", distance_to_station[0],distance_to_station[1],distance_to_station[2],distance_to_station[3]);
          			for(int i = 0; i < 4; ++i)
            			distance_to_station_esm[i] = kalman_calc(&kalman_d[i], distance_to_station[i]);
					height_esm = kalman_calc(&kalman_h, height / 1000.0);
					//printf("dis1=%f, dis2=%f\r\n dis3=%f, dis4=%f\r\n", distance_to_station_esm[0],distance_to_station_esm[1],distance_to_station_esm[2],distance_to_station_esm[3]);
          			//printf("raw_d1 %f raw_d2 %f raw_d3 %f raw_d4 %f kal_d1 %f kal_d2 %f kal_d3 %f kal_d4 %f\r\n", distance_to_station[0], distance_to_station[1], distance_to_station[2], distance_to_station[3], distance_to_station_esm[0], distance_to_station_esm[1], distance_to_station_esm[2], distance_to_station_esm[3]);
					calculate_location(distance_to_station_esm, location, height/1000.0);
					// location[0] = kx * location[0] + dx;
					// location[1] = ky * location[1] + dy;
					location_esm[0] = kalman_calc(&kalman_x, location[0]);
					location_esm[1] = kalman_calc(&kalman_y, location[1]);
          			//printf("kal_x:%f kal_y:%f\r\n", kalman_calc(&kalman_x, location[0]), kalman_calc(&kalman_y, location[1]));
					Dtime = HAL_GetTick() - Cxof_Time;
          			calculate_cxof(location_esm, d_location, speed, Dtime);
					//printf("vx:%f, vy:%f\r\n",speed[0], speed[1]);
          			Pack_cxof_buf(speed, 100, cxof_buf);
					Cxof_Time = HAL_GetTick();
					Cxof_Wait = HAL_GetTick();
					BEEP_OFF();
          			Send_cxof_buf(USART3, cxof_buf, 9);
        		}
				rxlen_uart_5 = 0;
				UART5_RX_STA = 0;
				BSP_USART_StartIT_LL(UART5); //启动下一次接�?
			}
			if(HAL_GetTick() - Cxof_Wait >= 500)			//超过300ms未接收到uwb的数据，蜂鸣器响起报�?
				BEEP_OFF();
			if (3 == RC_Read()) //飞控助手控制
			{	
				switch (task)
				{
				case 0:
					//任务�?:�?键起�?
					if(height > 300) Set_PWM_Mode(4500);		//Loiter -> 1500 x 3
					if(takeoff(height, location_esm, &is_takeoff, &is_settarget))
					{
						BEEP_ON();
						HAL_Delay(1000);
						BEEP_OFF();
						task = 1;
					}
					break;
				case 1:
					if(taskOne(location_esm, 300, 275, 150, 125, is_SetStartPoint))
					{
						BEEP_ON();
						HAL_Delay(1000);
						BEEP_OFF();
					}
					break;
				case 2:
					if(landon(height,location_esm,&is_settarget))
					{
						BEEP_ON();
						HAL_Delay(1000);
						BEEP_OFF();
						Set_PWM_Thr(3000);
						task = 4;
					}
				default:
					Back2Center();
					break;
				}
				//将UWB无线定位后的坐标结果传入PID外环，进行控�?
				ContriGetDataTime = HAL_GetTick() - ContriGetDataStart;
				if(ContriGetDataTime >= 300)
				{
					// printf("next_x:%d, next_y:%d\r\n", next_location[0], next_location[1]);
					// printf("roll_out:%d, pitch_out:%d\r\n", pwm_roll_out, pwm_pitch_out);
					printf("real_x:%f, real_y:%f, ", location[0], location[1]);
					printf("kal_x %f kal_y %f kal_h %f\r\n", location_esm[0], location_esm[1], height_esm);
					ContriGetDataStart = HAL_GetTick();	
				}
			}
			else if (2 == RC_Read())//通道回中
			{
				Set_PWM_Thr(4500);
				Set_PWM_Pitch(4500);
				Set_PWM_Roll(4500);
				Set_PWM_Yaw(4500);
			}
			else if (1 == RC_Read())//遥控器控�?
			{
				//桥接模式
				RC_bridge();
				task = 0;
				is_takeoff = 1;
				is_settarget = 0;
				is_SetStartPoint = 0;
				reset_path_flag(t1_path_flag, 4);
			}
		}

		RunTime = HAL_GetTick() - tickstart;
//		if (RunTime > 1000)
//		{
//			tickstart = HAL_GetTick();
//			LED_G_Flash();
//			LED_R_Flash();
//			LED_B_Flash();
//			HAL_Delay(10);
//		}
	}
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*
*********************************************************************************************************
*	�? �? �?: PrintfHardInfo
*	功能说明: 打印硬件接线信息
*	�?    参：�?
*	�? �? �?: �?
*********************************************************************************************************
*/
static void PrintfHardInfo(void)
{
	printf("*****************************************************************\r\n");
	printf("  接线方法: \r\n");
	printf("**梦创电子四旋翼飞行器室内自动飞行****\r\n");
	printf("  +5V       <------   5.0V      5V供电\r\n");
	printf("  GND       -------   GND       地\r\n");
	printf("  PB0       ------>   PWM3      与遥控器的pitch通道相连\r\n");
	printf("  PB1       ------>   PWM4      与遥控器的接收机的第五�?�道相连 \r\n");
	printf("  PA6       ------>   PWM1      与遥控器接收机油门相�?? \r\n");
	printf("  PB7       ------>   PWM2      与遥控器的偏航�?�道(YAW)相连\r\n");
	printf("  PA0       ------>   ADC_CH1   模拟超声波高度测量\r\n");
	printf("  打印采集数据: \r\n");
	printf("*****************************************************************\r\n");
}

unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
	unsigned short CRC_Temp;
	unsigned char i, j;
	CRC_Temp = 0xffff;

	for (i = 0; i < CRC_CNT; i++)
	{
		CRC_Temp ^= Buf[i];
		for (j = 0; j < 8; j++)
		{
			if (CRC_Temp & 0x01)
				CRC_Temp = (CRC_Temp >> 1) ^ 0xa001;
			else
				CRC_Temp = CRC_Temp >> 1;
		}
	}
	return (CRC_Temp);
}

//*********************OutPut_Data****************************************//
void OutPut_Data(void)
{
	int temp[4] = {0};
	unsigned int temp1[4] = {0};
	unsigned char databuf[10] = {0};
	unsigned char i;
	unsigned short CRC16 = 0;
	for (i = 0; i < 4; i++)
	{

		temp[i] = (int16_t)OutData[i];
		temp1[i] = (int16_t)temp[i];
	}

	for (i = 0; i < 4; i++)
	{
		databuf[i * 2] = (int8_t)(temp1[i] % 256);
		databuf[i * 2 + 1] = (int8_t)(temp1[i] / 256);
	}

	CRC16 = CRC_CHECK(databuf, 8);
	databuf[8] = CRC16 % 256;
	databuf[9] = CRC16 / 256;

	for (i = 0; i < 10; i++)
	{
		BSP_USART_SendArray_LL(USART1, databuf, sizeof(databuf));
		// HAL_UART_Transmit(&huart1,(uint8_t *)&databuf[i],1,10);       //串口发�??
	}
}

//****************************Data_to_VisualScope***************************************//
void Data_to_VisualScope(void)
{
	int16_t i;
	for (i = 0; i < CH_NUM; i++)
	{
		OutData[i] = s_volt[i];
		OutPut_Data();
	}
}

/*
*********************************************************************************************************
*	�? �? �?: AD转化函数
*	功能说明: 处理采样后的数据
*	�?    参：�?
*	�? �? �?: �?
*********************************************************************************************************
*/

float ADC_CvtVolt(void)
{
	float volt = 0;
	uint8_t i;
	int16_t adc;
	for (i = 0; i < CH_NUM; i++)
	{

		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 0xffff);

		ADC_ReadBuffer[i] = HAL_ADC_GetValue(&hadc1);
		s_dat[i] = ADC_ReadBuffer[i];
		adc = s_dat[i];
		s_volt[i] = (adc * 3.30f) / 4096;
	}
	HAL_ADC_Stop(&hadc1);
	volt = s_volt[0] * voltage_ratio;
	return volt;
}

/*
*********************************************************************************************************
*	�? �? �?: AD7606_Mak
*	功能说明: 处理采样后的数据
*	�?    参：�?
*	�? �? �?: �?
*********************************************************************************************************
*/

float ADC_CvtVolt_and_filter(void)
{
	float ADC_read = 0.0;
	ADC_read = filter_av(1);
	return ADC_read;
}

//***********读取ADC****************//
float get_adc(char adc_id)
{
	float volt = 0;
	int16_t adc;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 0xffff);

	adc = HAL_ADC_GetValue(&hadc1);
	volt = ((adc * 3.30f) / 4096) * voltage_ratio;
	HAL_ADC_Stop(&hadc1);
	HAL_Delay(1);
	// printf("V=%2.2f V Data_after_Deal=%2.2f cm  H=%2.2f cm  (count0=%4.1d) pwm_duty_out=%2.2f pwm_out=%d control_val=%2.2d \r\n",volt_0/voltage_ratio,Data_after_Deal,volt_0,count0,pwm_duty_out,PWM_Out,control_val);
	return volt;
}

//********算数平均滤波�?**************//
float filter_av(char filter_id)
{
	char count = 0;
	float sum = 0.0;
	for (count = 0; count < N; count++)
	{
		sum = sum + get_adc(1);
	}
	return sum / N;
}

//************************************数据处理函数********************************//
float DataProcessing(float IN_Data)			//权重滤波
{
	static float out_Data = 0, last_Data = 0, filter_Data = 0;
	out_Data = 0.7 * IN_Data + 0.3 * last_Data;
	filter_Data = filter(out_Data);
	last_Data = filter_Data;
	return filter_Data;
}

float filter(float new_value)
{
	if (new_value > 200)
		new_value = 200;
	if (new_value < 10)
		new_value = 10;
	if ((new_value - old_value > A) || (old_value - new_value > A))
	{
		old_value = old_value;
		return old_value;
	}
	old_value = new_value;
	return new_value;
}

//初始化测距模�?
VL53L1_Error vl53l1x_init(VL53L1_DEV pDev)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  pDev->I2cHandle = &hi2c1;
  pDev->I2cDevAddr = 0x52;
  pDev->comms_type = 1;
  pDev->comms_speed_khz = 400;

  Status = VL53L1_WaitDeviceBooted(pDev);
  if(Status != VL53L1_ERROR_NONE)
  {
    printf("Wait device Boot failed!\r\n");
		return Status;
  }
  HAL_Delay(2);

  Status = VL53L1_DataInit(pDev);
	if(Status!=VL53L1_ERROR_NONE) 
	{
		printf("datainit failed!\r\n");
		return Status;
	}
  HAL_Delay(2);

  Status = VL53L1_StaticInit(pDev);
	if(Status!=VL53L1_ERROR_NONE) 
	{
		printf("static init failed!\r\n");
		return Status;
	}
	HAL_Delay(2);

  Status = VL53L1_SetDistanceMode(pDev, VL53L1_DISTANCEMODE_MEDIUM);//设置测距距离模式
	if(Status!=VL53L1_ERROR_NONE) 
	{
		printf("set discance mode failed!\r\n");
		return Status;
	}
	HAL_Delay(2);

  Status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(pDev, 50000);//设置超时时间
	if(Status!=VL53L1_ERROR_NONE) 
		return Status;
	HAL_Delay(2);

	Status = VL53L1_SetInterMeasurementPeriodMilliSeconds(pDev, 50);//设置测量间隔
	if(Status!=VL53L1_ERROR_NONE) 
	{
		printf("SetInterMeasurementPeriodMilliSeconds failed!\r\n");
		return Status;
	}
	HAL_Delay(2);

  Status = VL53L1_StartMeasurement(pDev);
	if(Status!=VL53L1_ERROR_NONE) 
	{
		printf("start measurement failed!\r\n");
		return Status;
	}
  
  return Status;
}
//校准测距模块
VL53L1_Error vl53l1x_Cali(VL53L1_DEV pDev, VL53L1_CalibrationData_t* save)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  Status = VL53L1_StopMeasurement(pDev);
	if(Status!=VL53L1_ERROR_NONE) 
		return Status;

  Status = VL53L1_SetPresetMode(pDev, VL53L1_PRESETMODE_AUTONOMOUS);
  if(Status!=VL53L1_ERROR_NONE) 
		return Status;
  
  Status = VL53L1_PerformRefSpadManagement(pDev);//perform ref SPAD management
	if(Status!=VL53L1_ERROR_NONE) 
		return Status;

  Status = VL53L1_PerformOffsetSimpleCalibration(pDev,140);//14cm的出厂校验忿
  if(Status!=VL53L1_ERROR_NONE) 
		return Status;
  
  Status = VL53L1_PerformSingleTargetXTalkCalibration(pDev, 140);
	if(Status!=VL53L1_ERROR_NONE) 
		return Status;
  
  Status = VL53L1_GetCalibrationData(pDev,save);
	if(Status!=VL53L1_ERROR_NONE) 
		return Status;

  //全部完成 重新打开测量
  Status = VL53L1_StartMeasurement(pDev);
  if(Status!=VL53L1_ERROR_NONE) 
	{
		printf("start measurement failed!\r\n");
		return Status;
	}

  return Status;
}
//获取测量的距�?
VL53L1_Error vl53l1x_GetDistance(VL53L1_DEV pDev)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint8_t isDataReady=0;
  //status = VL53L1_WaitMeasurementDataReady(pDev);//阻塞
  Status = VL53L1_GetMeasurementDataReady(pDev,&isDataReady);//非阻塞测�?
  if(Status!=VL53L1_ERROR_NONE) 
	{
		printf("Wait too long!\r\n");
		return Status;
	}
  if(isDataReady)
  {
    Status = VL53L1_GetRangingMeasurementData(pDev, &result_data);
    height = result_data.RangeMilliMeter;
    // printf("distance: %d mm\r\n", height);
	//printf("%f m\r\n", height / 1000.0);
    Status = VL53L1_ClearInterruptAndStartMeasurement(pDev);
  }
  
  return Status;
}

/****************************串口中断回调*****************************/
void USART_RxCallback(USART_TypeDef *huart)
{
//	mavlink_message_t msg;
//	mavlink_status_t status;

	if (LL_USART_IsActiveFlag_RXNE(huart) && LL_USART_IsEnabledIT_RXNE(huart))
	{
		// printf("INT \n");
		//  ***********串口1中断**********************************
		if (huart == USART1)
		{
			uint8_t data = LL_USART_ReceiveData8(huart);
			// printf("USART1_RX_STA =%d data = %d \r\n",USART1_RX_STA , data);
			if ((USART1_RX_STA & (1 << 15)) == 0) //还可以接收数�?? ,�??高位不为1.
			{
				TIM11->CNT = 0;			//计数�??11清零
				if (USART1_RX_STA == 0) //新一轮接收数�??
				{
					TIM11_Set(1); //中断方式�??启定时器11
				}
				// BSP_USART_SendArray_LL( USART1,&USART1_RX_BUF[USART1_RX_STA],1);
				// printf("USART1 INT =%d \r\n",USART1_RX_STA);
				else
				{
					USART1_RX_STA |= 1 << 15; //强制标记接收完成
				}
			}
		}
		// ***********串口2中断，用于与视觉模块连接*********************
		else if (huart == USART2)
		{
			uint8_t data = LL_USART_ReceiveData8(huart);
			//printf("%c",data);
			if(data == 0x23)
			{
				UART2_Frame_Flag = 1;
			}
      if(((USART2_RX_STA  & (1<<15))==0) && (UART2_Frame_Flag == 1))		//还可以接收数据，�?高位不为1.
			{
				TIM13->CNT=0;											//计数�?13清空
        if(USART2_RX_STA == 0)
				{
					TIM13_Set(1);	 	                //使能定时�?13的中�?
					Recv_Cnt_UART2 = 0;
				}
				USART2_RX_BUF[USART2_RX_STA++] = data;
				Recv_Cnt_UART2 ++;			
				if(Recv_Cnt_UART2>=27)
				{
					Recv_Cnt_UART2 = 0;
					UART2_Frame_Flag = 0;
					USART2_RX_STA |= 1<<15;					//强制标记接收完成
					LL_USART_DisableIT_RXNE(USART2);
				}
      }	
		}
		// ******************串口3中断****************
		else if (huart == USART3)
		{
			uint8_t data = LL_USART_ReceiveData8(huart); 
			if ((USART3_RX_STA & (1 << 15)) == 0)
			{
				TIM14->CNT = 0;			//定时�?14清空
				if (USART3_RX_STA == 0) //新一轮接收开�?
				{
					TIM14_Set(1);
				}
				USART3_RX_BUF[USART3_RX_STA++] = data; //存入接收缓冲�?
				// printf("USART3 INT =%d \r\n",USART3_RX_STA);
			}
			else
			{
				USART3_RX_STA |= 1 << 15; //强制标记接收完成
				LL_USART_DisableIT_RXNE(UART5);
			}
		}
		// ******************串口5中断****************
		else if (huart == UART5)
		{
			uint8_t data = LL_USART_ReceiveData8(huart); //串口接收
			//printf("%c",data);
			if ((UART5_RX_STA & (1 << 7)) == 0)  //缓冲区还没满，继续接收数�?
			{
				UART5_RX_BUF[UART5_RX_STA++] = data; //存入接收缓冲�?
				// printf("USART5 INT =%d \r\n",USART5_RX_STA);
			}
			else
			{
				UART5_RX_STA |= 1 << 7; //强制标记接收完成
        LL_USART_DisableIT_RXNE(UART5);
			}
		}
	}
}

//*******定时器中断服务程�?	*************************************//
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint16_t tim11_1ms = 0; //中断次数计数
	static uint16_t tim13_1ms = 0; //中断次数计数
	static uint16_t tim14_1ms = 0; //中断次数计数
	static uint16_t tim10_1ms = 0; //中断次数计数

	//*****定时�?10中断服务函数->用于延时*********
	if (htim->Instance == htim10.Instance) //更新中断
	{
		tim10_1ms++;
		if (tim10_1ms == 10) 
		{
			// printf("TIME 10 INT \r\n");
		}
	}
	//*****定时�?11中断服务函数->在串�?1中使用到更新中断*********
	if (htim->Instance == htim11.Instance) //更新中断
	{
		tim11_1ms++;
		if (tim11_1ms == 50) 
		{
			USART1_RX_STA |= (1 << 15); //标记接收完成
			TIM11->SR &= ~(1 << 0);		//清除中断标志
			TIM11_Set(0);				//关闭TIM11
			tim11_1ms = 0;
			// printf("TIME 11 INT \r\n");
		}
	}
	//*****定时�??13中断服务函数->用于串口2*********************
	if (htim->Instance == htim13.Instance) 
	{
		tim13_1ms++;
		if (tim13_1ms == 40) 
		{
			USART2_RX_STA |= (1 << 15); //标记接收完成
			TIM13->SR &= ~(1 << 0);		//清除中断标志
			TIM13_Set(0);				//关闭TIM13
			tim13_1ms = 0;
			// printf("TIME 13 INT \r\n");
		}
	}
	//*****定时�??14中断服务函数->用于串口3*********************
	if (htim->Instance == htim14.Instance) //更新中断
	{
		tim14_1ms++;
		if (tim14_1ms == 50) 
		{
			USART3_RX_STA |= (1 << 15); //标记接收完成
			TIM14->SR &= ~(1 << 0);		//清除中断标志
			TIM14_Set(0);				//关闭TIM14
			tim14_1ms = 0;
			// printf("TIME 14 INT \r\n");
		}
	}
}
// tim10
void TIM10_Set(uint8_t sta)
{
	if (sta)
	{
		TIM10->CNT = 0;
		HAL_TIM_Base_Start_IT(&htim10);
	}
	else
		HAL_TIM_Base_Stop_IT(&htim10);
}

//定时�?11
void TIM11_Set(uint8_t sta)
{
	if (sta)
	{
		TIM11->CNT = 0;					//计数器清空计�?
		HAL_TIM_Base_Start_IT(&htim11); //使能定时�?11
	}
	else
		HAL_TIM_Base_Stop_IT(&htim11); //关闭定时�?11
}

//定时�?13
void TIM13_Set(uint8_t sta)
{
	if (sta)
	{
		TIM13->CNT = 0;					//计数器清�??
		HAL_TIM_Base_Start_IT(&htim13); //使能定时�??13
	}
	else
		HAL_TIM_Base_Stop_IT(&htim13); //关闭定时�??13
}

//定时�??14
//********************************
//采用定时器轮询的方式实现延时�??
//		for(int q=0;q<1000;q++)
//		{
//				Delay_us(1000);
//		}
//调用示例
//********************************

void TIM14_Set(uint8_t sta)
{
	if (sta)
	{
		TIM14->CNT = 0;					//计数器清�??
		HAL_TIM_Base_Start_IT(&htim14); //使能定时�??14
	}
	else
		HAL_TIM_Base_Stop_IT(&htim14); //关闭定时�??14
}

//定时器输入捕获PWM
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	// printf("TIM_IC \r\n");
	TIM_IC_InitTypeDef sConfigIC;
	//定时�??2输入捕获
	if (htim->Instance == htim2.Instance)
	{
		switch (htim->Channel)
		{
		/********************** TIM2_CH1->AUX2_In **************************/
		case HAL_TIM_ACTIVE_CHANNEL_1:
			if (ICFLAG_1)
			{
				CHANNEL_1_RISE = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);

				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING; //下降�??
				sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
				sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
				sConfigIC.ICFilter = 0;
				HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
				HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1);

				__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC1);
				if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK)
				{
					printf("ERROR\r\n");
				}

				ICFLAG_1 = 0;
			}
			else
			{
				CHANNEL_1_FALL = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);

				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING; //上升�??
				sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
				sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
				sConfigIC.ICFilter = 0;
				HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
				HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1);

				__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC1);
				if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK)
				{
					printf("ERROR\r\n");
				}

				CHANNEL_1_PULSE_WIDE = (CHANNEL_1_FALL > CHANNEL_1_RISE ? (CHANNEL_1_FALL - CHANNEL_1_RISE) : (CHANNEL_1_FALL - CHANNEL_1_RISE + 60000));
				ICFLAG_1 = 1;
			}
			// CH1_PWM_test(CHANNEL_1_PULSE_WIDE);
			// printf("channel1 pulsewidth = %d \r\n", CHANNEL_1_PULSE_WIDE);
			break;
		/*********************************  TIM2_CH2 -> AUX1_In ****************************************/
		case HAL_TIM_ACTIVE_CHANNEL_2:
			if (ICFLAG_2)
			{
				CHANNEL_2_RISE = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);

				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING; //下降�??
				sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
				sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
				sConfigIC.ICFilter = 0;
				HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);
				HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2);

				__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC2);
				if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2) != HAL_OK)
				{
					printf("ERROR\r\n");
				}

				ICFLAG_2 = 0;
			}
			else
			{
				CHANNEL_2_FALL = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);

				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING; //上升�??
				sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
				sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
				sConfigIC.ICFilter = 0;
				HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);
				HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2);

				__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC2);
				if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2) != HAL_OK)
				{
					printf("ERROR\r\n");
				}

				CHANNEL_2_PULSE_WIDE = (CHANNEL_2_FALL > CHANNEL_2_RISE ? CHANNEL_2_FALL - CHANNEL_2_RISE : CHANNEL_2_FALL - CHANNEL_2_RISE + 60000);
				ICFLAG_2 = 1;
			}
			// CH2_PWM_test(CHANNEL_2_PULSE_WIDE);
			// printf("channel2 pulsewidth = %d \r\n", CHANNEL_2_PULSE_WIDE);
			break;
		/***********************************  TIM2_CH3 -> Ctrl_In ***************************************/
		case HAL_TIM_ACTIVE_CHANNEL_3:
			if (ICFLAG_3)
			{
				CHANNEL_3_RISE = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);

				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING; //下降�??
				sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
				sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
				sConfigIC.ICFilter = 0;
				HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_3);
				HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3);

				__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC3);
				if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3) != HAL_OK)
				{
					printf("ERROR\r\n");
				}

				ICFLAG_3 = 0;
			}
			else
			{
				CHANNEL_3_FALL = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);

				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING; //上升�??
				sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
				sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
				sConfigIC.ICFilter = 0;
				HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_3);
				HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3);

				__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC3);
				if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3) != HAL_OK)
				{
					printf("ERROR\r\n");
				}

				CHANNEL_3_PULSE_WIDE = (CHANNEL_3_FALL > CHANNEL_3_RISE ? CHANNEL_3_FALL - CHANNEL_3_RISE : CHANNEL_3_FALL - CHANNEL_3_RISE + 60000);
				ICFLAG_3 = 1;
			}
			// CH3_PWM_test(CHANNEL_3_PULSE_WIDE);
			// printf("channel3 pulsewidth = %d \r\n", CHANNEL_3_PULSE_WIDE);
			break;
		/**********************************  TIM2_CH4 -> Mode_In ***************************************/
		case HAL_TIM_ACTIVE_CHANNEL_4:
			if (ICFLAG_4)
			{
				CHANNEL_4_RISE = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_4);

				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING; //下降�??
				sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
				sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
				sConfigIC.ICFilter = 0;
				HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_4);
				HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4);

				__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC4);
				if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4) != HAL_OK)
				{
					printf("ERROR\r\n");
				}

				ICFLAG_4 = 0;
			}
			else
			{
				CHANNEL_4_FALL = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_4);

				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING; //上升�??
				sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
				sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
				sConfigIC.ICFilter = 0;
				HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_4);
				////////////////同上
				HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4);

				__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC4);
				if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4) != HAL_OK)
				{
					printf("ERROR\r\n");
				}

				CHANNEL_4_PULSE_WIDE = (CHANNEL_4_FALL > CHANNEL_4_RISE ? CHANNEL_4_FALL - CHANNEL_4_RISE : CHANNEL_4_FALL - CHANNEL_4_RISE + 60000);
				ICFLAG_4 = 1;
			}
			// CH4_PWM_test(CHANNEL_3_PULSE_WIDE);
			// printf("channel4 pulsewidth = %d \r\n", CHANNEL_4_PULSE_WIDE);
			break;
		default:
			break;
		}
	}

	//****************************定时�??3输入捕获****************************************
	if (htim->Instance == htim3.Instance)
	{
		switch (htim->Channel)
		{
		/*****************************  TIM3_CH1 -> Yaw_In ********************************/
		case HAL_TIM_ACTIVE_CHANNEL_1:
			if (ICFLAG_5)
			{
				CHANNEL_5_RISE = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);

				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING; //下降�??
				sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
				sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
				sConfigIC.ICFilter = 0;
				HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);
				HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1);

				__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC1);
				HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);

				ICFLAG_5 = 0;
			}
			else
			{
				CHANNEL_5_FALL = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);

				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING; //上升�??
				sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
				sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
				sConfigIC.ICFilter = 0;
				HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);
				HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1);

				__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC1);
				HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);

				CHANNEL_5_PULSE_WIDE = (CHANNEL_5_FALL > CHANNEL_5_RISE ? CHANNEL_5_FALL - CHANNEL_5_RISE : CHANNEL_5_FALL - CHANNEL_5_RISE + 60000);
				ICFLAG_5 = 1;
			}
			// CH1_PWM_test(CHANNEL_1_PULSE_WIDE);
			// printf("channel5 pulsewidth = %d \r\n", CHANNEL_5_PULSE_WIDE);
			break;
		/******************************  TIM3_CH2 -> Roll_In ******************************/
		case HAL_TIM_ACTIVE_CHANNEL_2:
			if (ICFLAG_6)
			{
				CHANNEL_6_RISE = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);

				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING; //下降�??
				sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
				sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
				sConfigIC.ICFilter = 0;
				HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_2);
				HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2);

				__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC2);
				HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

				ICFLAG_6 = 0;
			}
			else
			{
				CHANNEL_6_FALL = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);

				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING; //上升�??
				sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
				sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
				sConfigIC.ICFilter = 0;
				HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_2);
				HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2);

				__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC2);
				HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

				CHANNEL_6_PULSE_WIDE = (CHANNEL_6_FALL > CHANNEL_6_RISE ? CHANNEL_6_FALL - CHANNEL_6_RISE : CHANNEL_6_FALL - CHANNEL_6_RISE + 60000);
				ICFLAG_6 = 1;
			}
			// CH1_PWM_test(CHANNEL_1_PULSE_WIDE);
			// printf("channel5 pulsewidth = %d \r\n", CHANNEL_5_PULSE_WIDE);
			break;
		/******************************  TIm3_CH3 -> Pitch_In *************************/
		case HAL_TIM_ACTIVE_CHANNEL_3:
			if (ICFLAG_7)
			{
				CHANNEL_7_RISE = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);

				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING; //下降�??
				sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
				sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
				sConfigIC.ICFilter = 0;
				HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_3);
				HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3);

				__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC3);
				HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);

				ICFLAG_7 = 0;
			}
			else
			{
				CHANNEL_7_FALL = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);

				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING; //上升�??
				sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
				sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
				sConfigIC.ICFilter = 0;
				HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_3);
				HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3);

				__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC3);
				HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);

				CHANNEL_7_PULSE_WIDE = (CHANNEL_7_FALL > CHANNEL_7_RISE ? CHANNEL_7_FALL - CHANNEL_7_RISE : CHANNEL_7_FALL - CHANNEL_7_RISE + 60000);
				ICFLAG_7 = 1;
			}
			// CH1_PWM_test(CHANNEL_1_PULSE_WIDE);
			// printf("channel5 pulsewidth = %d \r\n", CHANNEL_5_PULSE_WIDE);
			break;
		/************************************  TIM3_CH4 -> Thr_In *************************************/
		case HAL_TIM_ACTIVE_CHANNEL_4:
			if (ICFLAG_8)
			{
				CHANNEL_8_RISE = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4);

				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING; //下降�??
				sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
				sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
				sConfigIC.ICFilter = 0;
				HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_4);
				HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4);

				__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC4);
				HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_4);

				ICFLAG_8 = 0;
			}
			else
			{
				CHANNEL_8_FALL = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4);

				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING; //上升�??
				sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
				sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
				sConfigIC.ICFilter = 0;
				HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_4);
				HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4);

				__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC4);
				HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);

				CHANNEL_8_PULSE_WIDE = (CHANNEL_8_FALL > CHANNEL_8_RISE ? CHANNEL_8_FALL - CHANNEL_8_RISE : CHANNEL_8_FALL - CHANNEL_8_RISE + 60000);
				ICFLAG_8 = 1;
			}
			// CH2_PWM_test(CHANNEL_2_PULSE_WIDE);
			// printf("channel6 pulsewidth = %d \r\n", CHANNEL_6_PULSE_WIDE);
			break;
		default:
			break;
		}
	}
}

//手动发�?�mavlink信号
void MANUAL_CONTROL_Send(int16_t xpoint, int16_t ypoint)
{
	uint8_t system_id = 255;  // 发�?�本条消息帧的设备的系统编号（sys�??
	uint8_t component_id = 0; // 发�?�本条消息帧的设备的单元编号（comp�??
	uint8_t target = 0x01;	  //目标系统
	int16_t x = 0;
	int16_t y = ypoint;
	int16_t z = 0;
	int16_t r = 0;
	uint16_t buttons = 0;
	mavlink_message_t msg; // msg The MAVLink message to compress the data into
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;

	mavlink_msg_manual_control_pack(system_id, component_id, &msg, target, x, y, z, r, buttons);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	// UART_Send_Str(buf,len);
	BSP_USART_SendArray_LL(USART1, buf, sizeof(buf));
}

void RC_CHANNELS_OVERRIDE_Send(int16_t xpoint, int16_t ypoint)
{
	uint8_t system_id = 255;
	uint8_t component_id = 0;
	mavlink_message_t msg;
	uint8_t target_system = 1;
	uint8_t target_component = 0;
	uint16_t chan1_raw = ypoint;
	uint16_t chan2_raw = 65535;
	uint16_t chan3_raw = 65535;
	uint16_t chan4_raw = 65535;
	uint16_t chan5_raw = 65535;
	uint16_t chan6_raw = 65535;
	uint16_t chan7_raw = 65535;
	uint16_t chan8_raw = 65535;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;

	mavlink_msg_rc_channels_override_pack(system_id, component_id, &msg, target_system, target_component,
										  chan1_raw, chan2_raw, chan3_raw, chan4_raw, chan5_raw, chan6_raw, chan7_raw, chan8_raw);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	BSP_USART_SendArray_LL(USART1, buf, sizeof(buf));
	// UART_Send_Str(buf,len);

	mavlink_msg_rc_channels_override_pack(system_id, component_id, &msg, target_system, target_component,
										  0, chan2_raw, chan3_raw, chan4_raw, chan5_raw, chan6_raw, chan7_raw, chan8_raw);

	len = mavlink_msg_to_send_buffer(buf, &msg);
	// UART_Send_Str(buf,len);
	BSP_USART_SendArray_LL(USART1, buf, sizeof(buf));
}

//*****心跳信号*************************
void heartbeat_Mavlink(void)
{
	uint8_t system_id = 255;
	uint8_t component_id = 0;
	mavlink_message_t heart_msg;
	uint8_t type = 0x06;
	uint8_t autopilot = 0x08;
	uint8_t base_mode = 0xc0;
	uint32_t custom_mode = 0x0000;
	uint8_t system_status = 0x04;

	uint8_t buf_head[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;

	mavlink_msg_heartbeat_pack(system_id, component_id, &heart_msg, type, autopilot, base_mode, custom_mode, system_status);
	len = mavlink_msg_to_send_buffer(buf_head, &heart_msg);
	// UART_Send_Str(buf_head,len);
	BSP_USART_SendArray_LL(USART1, buf_head, sizeof(buf_head));
}

//读取工作模式
int RC_Read(void)
{
	 // 2500~4500 桥接模式
	 if (CHANNEL_3_PULSE_WIDE >= PWM_Ctrl_N1 && CHANNEL_3_PULSE_WIDE <= PWM_Ctrl_N2)
	 {
		 return 1;
	 }
	 // 4500~5000 回中(�??)
	 else if (CHANNEL_3_PULSE_WIDE >= PWM_Ctrl_N2 && CHANNEL_3_PULSE_WIDE <= PWM_Ctrl_N3)
	 {
		 return 2;
	 }
	 // 5000~6500 飞控助手模式
	 else if (CHANNEL_3_PULSE_WIDE >= PWM_Ctrl_N3 && CHANNEL_3_PULSE_WIDE <= PWM_Ctrl_N4)
	 {
		 return 3;
	 }
	 else
		 return 0;
}

//各�?�道回中
void Back_to_Center(void)
{
	Set_PWM_Thr(4500);
	Set_PWM_Pitch(4500);
	Set_PWM_Roll(4500);
	Set_PWM_Yaw(4500);
	Set_PWM_Mode(4500);
}

void RC_Week_Bridge(void)
{
	//printf("1= %d 2= %d 3= %d 4= %d 5= %d 8= %d \r\n", CHANNEL_1_PULSE_WIDE, CHANNEL_2_PULSE_WIDE, CHANNEL_3_PULSE_WIDE, CHANNEL_4_PULSE_WIDE, CHANNEL_5_PULSE_WIDE, CHANNEL_8_PULSE_WIDE);
	Set_PWM_Thr(CHANNEL_8_PULSE_WIDE);
	Set_PWM_Pitch(CHANNEL_7_PULSE_WIDE);
	Set_PWM_Yaw(CHANNEL_5_PULSE_WIDE);
	Set_PWM_Mode(CHANNEL_4_PULSE_WIDE);
	//Set_PWM_Ctrl(CHANNEL_3_PULSE_WIDE);
	//Set_PWM_Aux1(CHANNEL_2_PULSE_WIDE);
	//Set_PWM_Aux2(CHANNEL_1_PULSE_WIDE);
}

/* USER CODE END 4 */

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
