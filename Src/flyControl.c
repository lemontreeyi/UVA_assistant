#include "main.h"
#include "stdio.h"
#include "flyControl.h"
#include "coderDecoder.h"
#include "arm_math.h"
#include<math.h>


int PWM_Pitch_Max=6000;
int PWM_Pitch_mid=4500;
int PWM_Pitch_min=3000;

int PWM_Roll_Max=6000;
int PWM_Roll_mid=4500;
int PWM_Roll_min=3000;

int PWM_Thr_Max=6000;
int PWM_Thr_mid=4500;
int PWM_Thr_min=3000;

int PWM_yaw_Max=6000;
int PWM_yaw_mid=4500;
int PWM_yaw_min=3000;

int PWM_Mode_Max=6000;
int PWM_Mode_mid=4500;
int PWM_Mode_min=3000;

int DeadThreShold=2;

int PWM_Thr_min_LOCK =1101;

int          pwm_pitch_out=0;
int          pwm_roll_out=0;
int 		 pwm_pitch_SensorOut = 0;
int			 pwm_roll_SensorOut = 0;
int 		 pwm_pitch_time = 0;
int 		 pwm_roll_time = 0;

unsigned char  SetHigh=80;
uint8_t InitedFlag;

PID         PID_Control_Pitch;
PID         PID_Control_Roll;
PID_TIME 	PID_Pitch_Time;
PID_TIME  	PID_Roll_Time;
PID_LOCATION PID_Location_x;
PID_LOCATION PID_Location_y;

/*==================================================================================================== 
    PID Function 
     
    The PID (比例、积分、微分) function is used in mainly 
    control applications. PIDCalc performs one iteration of the PID 
    algorithm. 

    While the PID function works, main is just a dummy program showing 
    a typical usage. 
=====================================================================================================*/ 


//*********设备初始化********//
void Device_Init(void)
{
	for(int i = 0 ;i<2;i++)
	{
		HAL_Delay(50);
	   	LED1_Flash();
		LED2_Flash();
		LED3_Flash();
		BEEP_ON();
		HAL_Delay(100);
		BEEP_OFF();
		InitedFlag =1;
		LED_R_Off();
		LED_G_Off();
		LED_B_Off();
	}
	Set_PWM_Thr(PWM_Thr_min_LOCK);	
	Set_PWM_Pitch(PWM_Pitch_mid);
	Set_PWM_Roll(PWM_Roll_mid);
	Set_PWM_Yaw(PWM_yaw_mid);
	Set_PWM_Mode(PWM_Mode_mid);
}

void Attitude_init(ATTITUDE *p)
{
	p->Pitch = 0;
	p->Roll = 0;
	p->Yaw = 0;
	p->Heigh = 0;
	p->Position_x = 112;
	p->Position_y = 112;
	p->SetPoint_x = 112;
	p->SetPoint_y = 112;
}

///*==================================================================================================== 
//   PID计算部分 
//=====================================================================================================*/ 

//int PIDCalc( PID *pp, int CurrentPoint ,int SetPoint ) 
//{ 
//	  float out;  
//		float dError, Error; 
//		pp->SetPoint =SetPoint;

//		Error = (float)pp->SetPoint -  (float)CurrentPoint; //计算偏差
//		pp->SumError += Error;                              // 积分 
//		pp->LastError = Error; 		                          //更新状态
//		dError = pp->PreviousError - pp->LastError; // 当前微分 
//		out =   pp->Proportion * Error              // 比例项 
//				+   pp->Integral * pp->SumError         // 积分项 
//				+   (pp->Derivative * dError)* (-0.5*exp(-0.6*(Error*Error))+1);  // 微分项 
//		//计算输出,限幅 
//		out = range(out, -pp->Pid_Clc_limit, pp->Pid_Clc_limit);

//		pp->PreviousError = pp->LastError; 		     //更新状态
//		return (int)out;
//} 

/*==================================================================================================== 
   PID计算部分 
=====================================================================================================*/ 
int PIDCalc( PID *pp, int CurrentPoint ,int SetPoint ,bool flag) 
{ 
		float out;  
		float dError, Error;
		//static float basic_value = 0;
		static float PID_Control_param = 1.2;

		PID_Control_param = (abs(CurrentPoint - SetPoint) > 95)?  1.2  : 1.0;
		//basic_value = (abs(CurrentPoint - SetPoint) > 85)?  150 : 200;
		pp->SetPoint =SetPoint; //112
		Error = (float)CurrentPoint - (float)pp->SetPoint;  
		pp->PreviousError = Error; 		   				
		pp->SumError += Error;                      		
		dError = pp->PreviousError - pp->LastError; 	
		pp->LastError = Error;
		out  =  (pp->Proportion * Error    
				+   pp->Integral * pp->SumError    
				+   pp->Derivative * dError) * PID_Control_param;     
		out = range(out, -pp->Pid_Clc_limit, pp->Pid_Clc_limit);
		if(flag) out *= 0.5;
		return (int)out;
}

/*
计算时间PID
*/
int PID_GetTime(PID_TIME *pp, int CurrentPoint, int SetPoint)
{
	float Error, dError;
	float out;
	pp->SetPoint = SetPoint;
	Error = (float)CurrentPoint - (float)pp->SetPoint;
	pp->PreviousError = Error;
	dError = pp->PreviousError - pp->LastError;
	pp->LastError = Error;
	out = pp->Proportion * Error
			+ pp->Derivative * dError;
	out = range(out, -pp->Max, pp->Max);
	out = abs((int)out);
	out += (out > 40) ? 0:(40 - out);	
	return out;
}

int PID_location(PID_LOCATION* pp, int current_location, int target_location)
{
    
    int Error, dError;
	int out;
	pp->SetPoint = target_location;
	Error = current_location - pp->SetPoint;
	pp->PreviousError = Error;
	dError = pp->PreviousError - pp->LastError;
	pp->LastError = Error;
	out = pp->Proportion * Error
			+ pp->Derivative * dError;
	out = range(out, -pp->Max, pp->Max);
	return (int)out;
}

/*==================================================================================================== 
   Initialize PID Structure 
=====================================================================================================*/ 

void delay1ms(int time)
{
  HAL_Delay(time);
}


void PIDInit (PID *pp) 
{ 
	PID_Control_Pitch.Pid_Clc_limit = 500;		//4500+400 = 4900
    PID_Control_Pitch.Proportion    = 2.6;//2.22;
	//  45 * 2.22 + 200 = 300; 85*2.22+200 = 389; 85*2.22*1.3+150=395; 112*2.22*1.3+50 = 473
	PID_Control_Roll.Integral       = 0;    
    PID_Control_Pitch.Derivative    = 2.5; 
    PID_Control_Pitch.SetPoint      = 112;
	PID_Control_Pitch.LastError     = 0;  
    PID_Control_Pitch.PreviousError = 0;    
    PID_Control_Pitch.SumError      = 0;   
	
	PID_Control_Roll.Pid_Clc_limit = 500;		//4500+600 = 5100
    PID_Control_Roll.Proportion    = 2.6;
	//  45 * 2.22 + 200 = 300; 85*2.22+200 = 389; 85*2.22*1.3+150=395; 112*2.22*1.3+50 = 473
    PID_Control_Roll.Integral      = 0; 
    PID_Control_Roll.Derivative    = 2.5;
    PID_Control_Roll.SetPoint      = 112;
	PID_Control_Roll.LastError     = 0;  
    PID_Control_Roll.PreviousError = 0;    
    PID_Control_Roll.SumError      = 0;

	PID_Pitch_Time.Max = 210;
	PID_Pitch_Time.Proportion = 2.2;
	PID_Pitch_Time.Integral = 0;
	PID_Pitch_Time.Derivative = 1.8;
	PID_Pitch_Time.SetPoint = 112;
	PID_Pitch_Time.LastError = 0;
	PID_Pitch_Time.PreviousError = 0;
	PID_Pitch_Time.SumError = 0;

	PID_Roll_Time.Max = 210;
	PID_Roll_Time.Proportion = 2.2;
	PID_Roll_Time.Integral = 0;
	PID_Roll_Time.Derivative = 1.8;
	PID_Roll_Time.SetPoint = 112;
	PID_Roll_Time.LastError = 0;
	PID_Roll_Time.PreviousError = 0;
	PID_Roll_Time.SumError = 0;
    
	PID_Location_x.Max = 650;
	PID_Location_x.Proportion =  3.4;
	PID_Location_x.Integral = 0;
	PID_Location_x.Derivative = 1.45;
	PID_Location_x.SetPoint = 0;
	PID_Location_x.LastError = 0;
	PID_Location_x.PreviousError = 0;
	PID_Location_x.SumError = 0;

	PID_Location_y.Max = 650;			//
	PID_Location_y.Proportion = -3.4;
	PID_Location_y.Integral = 0;
	PID_Location_y.Derivative = -1.45;
	PID_Location_y.SetPoint = 0;
	PID_Location_y.LastError = 0;
	PID_Location_y.PreviousError = 0;
	PID_Location_y.SumError = 0;

} 

/*==================================================================================================== 
   解锁 
=====================================================================================================*/ 
void Unlock(void)
{
	 //**********油门最低值***********//
	 Set_PWM_Thr(PWM_Thr_min_LOCK); //1071------>1101
	 HAL_Delay(2000);
	 for(int i=PWM_yaw_mid;i<=PWM_yaw_Max;i=i+86)
	 {
		LED1_Flash();
		Set_PWM_Yaw(i);   //解锁
	 }
	 delay1ms(3000);
	 for(int i =PWM_yaw_Max;i>=PWM_yaw_mid;i=i-86)
	 {
		LED1_Flash();
		Set_PWM_Yaw(i);   //回中
	 }
	 delay1ms(1000);
	 printf("*******************Unlock***************\r\n");
}



/*==================================================================================================== 
   加锁
=====================================================================================================*/ 
void Lock(void)
{
 BEEP_ON();
 HAL_Delay(3000);
 BEEP_OFF();
 HAL_Delay(1000);
 //**********油门最低值***********//
 Set_PWM_Thr(PWM_Thr_min_LOCK); 
 HAL_Delay(1000);
 //***********偏航向左*************//
 for(int i=PWM_yaw_mid;i>=PWM_yaw_min;i=i-128)
 {
	 LED1_Flash();
   Set_PWM_Yaw(i);
 }
 //***********偏航回中*************//
 delay1ms(5000);
 for(int i=PWM_yaw_min;i<=PWM_yaw_mid;i=i+128)
 {
	 LED1_Flash();
   Set_PWM_Yaw(i);   //回中
 }
 delay1ms(5000);
 printf("*******************lock***************\r\n");
}

/*==================================================================================================== 
   起飞准备
=====================================================================================================*/ 
void Take_off_Preper(void)
{	
	 printf("*******************Take_off_Preper***************\r\n");
}


/*==================================================================================================== 
   起飞 Fly_Moder_AltHold
=====================================================================================================*/ 
void Take_off(float target_height, float current_height)//mm
{	
	if(fabs(current_height - target_height) < 50.0){
		Set_PWM_Thr(4500);
		//printf("get thr mid!\r\n");
	}
	else if(fabs(current_height - target_height) > 50.0)
	{	
		//printf("get in thr control!\r\n");
		if(current_height < target_height) {
			Set_PWM_Thr((int)(4500 + 750 * exp((-current_height / target_height) * 0.95)));
			//printf("cur_distance = %f\r\n", current_height);
		}
		else Set_PWM_Thr((int)(4500 - 750 * exp((current_height - 2 * target_height) / target_height)));
	}	
}


/*==================================================================================================== 
   降落
=====================================================================================================*/ 
void land(int current_height)
{
	printf("*******************Land***************\r\n");
	if(current_height > 500)
		Set_PWM_Thr(4500 - 300);
	else if(current_height < 80)
		Set_PWM_Thr(0);
	else
		Set_PWM_Thr(4500 - 600 - 3900 * exp(-2 * current_height / 500));
}

/*==================================================================================================== 
   前进
=====================================================================================================*/ 
void Go_ahead(uint16_t pulse)
{
	Set_PWM_Pitch(PWM_Pitch_mid-pulse);      
}

/*==================================================================================================== 
   后退
=====================================================================================================*/ 
void Go_back(uint16_t pulse)
{
	Set_PWM_Pitch(PWM_Pitch_mid-pulse);      
}

/*==================================================================================================== 
   向左飞
=====================================================================================================*/ 
void Go_left(uint16_t pulse)
{
	Set_PWM_Roll(PWM_Roll_mid-pulse);      
}

/*==================================================================================================== 
   向右飞
=====================================================================================================*/ 
void Go_right(uint16_t pulse)
{
	Set_PWM_Roll(PWM_Roll_mid+pulse);      
}

/*==================================================================================================== 
   左转
=====================================================================================================*/ 
void Turn_left(uint16_t pulse)
{
	Set_PWM_Yaw(PWM_yaw_mid-pulse);      
}

/*==================================================================================================== 
   右转
=====================================================================================================*/ 
void Turn_right(uint16_t pulse)
{
	Set_PWM_Yaw(PWM_yaw_mid+pulse);      
}


/*==================================================================================================== 
   飞行模式
=====================================================================================================*/ 
void Fly_Moder(int16_t FlyMode)
{
	BEEP_ON();
    HAL_Delay(500);
    BEEP_OFF();
	HAL_Delay(100);
	if(FlyMode==Fly_Moder_Stabilize)
	{
		Set_PWM_Mode(PWM_Mode_min); 
		HAL_Delay(20);
		printf("*******************Fly_Moder_Stabilize***************\r\n");
	}
	if(FlyMode==Fly_Moder_AltHold)
	{
		Set_PWM_Mode(PWM_Mode_mid); 
		HAL_Delay(20);
		printf("*******************Fly_Moder_AltHold***************\r\n");
	}
	if(FlyMode==Fly_Moder_Loiter)
	{
		Set_PWM_Mode(PWM_Mode_min); 
		HAL_Delay(20);
		printf("*******************Fly_Moder_Loiter***************\r\n");
	}
	
	if(FlyMode==Fly_Moder_LAND)
	{
		Set_PWM_Mode(PWM_Mode_Max); 
		HAL_Delay(5000);
		//land();
		printf("*******************Fly_Moder_LAND***************\r\n");
	}
}

void Loiter_location(int point_x, int point_y, int SetPoint_x, int SetPoint_y)
{
	int pwm_pitch_clc=0;
	int pwm_roll_clc=0;
	int deadZoneX = 0;
	int deadZoneY = 0;
	static int last_point_x = 0, last_pint_y = 0;
	if(point_x == last_point_x && point_y == last_pint_y)
	{
		return;
	}
	last_point_x = point_x; last_pint_y = point_y;
	deadZoneX = point_x - SetPoint_x;
	deadZoneY = point_y - SetPoint_y;

	if(abs(deadZoneX) >= 25)
	{
		pwm_roll_clc = PID_location(&PID_Location_x, point_x, SetPoint_x);
		pwm_roll_out = PWM_Roll_mid + pwm_roll_clc;
		// Set_PWM_Roll(pwm_roll_out);
	}
	else {
		pwm_roll_out = PWM_Roll_mid;
		// Set_PWM_Roll(pwm_roll_out);
	}
	if(abs(deadZoneY) >= 25)
	{
		pwm_pitch_clc = PID_location(&PID_Location_y, point_y, SetPoint_y);
		pwm_pitch_out = PWM_Pitch_mid + pwm_pitch_clc;
		// Set_PWM_Pitch(pwm_pitch_out);
	}
	else {
		pwm_pitch_out = PWM_Pitch_mid;
		// Set_PWM_Pitch(pwm_pitch_out);
	}
}

/*==================================================================================================== 
   悬停
=====================================================================================================*/ 
void Loiter(int point_x,int point_y,int SetPoint_x,int SetPoint_y,float pitch, float roll)
{  
	//printf("*******************Loiter***************\r\n");
	int pwm_pitch_clc=0;
	int pwm_roll_clc=0;
	int deadZoneX = 0;
	int deadZoneY = 0;
	int h = 50;

	deadZoneX = point_x - SetPoint_x;
	deadZoneY = point_y - SetPoint_y;

	//roll方向
	if(abs(deadZoneX)>=DeadThreShold)
	{
		if(abs(point_x - SetPoint_x) < 35)
			pwm_roll_clc  = PIDCalc(&PID_Control_Roll  ,point_x,SetPoint_x, 1);
		else 
			pwm_roll_clc  = PIDCalc(&PID_Control_Roll  ,point_x,SetPoint_x, 0);
		pwm_roll_SensorOut  = PWM_Roll_mid  + pwm_roll_clc;
	}
	if(abs(deadZoneX)<DeadThreShold)
	{
		pwm_roll_SensorOut = PWM_Roll_mid;
	}

	//pitch方向
	if(abs(deadZoneY)>=DeadThreShold)
	{
		if(abs(point_y - SetPoint_y) < 35)
			pwm_pitch_clc = PIDCalc(&PID_Control_Pitch ,point_y,SetPoint_y, 1);
		else 
			pwm_pitch_clc = PIDCalc(&PID_Control_Pitch ,point_y,SetPoint_y, 0);
		pwm_pitch_SensorOut = PWM_Pitch_mid + pwm_pitch_clc;
	} 
	if(abs(deadZoneY) < DeadThreShold)
	{
		pwm_pitch_SensorOut = PWM_Pitch_mid;
	}
	// printf("Roll_x = %d , %d, Pitch_y = %d, %d\r\n", point_x,pwm_roll_out, point_y, pwm_pitch_out);
}


/*==================================================================================================== 
   高度调整
=====================================================================================================*/ 
void AltAdj(float high)
{	 
	 Set_PWM_Thr(PWM_Thr_mid);
	 printf("******************高度调整**************\r\n");
}



/*==================================================================================================== 
   PWM底层控制
=====================================================================================================*/ 
//定时器输出PWM通道对应关系：
//TIM5-> CH1 = Yaw; CH2 = Roll; CH3 = Pitch; CH4 = Thr
//TIm4-> CH1 = Mode; CH2 = Ctrl; CH3 = AUX1; CH4 = AUX2
//***********油门*********************************************//
void Set_PWM_Thr(uint16_t pulse)
{
  TIM5 ->CCR4 =pulse ;
}

//***********俯仰**********************************************//
void Set_PWM_Pitch(uint16_t pulse)
{
  TIM5 ->CCR3 =pulse ;
}


//***********横滚*********************************************//
void Set_PWM_Roll(uint16_t pulse)
{
  TIM5 ->CCR2 =pulse ;
}

//***********偏航**********************************************//
void Set_PWM_Yaw(uint16_t pulse)
{
  TIM5 ->CCR1 =pulse ;
}

//***********模式**********************************************//
/*
AltHold->1000 x 3
Loiter ->1500 x 3
Loiter ->2000 x 3
*/
void Set_PWM_Mode(uint16_t pulse)
{
  TIM4 ->CCR1 =pulse ;
}
/************Ctrl************************************************/
/*
直通 -> 1000 x 3
桥接 -> 2000 x 3
*/
void Set_PWM_Ctrl(uint16_t pulse)
{
	TIM4->CCR2 = pulse;
}

//***********直通模式*********************************************//
void RC_bridge(void)
{
//	printf("1= %d 2= %d 3= %d 4= %d 5= %d 6= %d\r\n",CHANNEL_1_PULSE_WIDE, CHANNEL_2_PULSE_WIDE, CHANNEL_3_PULSE_WIDE, CHANNEL_4_PULSE_WIDE, CHANNEL_5_PULSE_WIDE, CHANNEL_6_PULSE_WIDE); 
//	printf("CHANNEL_1_PULSE_WIDE= %d \r\n",CHANNEL_1_PULSE_WIDE); 
//	printf("CHANNEL_2_PULSE_WIDE= %d \r\n",CHANNEL_2_PULSE_WIDE); 
//	printf("CHANNEL_3_PULSE_WIDE= %d \r\n",CHANNEL_3_PULSE_WIDE); 
//	printf("CHANNEL_4_PULSE_WIDE= %d \r\n",CHANNEL_4_PULSE_WIDE); 
//	printf("CHANNEL_5_PULSE_WIDE= %d \r\n",CHANNEL_5_PULSE_WIDE); 
//	printf("CHANNEL_6_PULSE_WIDE= %d \r\n",CHANNEL_6_PULSE_WIDE); 
	Set_PWM_Thr(CHANNEL_8_PULSE_WIDE);
	Set_PWM_Pitch(CHANNEL_7_PULSE_WIDE);
	Set_PWM_Roll(CHANNEL_6_PULSE_WIDE);
	Set_PWM_Yaw(CHANNEL_5_PULSE_WIDE);
	Set_PWM_Mode(CHANNEL_4_PULSE_WIDE);
	Set_PWM_Ctrl(CHANNEL_3_PULSE_WIDE);
}


void RC_bridge_Test(void)
{
	/*
	3:Ctrl, 4:Mode, 5:Yaw, 6:Roll, 7:Pitch, 8:Thr
	*/
	printf("1= %d 2= %d 3= %d 4= %d 5= %d 6= %d 7= %d 8= %d\r\n",CHANNEL_1_PULSE_WIDE, CHANNEL_2_PULSE_WIDE, CHANNEL_3_PULSE_WIDE, CHANNEL_4_PULSE_WIDE, CHANNEL_5_PULSE_WIDE, CHANNEL_6_PULSE_WIDE, CHANNEL_7_PULSE_WIDE, CHANNEL_8_PULSE_WIDE); 
	Set_PWM_Thr(CHANNEL_8_PULSE_WIDE);
	Set_PWM_Pitch(CHANNEL_7_PULSE_WIDE);
	Set_PWM_Roll(CHANNEL_6_PULSE_WIDE);
	Set_PWM_Yaw(CHANNEL_5_PULSE_WIDE);
	Set_PWM_Mode(CHANNEL_4_PULSE_WIDE);
	Set_PWM_Ctrl(CHANNEL_3_PULSE_WIDE);
}

//各通道回中
void Back2Center(void)
{
	Set_PWM_Roll(4500);
	Set_PWM_Pitch(4500);
	//Set_PWM_Thr(4500);
}

