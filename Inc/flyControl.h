#ifndef  __CONTROL_H__
#define  __CONTROL_H__

#include "stm32f4xx_hal.h"
#include "beep.h"
#include "led.h"
#include "uart_api.h"
#include <stdbool.h>

#define KEY1_PRES 	1
#define KEY2_PRES		2

#define High_eps  15.0
#define PWM_MIN_LIMIT 1101
#define PWM_MAX_LIMIT 1595  //21788

#define max(a, b)			(a>b? a:b)
#define min(a, b)			(a<b? a:b)
#define range(x, a, b)		(min(max(x, a), b))
#define exchange(a, b, tmp) (tmp=a, a=b, b=tmp)
#define myabs(x)			((x<0) ? -(x):x)

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
typedef enum  Fly_ModerDef
{
  Fly_Moder_Stabilize =0,
  Fly_Moder_AltHold ,
  Fly_Moder_Loiter,
	Fly_Moder_LAND,
}Fly_Mode_t;

typedef struct PID { 

        int   Pid_Clc_limit;		  //输出限幅    
	      int   SetPoint;           //  设定目标 Desired Value 

        float  Proportion;        //  比例常数 Proportional Const 
        float  Integral;          //  积分常数 Integral Const 
        float  Derivative;        //  微分常数 Derivative Const 

        float  LastError;         //  Error[-1] 
        float  PreviousError;     //  Error[-2] 
        float  SumError;          //  Sums of Errors 

} PID; 

typedef struct PID_TIME{
        int Max;
        int SetPoint;

        float Proportion;
        float Integral;
        float Derivative;

        float LastError;
        float PreviousError;
        float SumError; 
} PID_TIME;

typedef struct PID_LOCATION
{
        int Max;
        float SetPoint;

        float Proportion;
        float Integral;
        float Derivative;

        float LastError;
        float PreviousError;
        float SumError;
} PID_LOCATION;

typedef struct ATTITUDE { 
        float  Pitch;        
        float  Roll;          
        float  Yaw;         
        float  Heigh;          
        uint16_t  Position_x;          //  Error[-2] 
        uint16_t  Position_y;           //  Sums of Errors 
	uint16_t  SetPoint_x;
	uint16_t  SetPoint_y;
} ATTITUDE; 

extern ATTITUDE    Attitude; 
extern uint8_t InitedFlag;
extern int pwm_pitch_out;
extern int pwm_roll_out;
extern int pwm_pitch_SensorOut;
extern int pwm_roll_SensorOut;
extern int pwm_pitch_time;
extern int pwm_roll_time;
extern PID_TIME 	PID_Pitch_Time;
extern PID_TIME         PID_Roll_Time;
extern PID_LOCATION PID_Location_x;
extern PID_LOCATION PID_Location_y;

extern int CHANNEL_1_RISE,CHANNEL_1_FALL,CHANNEL_1_PULSE_WIDE;
extern int CHANNEL_2_RISE,CHANNEL_2_FALL,CHANNEL_2_PULSE_WIDE;
extern int CHANNEL_3_RISE,CHANNEL_3_FALL,CHANNEL_3_PULSE_WIDE;
extern int CHANNEL_4_RISE,CHANNEL_4_FALL,CHANNEL_4_PULSE_WIDE;
extern int CHANNEL_5_RISE,CHANNEL_5_FALL,CHANNEL_5_PULSE_WIDE;
extern int CHANNEL_6_RISE,CHANNEL_6_FALL,CHANNEL_6_PULSE_WIDE;
extern int CHANNEL_6_RISE,CHANNEL_6_FALL,CHANNEL_6_PULSE_WIDE;
extern int CHANNEL_7_RISE,CHANNEL_7_FALL,CHANNEL_7_PULSE_WIDE;
extern int CHANNEL_8_RISE,CHANNEL_8_FALL,CHANNEL_8_PULSE_WIDE;

void PIDInit (PID *pp);
void Device_Init(void);
void Attitude_init(ATTITUDE *p);
void Unlock(void);
void Lock(void);
void Take_off_Preper(void);
void Take_off(float target_height, float current_height);
void land(int current_height);
void Go_ahead(uint16_t pulse);
void Go_back(uint16_t pulse);
void Go_left(uint16_t pulse);
void Go_right(uint16_t pulse);
void Turn_left(uint16_t pulse);
void Turn_right(uint16_t pulse);
void Fly_Moder(int16_t FlyMode);
void Loiter(int point_x,int point_y,int SetPoint_x,int SetPoint_y,float pitch, float roll);
void Loiter_location(int point_x, int point_y, int SetPoint_x, int SetPoint_y);
void Back2Center(void);
void AltAdj(float high);
int PIDCalc( PID *pp, int CurrentPoint ,int SetPoint, bool flag);
int PID_GetTime(PID_TIME *pp, int CurrentPoint, int SetPoint);
int PID_location(PID_LOCATION* pp, int current_location, int target_location);

//***********油门*********************************************//
void Set_PWM_Thr(uint16_t pulse);
//***********俯仰**********************************************//
void Set_PWM_Pitch(uint16_t pulse);
//***********横滚*********************************************//
void Set_PWM_Roll(uint16_t pulse);
//***********偏航**********************************************//
void Set_PWM_Yaw(uint16_t pulse);
//***********模式**********************************************//
void Set_PWM_Mode(uint16_t pulse);
//***********桥接*********************************************//
void RC_bridge(void);
void RC_bridge_Test(void);
#endif


