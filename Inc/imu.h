#ifndef _IMU_H_
#define	_IMU_H_

#include "stm32f4xx.h"
#include "math.h"
#include "mymath.h"
#include "mpu6050.h"

typedef struct 
{
	xyz_f_t err;
	xyz_f_t err_tmp;
	xyz_f_t err_lpf;
	xyz_f_t err_Int;
	xyz_f_t g;
	
}ref_t;

extern xyz_f_t reference_v;
extern float Roll,Pitch,Yaw;
extern u8 fly_ready;

void IMUupdate(float half_T,float gx, float gy, float gz, float ax, float ay, float az,float *rol,float *pit,float *yaw);

#endif

