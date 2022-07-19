/******************** (C) COPYRIGHT 2022  MengChuang  Tech ********************************************
  * ����   ���δ��Ƽ�
 * �ļ���  ��mpu6050.c
 * ����    ��6�ᴫ����mpu6050����
 * �Ա�    ��https://shop144519723.taobao.com/index.htm?spm=2013.1.w5002-13163471369.2.71db1223NyFC4j 
 * ����QȺ ��642013549
********************************************************************************************************/

#include "imu.h"
#include "mymath.h"

#define Kp 0.6f                	// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.1f                	// 0.001  integral gain governs rate of convergence of gyroscope biases

#define IMU_INTEGRAL_LIM  ( 2.0f *ANGLE_TO_RADIAN )
#define NORM_ACC_LPF_HZ 10  		//(Hz)
#define REF_ERR_LPF_HZ  1				//(Hz)

xyz_f_t reference_v;
ref_t 	ref;

u8 fly_ready=1;

//xyz_f_t Gravity_Vec;  				//�������������
	
float Roll,Pitch,Yaw;    				//��̬��
float ref_q[4] = {1,0,0,0};
float norm_acc,norm_q;
float norm_acc_lpf;

void IMUupdate(float half_T,float gx, float gy, float gz, float ax, float ay, float az,float *rol,float *pit,float *yaw) 
{		
	float ref_err_lpf_hz;
	static float mag_norm ,mag_norm_xyz ,mag_tmp_x,mag_tmp_y,yaw_mag;
	float yaw_correct,yaw_tmp;
	float mag_norm_tmp;
	
	//mag_norm = my_sqrt(ak8975.Mag_Val.x * ak8975.Mag_Val.x + ak8975.Mag_Val.y * ak8975.Mag_Val.y);
	//mag_norm_xyz = my_sqrt(ak8975.Mag_Val.x * ak8975.Mag_Val.x + ak8975.Mag_Val.y * ak8975.Mag_Val.y + ak8975.Mag_Val.z * ak8975.Mag_Val.z);
	
	//mag_norm_tmp = LIMIT( 0.02f *( 50 - ABS(my_deathzoom((mag_norm_xyz - 100),10)) ) ,0.05f ,1 ) * 20 *(6.28f *half_T);
	
	//mag_tmp_x += mag_norm_tmp *( (float)ak8975.Mag_Val.x - mag_tmp_x);
	//mag_tmp_y += mag_norm_tmp *( (float)ak8975.Mag_Val.y - mag_tmp_y);	
	
	//if(ak8975.Mag_Val.x != 0 && ak8975.Mag_Val.y != 0 )
	//{
	//	yaw_tmp = fast_atan2(mag_tmp_y/mag_norm , mag_tmp_x/mag_norm) *57.3f;
		
	//	yaw_mag +=  10 *(6.28f *half_T) *(yaw_tmp - yaw_mag);
	//}
	//=============================================================================
	// �����Ч��������
	reference_v.x = 2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2]);
	reference_v.y = 2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]);
	reference_v.z = 1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2]);//ref_q[0]*ref_q[0] - ref_q[1]*ref_q[1] - ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3];

	
	//���ǰ���Ԫ������ɡ��������Ҿ����еĵ����е�����Ԫ�ء�
	//�������Ҿ����ŷ���ǵĶ��壬��������ϵ������������ת����������ϵ��������������Ԫ�ء�
	//���������vx\y\z����ʵ���ǵ�ǰ��ŷ���ǣ�����Ԫ�����Ļ����������ϵ�ϣ����������������λ������       
	//=============================================================================


	// ������ٶ�������ģ
	norm_acc = my_sqrt(ax*ax + ay*ay + az*az);   
	norm_acc_lpf +=  NORM_ACC_LPF_HZ *(6.28f *half_T) *(norm_acc - norm_acc_lpf);  //10hz *3.14 * 2*0.001


	if(ABS(ax)<4400 && ABS(ay)<4400 && ABS(az)<4400 )
	{	
		//�ѼӼƵ���ά����ת�ɵ�λ������
		ax = ax / norm_acc;//4096.0f;
		ay = ay / norm_acc;//4096.0f;
		az = az / norm_acc;//4096.0f; 
		
		if( 3800 < norm_acc && norm_acc < 4400 )
		{
			/* ��˵õ���� */
			ref.err_tmp.x = ay*reference_v.z - az*reference_v.y;
			ref.err_tmp.y = az*reference_v.x - ax*reference_v.z;
	    //ref.err_tmp.z = ax*reference_v.y - ay*reference_v.x;
			
			/* ����ͨ */
			ref_err_lpf_hz = REF_ERR_LPF_HZ *(6.28f *half_T);
			ref.err_lpf.x += ref_err_lpf_hz *( ref.err_tmp.x  - ref.err_lpf.x );
			ref.err_lpf.y += ref_err_lpf_hz *( ref.err_tmp.y  - ref.err_lpf.y );
	    //ref.err_lpf.z += ref_err_lpf_hz *( ref.err_tmp.z  - ref.err_lpf.z );
			
			ref.err.x = ref.err_lpf.x;//
			ref.err.y = ref.err_lpf.y;//
      //ref.err.z = ref.err_lpf.z ;
		}
	}
	else
	{
		ref.err.x = 0; 
		ref.err.y = 0  ;
//		ref.err.z = 0 ;
	}
	/* ������ */
	ref.err_Int.x += ref.err.x *Ki *2 *half_T ;
	ref.err_Int.y += ref.err.y *Ki *2 *half_T ;
	ref.err_Int.z += ref.err.z *Ki *2 *half_T ;
	
	/* �����޷� */
	ref.err_Int.x = LIMIT(ref.err_Int.x, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ref.err_Int.y = LIMIT(ref.err_Int.y, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ref.err_Int.z = LIMIT(ref.err_Int.z, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );

	if(fly_ready)
	{
		yaw_correct = Kp *0.3f *LIMIT(To_180_degrees(yaw_mag - Yaw),-20,20);
	}
	else
	{
		yaw_correct = Kp *1.5f *To_180_degrees(yaw_mag - Yaw);
	}

	if( reference_v.z > 0.7f )
	{	
		ref.g.x = (gx - reference_v.x *yaw_correct)*ANGLE_TO_RADIAN + ( Kp*(ref.err.x + ref.err_Int.x) ) ;     //IN RADIAN
	  ref.g.y = (gy - reference_v.y *yaw_correct)*ANGLE_TO_RADIAN + ( Kp*(ref.err.y + ref.err_Int.y) ) ;		  //IN RADIAN
		ref.g.z = (gz - reference_v.z *yaw_correct)*ANGLE_TO_RADIAN ;//+ ( Kp*(ref.err.z + ref.err_Int.z) )  ;   //IN RADIAN    
	}
	else
	{
		ref.g.x = gx *ANGLE_TO_RADIAN + ( Kp*(ref.err.x + ref.err_Int.x) ) ;     //IN RADIAN
	  ref.g.y = gy *ANGLE_TO_RADIAN + ( Kp*(ref.err.y + ref.err_Int.y) ) ;		  //IN RADIAN
		ref.g.z = gz *ANGLE_TO_RADIAN;
	}
	/* �ò���������PI����������ƫ */

	// integrate quaternion rate and normalise
	ref_q[0] = ref_q[0] +(-ref_q[1]*ref.g.x - ref_q[2]*ref.g.y - ref_q[3]*ref.g.z)*half_T;
	ref_q[1] = ref_q[1] + (ref_q[0]*ref.g.x + ref_q[2]*ref.g.z - ref_q[3]*ref.g.y)*half_T;
	ref_q[2] = ref_q[2] + (ref_q[0]*ref.g.y - ref_q[1]*ref.g.z + ref_q[3]*ref.g.x)*half_T;
	ref_q[3] = ref_q[3] + (ref_q[0]*ref.g.z + ref_q[1]*ref.g.y - ref_q[2]*ref.g.x)*half_T;  

	/* ��Ԫ����һ�� normalise quaternion */
	norm_q = my_sqrt(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3]);
	ref_q[0] = ref_q[0] / norm_q;
	ref_q[1] = ref_q[1] / norm_q;
	ref_q[2] = ref_q[2] / norm_q;
	ref_q[3] = ref_q[3] / norm_q;
	

	*rol = fast_atan2(2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]),1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2])) *57.3f;
	*pit = asin(2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2])) *57.3f;
  //Yaw   = ( - fast_atan2(2*(ref_q[1]*ref_q[2] + ref_q[0]*ref_q[3]),ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1] - ref_q[2]*ref_q[2] - ref_q[3]*ref_q[3]) )* 57.3;
	*yaw = fast_atan2(2*(ref_q[1]*ref_q[2] - ref_q[0]*ref_q[3]), 2*(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1]) - 1) *57.3f;
}


/******************* (C) COPYRIGHT 2022 MengChuang TECH *****END OF FILE************************************************************/

