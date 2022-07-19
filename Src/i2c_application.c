/******************** (C) COPYRIGHT 2022  MengChuang  Tech ********************************************
  * ����   ���δ��Ƽ�
 * �ļ���  ��mpu6050.c
 * ����    ��6�ᴫ����mpu6050����
 * �Ա�    ��https://shop144519723.taobao.com/index.htm?spm=2013.1.w5002-13163471369.2.71db1223NyFC4j 
 * ����QȺ ��642013549
********************************************************************************************************/

#include "stm32f4xx_hal.h"
#include "i2c_application.h"

// IICдһ���ֽ�����
u8 IIC_Write_1Byte(u8 SlaveAddress,u8 REG_Address,u8 REG_data)
{
	HAL_StatusTypeDef ret;
	unsigned char buf[2];
	buf[0] = REG_Address;
	buf[1] = REG_data;
 
	ret = HAL_I2C_Master_Transmit( &hi2c3, SlaveAddress,buf, 2, 100);
	if( ret != HAL_OK )
	{
			//printf("Error while iic reg transmit(0x%02x).\r\n", ret );
			return 1;
	}
	return 0;
}

// IIC��1�ֽ�����
u8 IIC_Read_1Byte(u8 SlaveAddress,u8 REG_Address,u8 *REG_data)
{      		
	HAL_StatusTypeDef ret;
	unsigned char buf[2];
 
	ret = HAL_I2C_Master_Transmit( &hi2c3, SlaveAddress,&REG_Address, 1, 100);
	if( ret != HAL_OK )
	{
			//printf("Error while iic reg addr transmit(0x%02x).\r\n",ret);
			return 1;
	}
	
	HAL_I2C_Master_Receive (&hi2c3, SlaveAddress, buf, 1, 100);
	if( ret != HAL_OK )
	{
			//printf("Error while iic data receive(0x%02x).\r\n",ret );
			return 1;
	}
	
	*REG_data = buf[0];
	return 0;
}	

// IICдn�ֽ�����
u8 IIC_Write_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
{	
  unsigned char data[2];
	data[0] = REG_Address;
	data[1] = (*buf)>>8;
 
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Master_Transmit( &hi2c3, SlaveAddress,data, 2, 100);
	if( ret != HAL_OK ){
			//printf("Error while iic reg transmit(0x%02x).\r\n", ret );
			return -1;
	}
	return 0;
}

// IIC��n�ֽ�����
u8 IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
{	
	  HAL_StatusTypeDef ret;
    unsigned char data[2];
   
    ret = HAL_I2C_Master_Transmit( &hi2c3, SlaveAddress,&REG_Address, 1, 100);
    if( ret != HAL_OK ){
        //printf("Error while iic reg addr transmit(0x%02x).\r\n",ret);
        return 1;
    }
    
    HAL_I2C_Master_Receive (&hi2c3, SlaveAddress, data, 1, 100);
    if( ret != HAL_OK ){
        //printf("Error while iic data receive(0x%02x).\r\n",ret );
        return 1;
    }
    
    *buf = data[0];
    //*buf <<=8;
    //*buf |= data[1];
	  
		return 0;
}


/******************* (C) COPYRIGHT 2022 MengChuang TECH *****END OF FILE******************************/

