#ifndef _I2C_SOFT_H
#define	_I2C_SOFT_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "time.h"

#define SCL_H         ANO_GPIO_I2C->BSRRL = I2C_Pin_SCL
#define SCL_L         ANO_GPIO_I2C->BSRRH = I2C_Pin_SCL
#define SDA_H         ANO_GPIO_I2C->BSRRL = I2C_Pin_SDA
#define SDA_L         ANO_GPIO_I2C->BSRRH = I2C_Pin_SDA
#define SCL_read      ANO_GPIO_I2C->IDR  & I2C_Pin_SCL
#define SDA_read      ANO_GPIO_I2C->IDR  & I2C_Pin_SDA

/***************I2C GPIO∂®“Â******************/
#define ANO_GPIO_I2C	GPIOB
#define I2C_Pin_SCL		GPIO_Pin_6
#define I2C_Pin_SDA		GPIO_Pin_7
#define ANO_RCC_I2C		RCC_AHB1Periph_GPIOB
/*********************************************/
extern I2C_HandleTypeDef hi2c3;

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;
typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

u8 IIC_Write_1Byte(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
u8 IIC_Read_1Byte(u8 SlaveAddress,u8 REG_Address,u8 *REG_data);
u8 IIC_Write_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);
u8 IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);

#endif
