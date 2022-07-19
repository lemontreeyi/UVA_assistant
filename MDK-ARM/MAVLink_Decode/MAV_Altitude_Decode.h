#ifndef __MAV_ALTITUDE_DECODE_H__
#define __MAV_ALTITUDE_DECODE_H__
#include "stm32f4xx_hal.h"
#include "mavlink.h"
#include "mavlink_types.h"
int Mav_Altitude_Decoder(uint16_t rxlen, mavlink_message_t msg, uint8_t *buf, float* height);
#endif
