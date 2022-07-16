#ifndef  __CODER_H__
#define  __CODER_H__
#include "stm32h7xx_hal.h"
#include "main.h"

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Initialization
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
uint8_t encodeDecode_Analysis(uint8_t *inBuf,uint8_t *outBuf,uint16_t Buflen);
void encodeDecode_UWBpacket(uint8_t **inbuf, uint8_t *dist);

#endif


