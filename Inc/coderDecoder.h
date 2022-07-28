#ifndef  __CODER_H__
#define  __CODER_H__
#include "stm32f4xx_hal.h"
#include "main.h"
#include "stdbool.h"

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Initialization
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
uint8_t encodeDecode_Analysis(uint8_t *inBuf,uint8_t *outBuf,uint16_t Buflen);
bool encodeDecode_Analysis_UWB(uint8_t *inBuf,float *outBuf,uint16_t Buflen);
void encodeDecode_UWBpacket(uint8_t inbuf[][20], float *dist);
bool encodeDecode_Analysis_SecondBoard(uint8_t *inBuf, uint16_t Buflen);
#endif


