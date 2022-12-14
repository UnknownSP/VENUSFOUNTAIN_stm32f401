/*
 * MW_ENCODER.h
 *
 *  Created on: 2016/06/12
 *      Author: evaota
 */

#ifndef __MW_ENCODER_H_
#define __MW_ENCODER_H_

#include "stm32f4xx_hal.h"
#include <stdlib.h>

typedef enum
  {
    ENCODER1ID = 0,
    ENCODER2ID = 1,
    ENCODER3ID = 2,
    ENCODER4ID = 3
  }encoderid_t;

/******************************* TIM ID *******************************/
#define IS_ENCODER_ID(ID) (((ID) == ENCODER1ID) ||	\
		       ((ID) == ENCODER2ID)) ||	\
           ((ID) == ENCODER3ID)) ||	\
           ((ID) == ENCODER4ID)) 

int MW_EncoderInit(encoderid_t id);
int16_t MW_GetEncoderVal(encoderid_t id);
void  MW_WriteEncoderVal(encoderid_t id,int16_t val);

#endif /* MIDLEWARE_INC_MW_ENCODER_H_ */
