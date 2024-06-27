/*
 * ws2812.h
 *
 *  Created on: 8 июн. 2024 г.
 *      Author: Proger
 */

#ifndef SRC_WS2812_H_
#define SRC_WS2812_H_
//--------------------------------------------------

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

//--------------------------------------------------
#define DELAY_LEN 48
#define LED_COUNT 72
#define ARRAY_LEN DELAY_LEN + LED_COUNT*24
#define HIGH 65
#define LOW 26
#define BitIsSet(reg, bit) ((reg & (1<<bit)) != 0)
//--------------------------------------------------


#endif /* SRC_WS2812_H_ */
