/*
 * eeprom.h
 *
 *  Created on: 19 мая 2024 г.
 *      Author: Proger
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "FreeRTOS.h"
#include "cmsis_os.h"

#define ADDRES (0xA0<<1)

extern I2C_HandleTypeDef hi2c1;
void writeByteEEPROM(uint8_t byte, uint16_t addr);
void readEEPROM(uint8_t *data[]);

void writeByteEEPROM(uint8_t byte, uint16_t addr){
	uint8_t array[2];
	array[1] = addr;
	array[0] = addr >> 8;
	HAL_I2C_Master_Transmit_IT(&hi2c1, (ADDRES & 0xFE), array , sizeof(array));
	HAL_I2C_Mem_Write(&hi2c1, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout)
}

void readEEPROM(uint8_t *data[]){
	HAL_I2C_Master_Transmit(&hi2c1, ADDRES|0x01, data, sizeof(data), portMAX_DELAY);
}

#endif /* INC_EEPROM_H_ */
