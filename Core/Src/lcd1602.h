/*
 * lcd1602.h
 *
 *  Created on: 27 июн. 2024 г.
 *      Author: Proger
 */

#ifndef SRC_LCD1602_H_
#define SRC_LCD1602_H_

#define LCD_ADDR (0x27 << 1)       // адрес дисплея, сдвинутый на 1 бит влево (HAL работает с I2C-адресами, сдвинутыми на 1 бит влево)
#define PIN_RS    (1 << 0)         // если на ножке 0, данные воспринимаются как команда, если 1 - как символы для вывода
#define PIN_EN    (1 << 2)         // бит, по изменению сост. которого считывается информация
#define BACKLIGHT (1 << 3)         // управление подсветкой
#define LCD_DELAY_MS 2

extern I2C_HandleTypeDef hi2c1;

void I2C_send(uint8_t data, uint8_t flags);

void LCD_Init(){
	I2C_send(0b00110000, 0);   // 8ми битный интерфейс
	I2C_send(0b00000010, 0);   // установка курсора в начале строки
	I2C_send(0b00001100, 0);   // нормальный режим работы, выкл курсор
	I2C_send(0b00000001, 0);   // очистка дисплея
	I2C_send(0b10000000, 0);   // переход на 1 строку
}

void I2C_send(uint8_t data, uint8_t flags) {
	HAL_StatusTypeDef res;

	for (;;) {                                               // бесконечный цикл
		res = HAL_I2C_IsDeviceReady(&hi2c1, LCD_ADDR, 1, HAL_MAX_DELAY); // проверяем, готово ли устройство по адресу lcd_addr для связи
		if (res == HAL_OK)
			break;                  // если да, то выходим из бесконечного цикла
	}

	uint8_t up = data & 0xF0; // операция  с 1111 0000, приводит к обнулению последних бит с 0 по 3, остаются биты с 4 по 7
	uint8_t lo = (data << 4) & 0xF0; // тоже самое, но data сдвигается на 4 бита влево, т.е. в этой
									 // переменной остаются  биты с 0 по 3
	uint8_t data_arr[4];
	data_arr[0] = up | flags | BACKLIGHT | PIN_EN; // 4-7 биты содержат информацию, биты 0-3 конфигурируют работу
	data_arr[1] = up | flags | BACKLIGHT; // ублирование сигнала, на выводе Е в этот раз 0
	data_arr[2] = lo | flags | BACKLIGHT | PIN_EN;
	data_arr[3] = lo | flags | BACKLIGHT;

	HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data_arr, sizeof(data_arr), HAL_MAX_DELAY);
	//HAL_I2C_Master_Transmit_DMA(&hi2c1, LCD_ADDR, data_arr, sizeof(data_arr));
	HAL_Delay (LCD_DELAY_MS);
}
void LCD_SendString(char *str) {
	// *char по сути является строкой
	while (*str) {                                 // пока строчка не закончится
		I2C_send((uint8_t)(*str), 1);        // передача первого символа строки
		str++;                                // сдвиг строки налево на 1 символ
	}
}


#endif /* SRC_LCD1602_H_ */
