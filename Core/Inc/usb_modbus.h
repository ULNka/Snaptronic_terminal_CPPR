/*
 * usb_modbus.h
 *
 *  Created on: 30 апр. 2023 г.
 *      Author: Proger
 */

#ifndef SRC_USB_MODBUS_H_
#define SRC_USB_MODBUS_H_

#define OWEN_OFFSET 512
#define USB_ID 0x1F
#define MAX_USB_BUFF_SIZE 32

//#include "modbus_util.h"
//#include "usbd_cdc_if.h"
uint8_t usbRxBuffer[MAX_USB_BUFF_SIZE], usbTxBuffer[MAX_USB_BUFF_SIZE]; //Буферы данных USB
uint16_t usbHoldingRegister[16]; //Массив Регистров хранения (HoldingRegister)
uint8_t usbDiscreteRegister[10]; //Массив дискретных регистров
uint16_t usbRxBufferSize;	//Размер принятых данных

uint16_t modbusSlaveCRC;
uint8_t crcArray[2];
uint8_t command;

extern UART_HandleTypeDef huart1;  //Дескриптор порта данных
//extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
void usbBufferClear();

uint16_t modbusCalcCRC(uint8_t bufferArray[], uint16_t length);
uint8_t checkSlaveCRC();
void modbusCRCtoArray(uint16_t crc);
void sendEcho();
void writeHoldigRegister();
void readDiscreteRegisterOwen();
void writeDiscreteRegisterOwen();
void sendError(uint8_t);
void bitSwapper();
void readHoldingRegister();
void usbBufferClear();

enum ErrorCode {
	empty,
	invalidFunctions,
	invalidAddress,
	invalidData,
	another,
	deviceBusy
};



void usbModbusProcessing() {

	if (usbRxBufferSize > 0 && usbRxBuffer[0] == USB_ID	&& checkSlaveCRC() == 1) {

		command = usbRxBuffer[1];
		switch (command) {
		case 0x02:
			bitSwapper();
			readDiscreteRegisterOwen();
			break;
		case 0x03:
			readHoldingRegister();
			break;
		case 0x04:
			readHoldingRegister();
			break;
		case 0x05:
//			sendEcho(); //эхо вначале, тк свапаем байты в регистрах(овен эмулятор)
			bitSwapper();
			writeDiscreteRegisterOwen();
			break;
		case 0x06:
			writeHoldigRegister();
			sendEcho();
			break;
		default:
			sendError(invalidFunctions);
			break;
		}
		usbBufferClear();
	}
}

void modbusCRCtoArray(uint16_t crc) { // Разбиваем CRC на двухбайтовый массив
	crcArray[1] = crc;
	crcArray[0] = crc >> 8;
}

uint8_t checkSlaveCRC() { // Проверяем входящие данные на корректность CRC
	modbusSlaveCRC = 0x0000;
	modbusSlaveCRC |= usbRxBuffer[usbRxBufferSize - 2];
	modbusSlaveCRC <<= 8;
	modbusSlaveCRC |= usbRxBuffer[usbRxBufferSize - 1];
	if (modbusCalcCRC(usbRxBuffer, usbRxBufferSize - 2) == modbusSlaveCRC) {
		return 1;
	} else
		return 0;
}

void sendEcho() {
//	CDC_Transmit_FS(usbRxBuffer, usbRxBufferSize);
	HAL_UART_Transmit_IT(&huart1, usbRxBuffer, usbRxBufferSize);
}

void readHoldingRegister() {
	uint16_t firstRegister = 0;
	uint16_t numberOfregisters = 0;
	uint8_t j = 3;
	for (uint8_t i = 0; i < 16; i++) {
		usbTxBuffer[i] = 0;
	}
	firstRegister = usbRxBuffer[3] | (usbRxBuffer[2] << 8);
	numberOfregisters = usbRxBuffer[5] | (usbRxBuffer[4] << 8);
	if (firstRegister + numberOfregisters > sizeof(usbHoldingRegister)) {
		sendError(invalidAddress);
		return;
	}
	usbTxBuffer[0] = USB_ID;
	usbTxBuffer[1] = command;
	usbTxBuffer[2] = numberOfregisters * 2;

	for (uint16_t i = firstRegister; i < (firstRegister + numberOfregisters);
			i++) {
		usbTxBuffer[j] |= usbHoldingRegister[i] >> 8;
		usbTxBuffer[j + 1] |= usbHoldingRegister[i];
		j += 2;
	}

	modbusCRCtoArray(modbusCalcCRC(usbTxBuffer, numberOfregisters + 4));
	usbTxBuffer[usbTxBuffer[2] + 3] = crcArray[0];
	usbTxBuffer[usbTxBuffer[2] + 4] = crcArray[1];
//	CDC_Transmit_FS(usbTxBuffer, usbTxBuffer[2] + 5);
	HAL_UART_Transmit_IT(&huart1, usbTxBuffer, usbTxBuffer[2] + 5);


}

void writeHoldigRegister() {
	uint16_t reg = 0;
	reg = usbRxBuffer[3] | (usbRxBuffer[2] << 8);
	if (reg > sizeof(usbHoldingRegister)) {
		sendError(invalidAddress);
		return;
	}
	usbHoldingRegister[reg] = 0x0000;
	usbHoldingRegister[reg] |= usbRxBuffer[4];
	usbHoldingRegister[reg] <<= 8;
	usbHoldingRegister[reg] |= usbRxBuffer[5];
}

void readDiscreteInputs() {
	uint16_t registerNumber = 0;
	uint16_t numberOfRegisters = 0;
	uint8_t numberOfBytes = 0;
	uint8_t j = 3;
	registerNumber = usbRxBuffer[3] | (usbRxBuffer[2] << 8);
	numberOfRegisters = usbRxBuffer[5] | (usbRxBuffer[4] << 8);
	if (registerNumber + numberOfRegisters > sizeof(usbDiscreteRegister)) {
		sendError(invalidAddress);
		return;
	}
	usbTxBuffer[0] = USB_ID;
	usbTxBuffer[1] = command;
	if (numberOfRegisters % 8 == 0) {
		numberOfBytes = numberOfRegisters / 8;
	} else {
		numberOfBytes = (numberOfRegisters / 8) + 1;
	}
	usbTxBuffer[2] = numberOfBytes;
	for (uint16_t i = registerNumber; i <= numberOfBytes; i++) {
		usbTxBuffer[j] = usbDiscreteRegister[i];
		j++;
	}
	modbusCRCtoArray(modbusCalcCRC(usbTxBuffer, numberOfBytes + 3));
	usbTxBuffer[numberOfBytes + 3] = crcArray[0];
	usbTxBuffer[numberOfBytes + 4] = crcArray[1];
//	CDC_Transmit_FS(usbTxBuffer, numberOfBytes + 5);
	HAL_UART_Transmit_IT(&huart1, usbTxBuffer, numberOfBytes + 5);

}

void readDiscreteRegisterOwen() {
	uint16_t registerNumber = 0; // Переменная для хранения номера регистра
	uint16_t numberOfRegisters = 0;	//Переменная для хранения количества запрашиваемых реистров

	registerNumber = usbRxBuffer[3] | (usbRxBuffer[2] << 8);//соединяем два uint8 в один uint16
	if (registerNumber < OWEN_OFFSET) { //Проверяем адрес регистра
		sendError(invalidAddress);
			return;
		}
	registerNumber -= OWEN_OFFSET; //Для эмуляции овен ПР200 отнимаем смещение 512

	numberOfRegisters = usbRxBuffer[5] | (usbRxBuffer[4] << 8); //соединяем два uint8 в один uint16
//	HAL_UART_Transmit(&huart4, usbRxBuffer[5], 2, HAL_MAX_DELAY);

//	PRINT("DATA " + numberOfRegisters);
	if (registerNumber + numberOfRegisters > 16) {
		sendError(invalidAddress);
		return;
	}
	usbTxBuffer[0] = USB_ID;
	usbTxBuffer[1] = command;
	usbTxBuffer[2] = numberOfRegisters * 2; // количество байт в ответе

	for (int8_t i = 0; i < usbTxBuffer[2]; i += 2) { // Данные ответа
		usbTxBuffer[i + 3] = usbDiscreteRegister[registerNumber];
		usbTxBuffer[i + 4] = 0x00;
		registerNumber++;
	}

	modbusCRCtoArray(modbusCalcCRC(usbTxBuffer, usbTxBuffer[2] + 3)); //Считаем контрольку
	usbTxBuffer[usbTxBuffer[2] + 3] = crcArray[0];
	usbTxBuffer[usbTxBuffer[2] + 4] = crcArray[1];
//	CDC_Transmit_FS(usbTxBuffer, usbTxBuffer[2] + 5); //Шлем ответ
	HAL_UART_Transmit_IT(&huart1, usbTxBuffer, usbTxBuffer[2] + 5);

}

void writeDiscreteRegisterOwen() {
	uint16_t registerNumber = 0;
	registerNumber = usbRxBuffer[3] | (usbRxBuffer[2] << 8);
	if (registerNumber < OWEN_OFFSET) { //Проверяем адрес регистра
		sendError(invalidAddress);
		return;
	}
	registerNumber -= OWEN_OFFSET; //Для эмуляции овен ПР200
	if (registerNumber > sizeof(usbDiscreteRegister)) {
		sendError(invalidAddress);
		return;
	}
	if (usbRxBuffer[4] == 0xFF && usbRxBuffer[5] == 0x00 ) {
		usbDiscreteRegister[registerNumber] = 0x10;
	} else if (usbRxBuffer[4] == 0x00 && usbRxBuffer[5] == 0x00) {
		usbDiscreteRegister[registerNumber] = 0x00;
	}
	else {
		sendError(invalidData);
		return;
	}
	bitSwapper();
	sendEcho();
}

void sendError(uint8_t error) {
	usbTxBuffer[0] = USB_ID;
	usbTxBuffer[1] = command | 0x80;
	usbTxBuffer[2] = error;
	modbusCRCtoArray(modbusCalcCRC(usbTxBuffer, 3));
	usbTxBuffer[3] = crcArray[0];
	usbTxBuffer[4] = crcArray[1];
//	CDC_Transmit_FS(usbTxBuffer, 5);
	HAL_UART_Transmit_IT(&huart1, usbTxBuffer, 5);

}



void bitSwapper() {
	uint8_t temp = 0;
	for (int8_t j = 2; j < 6; j++) {
		temp |= (usbRxBuffer[j] << 4);
		temp |= (usbRxBuffer[j] >> 4);
		usbRxBuffer[j] = temp;
		temp = 0;
	}
//	void bitSwapper((uint8_t)& buff[]) {
//		uint8_t temp = 0;
//		for (int8_t j = 2; j < 4; j++) {
//			temp |= (buff[j] << 4);
//			temp |= (buff[j] >> 4);
//			buff[j] = temp;
//			temp = 0;
//		}
}

uint16_t modbusCalcCRC(uint8_t bufferArray[], uint16_t length) { // Вычисление контрольной суммы ModBus CRC16
	uint16_t crc = 0xFFFF;

	  for (int pos = 0; pos < length; pos++)
	  {
	    crc ^= bufferArray[pos];          // XOR byte into least sig. byte of crc

	    for (int i = 8; i != 0; i--) {    // Loop over each bit
	      if ((crc & 0x0001) != 0) {      // If the LSB is set
	        crc >>= 1;                    // Shift right and XOR 0xA001
	        crc ^= 0xA001;
	      }
	      else                            // Else LSB is not set
	        crc >>= 1;                    // Just shift right
	    }
	  }
/* Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes) */
	  return (uint16_t)((crc >> 8) | (crc << 8));
//	  return crc;
}
void usbBufferClear() {
	for (uint8_t i = 0; i < 16; i++) {
		usbRxBuffer[i] = 0;
		usbTxBuffer[i] = 0;
	}
	usbRxBufferSize = 0;
	crcArray[1] = 0;
	crcArray[0] = 0;
}
#endif /* SRC_USB_MODBUS_H_ */
