/*
 * terminal_util_360.h
 *
 *  Created on: 21 апр. 2024 г.
 *      Author: Proger
 */

#ifndef INC_TERMINAL_UTIL_360_H_
#define INC_TERMINAL_UTIL_360_H_

#include "main.h"
#include "Modbus.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

#ifdef MYDEBUG
#define PRINT(X) HAL_UART_Transmit_DMA(&huart4, (uint8_t*)X, sizeof(X)-1);
#else
#define PRINT(X)
#endif

enum {
	pr1,
	pr2,
	pr3,
	no_ch,
	check,
	reset
};

extern uint16_t usbHoldingRegister[16];
extern uint8_t usbDiscreteRegister[9];
extern modbusHandler_t ModbusH2;
extern modbus_t robot[2], gateIN[2], gateOUT[2], remote[2]; //Структуры для Modbus Master

const uint16_t impTime = 1500; //Длительность сигнального импульса, мс
uint8_t timeToEntance, timeToExit, gateMode, startDelay, gateOutputFlag;
uint8_t isReady = 1, readyFlag=1, inProgress, isFinish, pause, chasisDisabled, controllerError, carInEntanceGateway, carInExitGateway, carInRobotBay;

int8_t coolerTemp, heaterTemp;
int16_t readyDelay;

/* Definitions for impEntrTimer */
osTimerId_t impEntrTimerHandle;
const osTimerAttr_t impEntrTimer_attributes = {
  .name = "impEntrTimer"
};
/* Definitions for gateINWaitTimer */
osTimerId_t gateINWaitTimerHandle;
const osTimerAttr_t gateINWaitTimer_attributes = {
  .name = "gateINWaitTimer"
};
/* Definitions for impExitTimer */
osTimerId_t impExitTimerHandle;
const osTimerAttr_t impExitTimer_attributes = {
  .name = "impExitTimer"
};
/* Definitions for impRobotTimer */
osTimerId_t impRobotTimerHandle;
const osTimerAttr_t impRobotTimer_attributes = {
  .name = "impRobotTimer"
};
/* Definitions for photoDelayTimer */
osTimerId_t photoDelayTimerHandle;
const osTimerAttr_t photoDelayTimer_attributes = {
  .name = "photoDelayTimer"
};
/* Definitions for gateOUTWaitTimer */
osTimerId_t gateOUTWaitTimerHandle;
const osTimerAttr_t gateOUTWaitTimer_attributes = {
  .name = "gateOUTWaitTimer"
};
/* Definitions for ONcarInTimer */
osTimerId_t ONcarInTimerHandle;
const osTimerAttr_t ONcarInTimer_attributes = {
  .name = "ONcarInTimer"
};
/* Definitions for readyDelayTimer */
osTimerId_t readyDelayTimerHandle;
const osTimerAttr_t readyDelayTimer_attributes = {
  .name = "readyDelayTimer"
};
/* Definitions for PhotoOutDelay */
osTimerId_t PhotoOutDelayHandle;
const osTimerAttr_t PhotoOutDelay_attributes = {
  .name = "PhotoOutDelay"
};

uint8_t dataFromUsbProcessing();
void startProgramm(uint8_t programm);
void controllerReset();
void gateInOpenAuto();
void gateInClosedAuto();
void gateOutOpenAuto();
void gatePhotoHandler();
void gateOutClosedAuto();
void robotHandler();
void robotProcessingInit();
void setGreenTrafficLight();
void setRedTrafficLight();

void impEntrCallback(void *argument);
void gateINWaitCallback(void *argument);
void impExitCallback(void *argument);
void impRobotCallback(void *argument);
void photoDelayCallback(void *argument);
void gateOUTWaitCallback(void *argument);
void ONcarInCallback(void *argument);
void readyDelayCallback(void *argument);
void PhotoOutDelayCallback(void *argument);

void robotProcessingInit(){
	/* creation of impEntrTimer */
	  impEntrTimerHandle = osTimerNew(impEntrCallback, osTimerOnce, NULL, &impEntrTimer_attributes);

	  /* creation of gateINWaitTimer */
	  gateINWaitTimerHandle = osTimerNew(gateINWaitCallback, osTimerOnce, NULL, &gateINWaitTimer_attributes);

	  /* creation of impExitTimer */
	  impExitTimerHandle = osTimerNew(impExitCallback, osTimerOnce, NULL, &impExitTimer_attributes);

	  /* creation of impRobotTimer */
	  impRobotTimerHandle = osTimerNew(impRobotCallback, osTimerOnce, NULL, &impRobotTimer_attributes);

	  /* creation of photoDelayTimer */
	  photoDelayTimerHandle = osTimerNew(photoDelayCallback, osTimerOnce, NULL, &photoDelayTimer_attributes);

	  /* creation of gateOUTWaitTimer */
	  gateOUTWaitTimerHandle = osTimerNew(gateOUTWaitCallback, osTimerOnce, NULL, &gateOUTWaitTimer_attributes);

	  /* creation of ONcarInTimer */
	  ONcarInTimerHandle = osTimerNew(ONcarInCallback, osTimerOnce, NULL, &ONcarInTimer_attributes);

	  /* creation of readyDelayTimer */
	  readyDelayTimerHandle = osTimerNew(readyDelayCallback, osTimerOnce, NULL, &readyDelayTimer_attributes);

	  /* creation of PhotoOutDelay */
	  PhotoOutDelayHandle = osTimerNew(PhotoOutDelayCallback, osTimerOnce, NULL, &PhotoOutDelay_attributes);

}

uint8_t dataFromUsbProcessing(){

	if (usbDiscreteRegister[reset]) {
		usbDiscreteRegister[reset]=0;
//		NVIC_SystemReset();
		controllerReset();
		return reset;
	}else if (usbDiscreteRegister[no_ch]) {
		usbDiscreteRegister[no_ch]=0;
		if(isReady) chasisDisabled = 1;
		return 4;
	}else if (usbDiscreteRegister[pr1]) {
		usbDiscreteRegister[pr1]=0;
		startProgramm(1);
		return 1;
	} else if (usbDiscreteRegister[pr2]) {
		usbDiscreteRegister[pr2]=0;
		startProgramm(2);
		return 2;
	} else if (usbDiscreteRegister[pr3]) {
		usbDiscreteRegister[pr3]=0;
		startProgramm(3);
		return 3;
	}else return 0;
}

void startProgramm(uint8_t programm){

	if (isReady && !inProgress) {
		osDelay(startDelay * 1000);
		inProgress = 1;
		readyFlag = 0;
		if (gateMode == 1 || gateMode == 2) {
			gateInOpenAuto();
		}
		osDelay(3000); //Задержка запуска программы робота
		setRobotNoChasis(chasisDisabled);


	switch (programm) {
			  case 1:
				  PRINT("START PROGRAMM 1\r\n");
				  setRobotProg1(1);
				  osDelay(impTime);
				  setRobotProg1(0);
				  break;
			  case 2:
				  PRINT("START PROGRAMM 2\r\n");
				  setRobotProg2(1);
				  osDelay(impTime);
				  setRobotProg2(0);
				  break;
			  case 3:
				  PRINT("START PROGRAMM 3\r\n");
				  setRobotProg3(1);
				  osDelay(impTime);
				  setRobotProg3(0);
				  break;
			  default:
				  PRINT("INVALID DATA\r\n");
				  controllerReset();
				  break;
				}
	}
}

void robotHandler(){
	/* CODE Обработка сигналов "Автомобиль в боксе" */
	if(!carInRobotBay && getCarIsideStatus() && !osTimerIsRunning(ONcarInTimerHandle)){
		osTimerStart(ONcarInTimerHandle, 2000);
	}
	if(osTimerIsRunning(ONcarInTimerHandle) && !getCarIsideStatus()){
		osTimerStop(ONcarInTimerHandle);
	}
	if(carInRobotBay && !getCarIsideStatus()) {
		carInRobotBay = 0;
	}
	/* END OF CODE Обработка сигналов "Автомобиль в боксе" */

	/* CODE Обработка сигналов "Окончание мойки" */
	if(inProgress && getFinishStatus()) {
		inProgress = 0;
		isFinish = 1;
		if(gateMode == 1 || gateMode == 3) {
			osDelay(3000);
			gateOutOpenAuto();
		}
	}
	/* END OF CODE Обработка сигналов "Окончание мойки" */

	/* CODE Генератор статуса "Готов" */
	if (!controllerError && readyFlag && !carInRobotBay && !pause){ //Переделать!
		isReady = 1;
		setGreenTrafficLight();
	} else {
		isReady = 0;
	}
	/* END OF CODE Генератор статуса "Готов" */

	/* Сетофор */
	if(controllerError || carInRobotBay) setRedTrafficLight();
}

void gatePhotoHandler(){
	/* USER CODE Ворота въезд */
	/* Если флаг не установился, сенсор видит авто в проеме и таймер задержки сенсора не активен то стартуем таймер  */
	if (!carInEntanceGateway && getPhotoInSensor()
			&& !osTimerIsRunning(photoDelayTimerHandle)) {
		if (osTimerIsRunning(gateINWaitTimerHandle))
			osTimerStart(photoDelayTimerHandle, 2000);
	}
	/* Если не видит авто в проеме и таймер задержки сенсора активен то останавливаем таймер  */
	if (osTimerIsRunning(photoDelayTimerHandle) && !getPhotoInSensor()) {
		osTimerStop (photoDelayTimerHandle);
	}
	/* Если флаг установился и проем освободился то останавливаем таймер ожидания авто и закрываем ворота */
	if (carInEntanceGateway && !getPhotoInSensor()) {
		carInEntanceGateway = 0;
		osTimerStop(gateINWaitTimerHandle);
		gateInClosedAuto();
	}
	/* END OF USER CODE Ворота въезд */

	/* USER CODE Ворота выезд */
	/* Если флаг не установился, сенсор видит авто в проеме и таймер задержки сенсора не активен то стартуем таймер  */
	if (!carInExitGateway && getPhotoOutSensor()
			&& !osTimerIsRunning(PhotoOutDelayHandle)) {
		if (osTimerIsRunning(gateOUTWaitTimerHandle))
			osTimerStart(PhotoOutDelayHandle, 2000);
	}
	/* Если не видит авто в проеме и таймер задержки сенсора активен то останавливаем таймер  */
	if (osTimerIsRunning(PhotoOutDelayHandle) && !getPhotoOutSensor()) {
		osTimerStop (PhotoOutDelayHandle);
	}
	/* Если флаг установился и проем освободился то останавливаем таймер  ожидания авто и закрываем ворота */
	if (carInExitGateway && !getPhotoOutSensor()) {
		carInExitGateway = 0;
		osTimerStop(gateOUTWaitTimerHandle);
		gateOutClosedAuto();
		osTimerStart(readyDelayTimerHandle, readyDelay);
	}
	/* END OF USER CODE Ворота выезд */

}

void controllerReset(){
	chasisDisabled=0;
	inProgress = 0;
	controllerError=0;
	readyFlag = 1;
	setRobotNoChasis(chasisDisabled);
	PRINT("CONTROLLER RESET\r\n");
}

void gateInOpenAuto(){
	setGateInOpen(1);
	osTimerStart( impEntrTimerHandle, impTime); //Таймер отключения выхода
	osTimerStart( gateINWaitTimerHandle, (timeToEntance*1000)); //Таймер автозакрытия ворот
}
void gateInClosedAuto(){
	setGateInClosed(1);
	osTimerStart( impEntrTimerHandle, impTime); //Таймер отключения выхода
}

void gateOutOpenAuto(){
	setGateOutOpen(1);
	osTimerStart(gateOUTWaitTimerHandle, (timeToExit*1000));
	osTimerStart(impExitTimerHandle, impTime);
}
void gateOutClosedAuto(){
	setGateOutClosed(1);
	osTimerStart(impExitTimerHandle, impTime);
}

void setGreenTrafficLight(){
	setGateInRed(0);
	setGateInGreen(1);
}

void setRedTrafficLight(){
	setGateInGreen(0);
	setGateInRed(1);
}

/* impEntrCallback function */
void impEntrCallback(void *argument)
{
  /* USER CODE BEGIN impEntrCallback */
	setGateInOpen(0);
	setGateInClosed(0);
  /* USER CODE END impEntrCallback */
}

/* gateINWaitCallback function */
void gateINWaitCallback(void *argument)
{
  /* USER CODE BEGIN gateINWaitCallback */
	controllerError = 1;
	setRobotCansel(1);
	chasisDisabled = 0;
	setRobotNoChasis(chasisDisabled);
	osTimerStart(impRobotTimerHandle, impTime);
	gateInClosedAuto();
  /* USER CODE END gateINWaitCallback */
}

/* impExitCallback function */
void impExitCallback(void *argument)
{
  /* USER CODE BEGIN impExitCallback */
	setGateOutOpen(0);
	setGateOutClosed(0);
  /* USER CODE END impExitCallback */
}

/* impRobotCallback function */
void impRobotCallback(void *argument)
{
  /* USER CODE BEGIN impRobotCallback */
	setRobotCansel(0);
  /* USER CODE END impRobotCallback */
}

/* photoDelayCallback function */
void photoDelayCallback(void *argument)
{
  /* USER CODE BEGIN photoDelayCallback */
	if(getPhotoInSensor()){
		carInEntanceGateway=1;
	}
  /* USER CODE END photoDelayCallback */
}

/* gateOUTWaitCallback function */
void gateOUTWaitCallback(void *argument)
{
  /* USER CODE BEGIN gateOUTWaitCallback */
	controllerError = 1;
	setRobotCansel(1);
	osTimerStart(impRobotTimerHandle, impTime);
	gateOutClosedAuto();
  /* USER CODE END gateOUTWaitCallback */
}

/* ONcarInCallback function */
void ONcarInCallback(void *argument)
{
  /* USER CODE BEGIN ONcarInCallback */
	chasisDisabled = 0;
	setRobotNoChasis(chasisDisabled);
	carInRobotBay = 1;
	setRedTrafficLight();
  /* USER CODE END ONcarInCallback */
}

/* readyDelayCallback function */
void readyDelayCallback(void *argument)
{
  /* USER CODE BEGIN readyDelayCallback */
	readyFlag = 1;
  /* USER CODE END readyDelayCallback */
}

/* PhotoOutDelayCallback function */
void PhotoOutDelayCallback(void *argument)
{
  /* USER CODE BEGIN PhotoOutDelayCallback */
	if (getPhotoOutSensor()) {
		carInExitGateway = 1;
	}
  /* USER CODE END PhotoOutDelayCallback */
}

#endif /* INC_TERMINAL_UTIL_360_H_ */
