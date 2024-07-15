/*
 * terminal_util_DG.h
 *
 *  Created on: 27 июн. 2024 г.
 *      Author: Proger
 */

#ifndef INC_TERMINAL_UTIL_DG_H_
#define INC_TERMINAL_UTIL_DG_H_

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
extern uint8_t usbDiscreteRegister[10];
extern modbusHandler_t ModbusH2;
extern modbus_t robot[2], gateIN[2], gateOUT[2], remote[2]; //Структуры для Modbus Master
extern uint8_t lightEffect;
const uint16_t impTime = 1500; //Длительность сигнального импульса, мс
uint8_t timeToEntance=31, timeToExit, gateMode, startDelay, gateOutputFlag;
uint8_t isReady = 1, readyFlag=1, inProgress, isFinish, pause, chasisDisabled, techBreack, controllerError, carInEntanceGateway, carInExitGateway, carInRobotBay;
uint8_t carInDGFalg, robotPhotoIsBlack=0;
int8_t coolerTemp, heaterTemp;
int16_t readyDelay;
uint32_t carInTimer, beepTimer;
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
void setOFFTrafficLight();

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
		lightEffect = 4;
		if (gateMode == 1 || gateMode == 2) {
			gateInOpenAuto();

		}
		osDelay(3000); //Задержка запуска программы робота
		setRobotNoChasis(chasisDisabled);


	switch (programm) {
			  case 1:
//				  PRINT("START PROGRAMM 1\r\n");
				  setRobotProg1(1);
				  osDelay(impTime);
				  setRobotProg1(0);
				  break;
			  case 2:
//				  PRINT("START PROGRAMM 2\r\n");
				  setRobotProg2(1);
				  osDelay(impTime);
				  setRobotProg2(0);
				  break;
			  case 3:
//				  PRINT("START PROGRAMM 3\r\n");
				  setRobotProg3(1);
				  osDelay(impTime);
				  setRobotProg3(0);
				  break;
			  default:
//				  PRINT("INVALID DATA\r\n");
				  controllerReset();
				  break;
				}
	}
}

void robotHandler(){
	/* CODE Обработка сигналов "Автомобиль в боксе" */
	if(!carInRobotBay && getCarIsideStatus() && !osTimerIsRunning(ONcarInTimerHandle)){
		osTimerStart(ONcarInTimerHandle, 2000);  //Защитный интервал фотоэлемета, по истечению ставим статус "авто в боксе" carInRobotBay = 1 при условии флага ожидания carInDGFalg = 1
	}
	if(osTimerIsRunning(ONcarInTimerHandle) && !getCarIsideStatus()){
		osTimerStop(ONcarInTimerHandle);
	}
	if(robotPhotoIsBlack && !getCarIsideStatus() ) {
		robotPhotoIsBlack = 0;
//		carInRobotBay = 0;
	}

	if(carInDGFalg && (HAL_GetTick()-carInTimer >= 60000)) carInDGFalg = 0; // Таймаут автосброса флага ожидания авто в боксе

	//if(inProgress && getCarIsideStatus()) carInRobotBay = 1; // Зачем я это сделал?? Если не сработал фотоэлемент. Пока без этого
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
	if(controllerError){
		isReady = 0;
		setRedTrafficLight();
		lightEffect = 7;
	}
	if(techBreack){
		isReady = 0;
		setOFFTrafficLight();
		lightEffect = 3;
	}else if(robotPhotoIsBlack){
		isReady = 0;
		setRedTrafficLight();
		lightEffect = 2;
	}else if (!controllerError && readyFlag && !carInRobotBay && !pause){ //Переделать!
		isReady = 1;
		setGreenTrafficLight();
		lightEffect = 1;
	} else {
		isReady = 0;
//		lightEffect = 2;
		if(!inProgress) setRedTrafficLight();
	}
#ifdef REMOTE
	if(controllerError){
		static uint8_t a;
		if(HAL_GetTick()-beepTimer >= 800){
			beepTimer = HAL_GetTick();
			a=!a;
			setErrLamp(a);
			setAlarmSound(a);
		}
	}else if(techBreack){
		setErrLamp(1);
		setAlarmSound(0);
	} else {
		setErrLamp(0);
		setAlarmSound(0);
	}
#endif
	/* END OF CODE Генератор статуса "Готов" */
setRobotGatesIsOpen((getClosedInSw() || getClosedOutSw())); //Сигнал открытых ворот для вентиляции
	/* Сетофор */
//	if(controllerError || carInRobotBay) setRedTrafficLight();
}

void gatePhotoHandler(){
	/* USER CODE Ворота въезд */
	/* Если флаг не установился, сенсор видит авто в проеме и таймер задержки сенсора не активен то стартуем таймер  */
	if (!carInEntanceGateway && getPhotoInSensor() && !osTimerIsRunning(photoDelayTimerHandle)) {
//		if (osTimerIsRunning(gateINWaitTimerHandle))
			osTimerStart(photoDelayTimerHandle, 2000);
	}
	/* Если не видит авто в проеме и таймер задержки сенсора активен то останавливаем таймер  */
	if (osTimerIsRunning(photoDelayTimerHandle) && !getPhotoInSensor()) {
		osTimerStop (photoDelayTimerHandle);
	}
	/* Если флаг установился и проем освободился то останавливаем таймер ожидания авто и закрываем ворота */
	if (carInEntanceGateway && !getPhotoInSensor()) {
		carInEntanceGateway = 0;

		if(osTimerIsRunning(gateINWaitTimerHandle)){
		osTimerStop(gateINWaitTimerHandle);
		osDelay(1000);
		gateInClosedAuto();
		}
	}
	/* END OF USER CODE Ворота въезд */

	/* USER CODE Ворота выезд */
	/* Если флаг не установился, сенсор видит авто в проеме и таймер задержки сенсора не активен то стартуем таймер  */
	if (!carInExitGateway && getPhotoOutSensor() && !osTimerIsRunning(PhotoOutDelayHandle)) {
//		if (osTimerIsRunning(gateOUTWaitTimerHandle))
			osTimerStart(PhotoOutDelayHandle, 2000);
	}
	/* Если не видит авто в проеме и таймер задержки сенсора активен то останавливаем таймер  */
	if (osTimerIsRunning(PhotoOutDelayHandle) && !getPhotoOutSensor()) {
		osTimerStop (PhotoOutDelayHandle);
	}
	/* Если флаг установился и проем освободился то останавливаем таймер  ожидания авто и закрываем ворота */
	if (carInExitGateway && !getPhotoOutSensor()) {
		carInExitGateway = 0;
		if(carInRobotBay){
		carInRobotBay = 0;
		}
		if(osTimerIsRunning(gateOUTWaitTimerHandle)){
		osTimerStop(gateOUTWaitTimerHandle);
		osDelay(1000);
		gateOutClosedAuto();
		}
		if(isFinish) osTimerStart(readyDelayTimerHandle, readyDelay);

	}
	/* END OF USER CODE Ворота выезд */

}

void remoteHandler(){
	static uint8_t resetFalg, gateInButtonOpenFlag, gateInButtonCloseFlag, gateOutButtonOpenFlag, gateOutButtonCloseFlag, handBoxButtonFlag;
	static uint32_t resetButtonTimer, handBoxTimer;
	/* Индикаторы концевиков ворот */
setInGreen(getClosedInSw());
setOutGreen(getClosedOutSw());

	/* Установка/снятие паузы */
/*
if(!pauseFalg && getPauseButton()){
	pauseFalg = 1;
	pause = !pause;
}
if(pauseFalg && !getPauseButton()){
	pauseFalg = 0;
}
*/
pause = getPauseButton();

	/* Кнопка сброса ошибки и режима перерыва */
if (!resetFalg && getResetButton()){
	resetButtonTimer = HAL_GetTick();
	resetFalg = 1;
}
if(resetFalg && getResetButton() && (HAL_GetTick() - resetButtonTimer >= 3000)){
	techBreack = 1;
}
if(resetFalg && !getResetButton() && (HAL_GetTick() - resetButtonTimer < 3000)){
	controllerReset();
	resetFalg=0;
}

/* кнопки управления воротами */
	/* вьезд открыть*/
if(!gateInButtonOpenFlag && getEntranceOpenButton()){
	gateInButtonOpenFlag = 1;
	setGateInOpen(1);
}
if(gateInButtonOpenFlag && !getEntranceOpenButton()){
	gateInButtonOpenFlag = 0;
	setGateInOpen(0);
}
	/* вьезд закрыть*/
if(!gateInButtonCloseFlag && getEntranceCloseButton()){
	gateInButtonCloseFlag = 1;
	setGateInClosed(1);
}
if(gateInButtonCloseFlag && !getEntranceCloseButton()){
	gateInButtonCloseFlag = 0;
	setGateInClosed(0);
}

	/* выезд открыть */
if(!gateOutButtonOpenFlag && getExitOpenButton()){
	gateOutButtonOpenFlag = 1;
	setGateOutOpen(1);
}
if(gateOutButtonOpenFlag && !getExitOpenButton()){
	gateOutButtonOpenFlag = 0;
	setGateOutOpen(0);
}

	/* выезд закрыть */
if(!gateOutButtonCloseFlag && getExitCloseButton()){
	gateOutButtonCloseFlag = 1;
	setGateOutClosed(1);
}
if(gateOutButtonCloseFlag && !getExitCloseButton()){
	gateOutButtonCloseFlag = 0;
	setGateOutClosed(0);
}
/* Переключатель режима ворот */
#ifdef REMOTE
if(getGateInAutoMode() && getGateOutAutoMode()) gateMode = 1;
else if(getGateInAutoMode() && !getGateOutAutoMode()) gateMode = 2;
else if(!getGateInAutoMode() && getGateOutAutoMode()) gateMode = 3;
else gateMode = 0;
#else
gateMode = 1;
#endif

/* кнопка управления светофором ручной мойки */
	if (!handBoxButtonFlag && getTrafficLightButton()) {
		handBoxTimer = HAL_GetTick();
		handBoxButtonFlag = 1;
	}

	if (handBoxButtonFlag && getTrafficLightButton() && (HAL_GetTick() - handBoxTimer >= 3000)) {
		handBoxButtonFlag = 1;
		setHandBoxRemote(1);
		setGateInHandGreen(0);
		setGateInHandRed(0);
//		setHandRed(0);
//		setHandGreen(0);
	}

	if (handBoxButtonFlag && !getTrafficLightButton() && (HAL_GetTick() - handBoxTimer < 3000)) {
		handBoxButtonFlag = 0;
		if (getHandBoxGreen()) {
			setHandBoxRemote(1);
			setGateInHandGreen(0);
			setGateInHandRed(1);
		} else {
			setHandBoxRemote(0);
			setGateInHandGreen(1);
		}
	}
	if(handBoxButtonFlag && !getTrafficLightButton()) handBoxButtonFlag=0;

	setHandGreen(getHandBoxGreen());
	setHandRed(getHandBoxRed());
}

void controllerReset(){
	pause = 0;
	chasisDisabled=0;
	inProgress = 0;
	controllerError=0;
	readyFlag = 1;
	techBreack=0;
	carInDGFalg=0;
	osTimerStop(gateOUTWaitTimerHandle);
	osTimerStop(gateINWaitTimerHandle);
	setRobotNoChasis(chasisDisabled);
//	PRINT("CONTROLLER RESET\r\n");
}

void gateInOpenAuto(){
	setGateInOpen(1);
	osTimerStart( impEntrTimerHandle, impTime); //Таймер отключения выхода
	osTimerStart( gateINWaitTimerHandle, (timeToEntance*1000)); //Таймер автозакрытия вороти по таймауту с генерацией ошибки
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
//#ifdef REMOTE
	setRobotRed(0);
	setRobotGreen(1);
//#endif
}

void setRedTrafficLight(){
	setGateInGreen(0);
	setGateInRed(1);
//#ifdef REMOTE
	setRobotGreen(0);
	setRobotRed(1);
//#endif
}

void setOFFTrafficLight(){
	setGateInGreen(0);
	setGateInRed(0);
#ifdef REMOTE
	setRobotGreen(0);
	setRobotRed(0);
#endif
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
void gateINWaitCallback(void *argument)	//Таймаут ожидания заезда авто
{
  /* USER CODE BEGIN gateINWaitCallback */
	controllerError = 1;
	lightEffect = 3;
	setRobotCansel(1);
	osTimerStart(impRobotTimerHandle, impTime);
	chasisDisabled = 0;
	setRobotNoChasis(chasisDisabled);
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
		carInDGFalg=1;
		carInTimer = HAL_GetTick();
		carInEntanceGateway=1;
		if(inProgress) {
			lightEffect = 2;
			setRedTrafficLight();
		}
	}
  /* USER CODE END photoDelayCallback */
}

/* gateOUTWaitCallback function */
void gateOUTWaitCallback(void *argument)		//Таймаут ожидания выезда авто
{
  /* USER CODE BEGIN gateOUTWaitCallback */
	controllerError = 1;
	lightEffect = 3;
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
	robotPhotoIsBlack = 1;
	if(carInDGFalg)	{
		carInDGFalg = 0;
		carInRobotBay = 1;
	}
	lightEffect = 2;
//	setRedTrafficLight();
  /* USER CODE END ONcarInCallback */
}

/* readyDelayCallback function */
void readyDelayCallback(void *argument)
{
  /* USER CODE BEGIN readyDelayCallback */
	readyFlag = 1;
	isFinish = 0;
//	lightEffect = 1;
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


#endif /* INC_TERMINAL_UTIL_DG_H_ */
