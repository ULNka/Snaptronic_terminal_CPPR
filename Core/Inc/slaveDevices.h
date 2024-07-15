/*
 * slaveDevices.h
 *
 *  Created on: 5 мая 2024 г.
 *      Author: Proger
 */

#ifndef INC_SLAVEDEVICES_H_
#define INC_SLAVEDEVICES_H_

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

uint16_t robotInputs[1], gateINInputs[1], gateOUTInputs[1], remoteInputs[1], robotOutputs[1], gateINOutputs[1], gateOUTOutnputs[1], remoteOutputs[1];

uint8_t getCarIsideStatus();
uint8_t getFinishStatus();
void setRobotProg1(uint8_t val);
void setRobotProg2(uint8_t val);
void setRobotProg3(uint8_t val);
void setRobotProg4(uint8_t val);
void setRobotNoChasis(uint8_t val);
void setRobotCansel(uint8_t val);
void setRobotGatesIsOpen(uint8_t val);
void setRobotNoChasis(uint8_t val);

/* Модуль робота */
uint8_t getCarIsideStatus(){
	 return bitRead(robotInputs[0], 0);
}
uint8_t getFinishStatus(){
	 return bitRead(robotInputs[0], 1);
}
void setRobotProg1(uint8_t val){
	bitWrite(robotOutputs[0], 0, val);
}
void setRobotProg2(uint8_t val){
	bitWrite(robotOutputs[0], 1, val);
}
void setRobotProg3(uint8_t val){
	bitWrite(robotOutputs[0], 2, val);
}
void setRobotProg4(uint8_t val){
	bitWrite(robotOutputs[0], 3, val);
}
void setRobotNoChasis(uint8_t val){
	bitWrite(robotOutputs[0], 4, val);
}
void setRobotCansel(uint8_t val){
	bitWrite(robotOutputs[0], 5, val);
}
void setRobotGatesIsOpen(uint8_t val){
	bitWrite(robotOutputs[0], 6, val);
}


/* Модуль ворот въезд */
uint8_t getPhotoInSensor(){
	 return bitRead(gateINInputs[0], 0);
}
uint8_t getOpenInSw(){
	 return bitRead(gateINInputs[0], 1);
}
uint8_t getClosedInSw(){
	 return bitRead(gateINInputs[0], 2);
}
uint8_t getPhotoHandBox(){
	 return bitRead(gateINInputs[0], 3);
}
uint8_t getHandBoxGreen(){
	 return bitRead(gateINInputs[0], 8);
}
uint8_t getHandBoxRed(){
	 return bitRead(gateINInputs[0], 9);
}
void setGateInOpen(uint8_t val){
	bitWrite(gateINOutputs[0], 0, val);
}
void setGateInClosed(uint8_t val){
	bitWrite(gateINOutputs[0], 1, val);
}
void setGateInGreen(uint8_t val){
	bitWrite(gateINOutputs[0], 2, val);
}
void setGateInRed(uint8_t val){
	bitWrite(gateINOutputs[0], 3, val);
}
void setGateInStop(uint8_t val){
	bitWrite(gateINOutputs[0], 4, val);
}
void setGateInHandGreen(uint8_t val){
	bitWrite(gateINOutputs[0], 5, val);
}
void setGateInHandRed(uint8_t val){
	bitWrite(gateINOutputs[0], 6, val);
}
void setHandBoxRemote(uint8_t val){
	bitWrite(gateINOutputs[0], 8, val);
}

/* Модуль ворот выезд */
uint8_t getPhotoOutSensor(){
	 return bitRead(gateOUTInputs[0], 0);
}
uint8_t getClosedOutSw(){
	 return bitRead(gateOUTInputs[0], 2);
}
uint8_t getOpenOutSw(){
	 return bitRead(gateOUTInputs[0], 1);
}
void setGateOutOpen(uint8_t val){
	bitWrite(gateOUTOutnputs[0], 0, val);
}
void setGateOutClosed(uint8_t val){
	bitWrite(gateOUTOutnputs[0], 1, val);
}
void setGateOutStop(uint8_t val){
	bitWrite(gateOUTOutnputs[0], 2, val);
}

/* Модуль пульта */
uint8_t getPauseButton(){
	 return bitRead(remoteInputs[0], 0);
}
uint8_t getExitOpenButton(){
	 return bitRead(remoteInputs[0], 1);
}
uint8_t getExitCloseButton(){
	 return bitRead(remoteInputs[0], 2);
}
uint8_t getResetButton(){
	 return bitRead(remoteInputs[0], 3);
}
uint8_t getGateInAutoMode(){
	 return bitRead(remoteInputs[0], 4);
}
uint8_t getGateOutAutoMode(){
	 return bitRead(remoteInputs[0], 5);
}
uint8_t getEntranceOpenButton(){
	 return bitRead(remoteInputs[0], 6);
}
uint8_t getEntranceCloseButton(){
	 return bitRead(remoteInputs[0], 7);
}
uint8_t getTrafficLightButton(){
	 return bitRead(remoteInputs[0], 8);
}

void setOutGreen(uint8_t val){
	bitWrite(remoteOutputs[0], 0, val);
}
void setErrLamp(uint8_t val){
	bitWrite(remoteOutputs[0], 1, val);
}
void setInGreen(uint8_t val){
	bitWrite(remoteOutputs[0], 2, val);
}
void setAlarmSound(uint8_t val){
	bitWrite(remoteOutputs[0], 3, val);
}
void setRobotGreen(uint8_t val){
	bitWrite(remoteOutputs[0], 4, val);
}
void setRobotRed(uint8_t val){
	bitWrite(remoteOutputs[0], 5, val);
}
void setHandGreen(uint8_t val){
	bitWrite(remoteOutputs[0], 6, val);
}
void setHandRed(uint8_t val){
	bitWrite(remoteOutputs[0], 7, val);
}
#endif /* INC_SLAVEDEVICES_H_ */
