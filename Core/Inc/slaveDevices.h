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
uint8_t getClosedInSw(){
	 return bitRead(gateINInputs[0], 1);
}
uint8_t getOpenInSw(){
	 return bitRead(gateINInputs[0], 2);
}
void setGateInOpen(uint8_t val){
	bitWrite(gateINOutputs[0], 0, val);
}
void setGateInClosed(uint8_t val){
	bitWrite(gateINOutputs[0], 1, val);
}
void setGateInStop(uint8_t val){
	bitWrite(gateINOutputs[0], 2, val);
}
void setGateInGreen(uint8_t val){
	bitWrite(gateINOutputs[0], 3, val);
}
void setGateInRed(uint8_t val){
	bitWrite(gateINOutputs[0], 4, val);
}
void setGateInHandGreen(uint8_t val){
	bitWrite(gateINOutputs[0], 5, val);
}
void setGateInHandRed(uint8_t val){
	bitWrite(gateINOutputs[0], 6, val);
}

/* Модуль ворот выезд */
uint8_t getPhotoOutSensor(){
	 return bitRead(gateOUTInputs[0], 0);
}
uint8_t getClosedOutSw(){
	 return bitRead(gateOUTInputs[0], 1);
}
uint8_t getOpenOutSw(){
	 return bitRead(gateOUTInputs[0], 2);
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

#endif /* INC_SLAVEDEVICES_H_ */
