#include "MotorController.h"

using namespace std;

MotorController::MotorController(uint16_t deviceID, uint16_t fchannel, uint16_t rchannel, std::string threadName, uint32_t period)
:PeriodicTask(threadName, period){
	controlHardware = PCA9685Driver::obtainPCA9685Instance(deviceID);
	controlHardware->init();
	controlHardware->setFrequency(200);
	this->rchannel = rchannel;
	this->fchannel = fchannel;
}

void MotorController::setSpeed(int speed){
	this->speed = speed;
}

void MotorController::setDirection(int dir){
	this->direction = dir;
}

void MotorController::stop(){
	PeriodicTask::stop();
	setDirection(0);
}

void MotorController::taskMethod(){
	if(this->direction == 0){
		this->controlHardware->setDutyCycle(fchannel, 0);
		this->controlHardware->setDutyCycle(rchannel, 0);
	} else if(this->direction == 1){
		this->controlHardware->setDutyCycle(fchannel, speed);
		this->controlHardware->setDutyCycle(rchannel, 0);
	} else if(this->direction == -1){
		this->controlHardware->setDutyCycle(fchannel, 0);
		this->controlHardware->setDutyCycle(rchannel, speed);
	}
}

MotorController::~MotorController(){

}
