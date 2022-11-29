#include "CollisionSensingRobotController.h"
#include <iostream>

using namespace std;

CollisionSensingRobotController::CollisionSensingRobotController(CommandQueue* queue, CommandQueue* hornQueue, se3910RPi::CollisionSensor* cs, std::string threadName) :
RobotController(queue, hornQueue, threadName){
	this->cs = cs;
}


void CollisionSensingRobotController::processSpeedControlCommand(int value){
	//value determined based off data from lab 5
	//tested on hardwood flooring, different robot and different terrain will require a different value for this
	int newMinDistance = ((value/10)*4) + 150;
	cs->setMinimumAcceptableDistance(newMinDistance);
	RobotController::processSpeedControlCommand(value);
}

void CollisionSensingRobotController::run(){
	this->leftFrontMotor->start(MOTOR_CTRL_TASK_PRIORITY);
	this->leftRearMotor->start(MOTOR_CTRL_TASK_PRIORITY);
	this->rightFrontMotor->start(MOTOR_CTRL_TASK_PRIORITY);
	this->rightRearMotor->start(MOTOR_CTRL_TASK_PRIORITY);
	while(keepGoing){
		if (this->referencequeue->hasItem()) {
			int command = this->referencequeue->dequeue();
			this->currentOperation = command & 0xf0000000;
			int commandVal = command & 0xfff;

			if (currentOperation == 0x20000000) {
				processMotionControlCommand(commandVal);
			} else if (currentOperation == 0x40000000) {
				processSpeedControlCommand(commandVal);
			} else if (currentOperation == 0x10000000) {
				processSteeringControlCommand(commandVal);
			}
		}
	}
}
