#include "RobotController.h"
#include <string>

using namespace std;

RobotController::RobotController(CommandQueue * queue, string threadName) :
RunnableClass(threadName){
	this->referencequeue = queue;
	this->leftFrontMotor = new MotorController(PCADEVICE_ADDR, LFMFCHANNEL, LFMRCHANNEL, threadName, MOTOR_CTRL_TASK_PERIOD);
	this->leftRearMotor = new MotorController(PCADEVICE_ADDR, LRMFCHANNEL, LRMRCHANNEL, threadName, MOTOR_CTRL_TASK_PERIOD);
	this->rightFrontMotor = new MotorController(PCADEVICE_ADDR, RFMFCHANNEL, RFMRCHANNEL, threadName, MOTOR_CTRL_TASK_PERIOD);
	this->rightRearMotor = new MotorController(PCADEVICE_ADDR, RRMFCHANNEL, RRMRCHANNEL, threadName, MOTOR_CTRL_TASK_PERIOD);
	processSpeedControlCommand(50);
}

RobotController::RobotController(CommandQueue* queue, CommandQueue* hornQueue, std::string threadName) :
RunnableClass(threadName){
	this->referencequeue = queue;
	this->hornQueue = hornQueue;
	this->leftFrontMotor = new MotorController(PCADEVICE_ADDR, LFMFCHANNEL, LFMRCHANNEL, threadName, MOTOR_CTRL_TASK_PERIOD);
	this->leftRearMotor = new MotorController(PCADEVICE_ADDR, LRMFCHANNEL, LRMRCHANNEL, threadName, MOTOR_CTRL_TASK_PERIOD);
	this->rightFrontMotor = new MotorController(PCADEVICE_ADDR, RFMFCHANNEL, RFMRCHANNEL, threadName, MOTOR_CTRL_TASK_PERIOD);
	this->rightRearMotor = new MotorController(PCADEVICE_ADDR, RRMFCHANNEL, RRMRCHANNEL, threadName, MOTOR_CTRL_TASK_PERIOD);
	processSpeedControlCommand(50);
}

void RobotController::run(){
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

int RobotController::processMotionControlCommand(int value) {
	switch (value) {
	case 0x1:
		// forward
		this->leftFrontMotor->setDirection(1);
		this->rightFrontMotor->setDirection(1);
		this->leftRearMotor->setDirection(1);
		this->rightRearMotor->setDirection(1);
		this->hornQueue->enqueue(HORN_MUTE_COMMAND);
		break;
	case 0x2:
		//reverse
		this->leftFrontMotor->setDirection(-1);
		this->rightFrontMotor->setDirection(-1);
		this->leftRearMotor->setDirection(-1);
		this->rightRearMotor->setDirection(-1);
		this->hornQueue->enqueue(HORN_PULSE_COMMAND | (300 << 12) | (600));
		break;
	case 0x4:
		// left
		this->leftFrontMotor->setDirection(-1);
		this->rightFrontMotor->setDirection(1);
		this->leftRearMotor->setDirection(-1);
		this->rightRearMotor->setDirection(1);
		this->hornQueue->enqueue(HORN_MUTE_COMMAND);
		break;
	case 0x5:
		//forward and to the left
		this->leftFrontMotor->setDirection(1);
		this->rightFrontMotor->setDirection(1);
		this->leftRearMotor->setDirection(0);
		this->rightRearMotor->setDirection(1);
		this->hornQueue->enqueue(HORN_MUTE_COMMAND);
		break;
	case 0x6:
		//reverse and to the left
		this->leftFrontMotor->setDirection(0);
		this->rightFrontMotor->setDirection(-1);
		this->leftRearMotor->setDirection(-1);
		this->rightRearMotor->setDirection(-1);
		this->hornQueue->enqueue(HORN_PULSE_COMMAND | (300 << 12) | (600));
		break;
	case 0x8:
		//right
		this->leftFrontMotor->setDirection(1);
		this->rightFrontMotor->setDirection(-1);
		this->leftRearMotor->setDirection(1);
		this->rightRearMotor->setDirection(-1);
		this->hornQueue->enqueue(HORN_MUTE_COMMAND);
		break;
	case 0x9:
		//forward and to the right
		this->leftFrontMotor->setDirection(1);
		this->rightFrontMotor->setDirection(1);
		this->leftRearMotor->setDirection(1);
		this->rightRearMotor->setDirection(0);
		this->hornQueue->enqueue(HORN_MUTE_COMMAND);
		break;
	case 0xA:
		//reverse and to the right
		this->leftFrontMotor->setDirection(-1);
		this->rightFrontMotor->setDirection(0);
		this->leftRearMotor->setDirection(-1);
		this->rightRearMotor->setDirection(-1);
		this->hornQueue->enqueue(HORN_PULSE_COMMAND | (300 << 12) | (600));
		break;
	case 0x10:
		//stop
		this->leftFrontMotor->setDirection(0);
		this->rightFrontMotor->setDirection(0);
		this->leftRearMotor->setDirection(0);
		this->rightRearMotor->setDirection(0);
		this->hornQueue->enqueue(HORN_MUTE_COMMAND);
		break;
	}
	return currentOperation & value;
}

void RobotController::processSpeedControlCommand(int value){
	this->currentSpeed = value;
	if(this->currentSteering == 0){
		leftFrontMotor->setSpeed(this->currentSpeed);
		leftRearMotor->setSpeed(this->currentSpeed);
		rightFrontMotor->setSpeed(this->currentSpeed);
		rightRearMotor->setSpeed(this->currentSpeed);
	}
	if(this->currentSteering < 0){
		//reduce speed of left wheels
		double reductionPercentage = ((100.0+this->currentSteering)/100.0);
		int reducedSpeed = static_cast<int>(reductionPercentage*this->currentSpeed);
		leftFrontMotor->setSpeed(reducedSpeed);
		leftRearMotor->setSpeed(reducedSpeed);
		rightFrontMotor->setSpeed(this->currentSpeed);
		rightRearMotor->setSpeed(this->currentSpeed);
	} else if(this->currentSteering > 0){
		//reduce speed of right wheels
		double reductionPercentage = ((100.0-this->currentSteering)/100.0);
		int reducedSpeed = static_cast<int>(reductionPercentage*this->currentSpeed);
		rightFrontMotor->setSpeed(reducedSpeed);
		rightRearMotor->setSpeed(reducedSpeed);
		leftFrontMotor->setSpeed(this->currentSpeed);
		leftRearMotor->setSpeed(this->currentSpeed);
	}
}


void RobotController::processSteeringControlCommand(int value){
	this->currentSteering = value - 100;
	this->processSpeedControlCommand(this->currentSpeed);
}



void RobotController::stop(){
	RunnableClass::stop();
	this->leftFrontMotor->stop();
	this->rightFrontMotor->stop();
	this->leftRearMotor->stop();
	this->rightRearMotor->stop();
}

void RobotController::waitForShutdown(){
	leftFrontMotor->waitForShutdown();
	leftRearMotor->waitForShutdown();
	rightFrontMotor->waitForShutdown();
	rightRearMotor->waitForShutdown();
	RunnableClass::waitForShutdown();
}

RobotController::~RobotController(){
	delete leftFrontMotor;
	delete leftRearMotor;
	delete rightFrontMotor;
	delete rightRearMotor;
}
