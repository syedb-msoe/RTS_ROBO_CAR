#include "LineSensor.h"
#include "NetworkCommands.h"
using namespace std;
namespace se3910RPi {
/**
* This is the constructor for this class.
* @param ctrlQueue This is the queue that this item will use to receive enable and disable commands from the network.
* @param mcq This is the command queue used by the motor for control purposes.
* @param leftSensorNumber This is the GPIO pin number for the left sensor.
* @param centerSensorNumber This is the GPIO pin number for the center sensor.
* @param rightSensor This is the GPIO pin number for the right sensor.
* @param threadName This is the name of the thread that is to periodically be invoked.
* @param period This is the period for the task.  This period is given in microseconds.
*/
LineSensor::LineSensor(CommandQueue *ctrlQueue, CommandQueue* mcq, int leftSensorNumber, int centerSensorNumber, int rightSensorNumber, std::string threadName, uint32_t period):
PeriodicTask(threadName, period) {
    this->ctrlQueue = ctrlQueue;
    this->mcq = mcq;
    this->leftSensor = new GPIO(leftSensorNumber, GPIO::GPIO_IN);
    this->centerSensor = new GPIO(centerSensorNumber, GPIO::GPIO_IN);
    this->rightSensor = new GPIO(rightSensorNumber, GPIO::GPIO_IN);
    this->currentlyActive = false;
    this->lineFollowingEnabled = false;
}
/**
* This is the default destructor which will clean up from the distance sensor.
*/
LineSensor::~LineSensor() {
    delete ctrlQueue;
    delete mcq;
    delete leftSensor;
    delete centerSensor;
    delete rightSensor;
}
/**
* This is the task method.  The task method will be invoked periodically every taskPeriod
* units of time.
*/
void LineSensor::taskMethod() {
    if (ctrlQueue->hasItem()) {
        int item = ctrlQueue->dequeue();
        if (item == START_LINE_SENSING) {
            currentlyActive = true;
        } else if (item == STOP_LINE_SENSING) {
            currentlyActive = false;
        } else if (item == ENABLE_LINE_FOLLOWING) {
            lineFollowingEnabled = true;
        } else if (item == DISABLE_LINE_FOLLOWING) {
            lineFollowingEnabled = false;
        }
    }
    if (currentlyActive) {
        GPIO::VALUE leftValue = leftSensor->getValue();
        GPIO::VALUE centerValue = centerSensor->getValue();
        GPIO::VALUE rightValue = rightSensor->getValue();


        if (lineFollowingEnabled) {
            if (leftValue == GPIO::GPIO_LOW && rightValue == GPIO::GPIO_LOW && centerValue == GPIO::GPIO_HIGH) {
                mcq->enqueue(MOTORDIRECTIONBITMAP|FORWARD);
                stopCount = 0;
            } else if (rightValue == GPIO::GPIO_LOW && leftValue == GPIO::GPIO_HIGH) {
                mcq->enqueue(MOTORDIRECTIONBITMAP|LEFT);
                stopCount = 0;
            } else if (leftValue == GPIO::GPIO_LOW && rightValue == GPIO::GPIO_HIGH) {
                mcq->enqueue(MOTORDIRECTIONBITMAP|RIGHT);
                stopCount = 0;
            } else if (leftValue == GPIO::GPIO_HIGH && rightValue == GPIO::GPIO_HIGH && centerValue == GPIO::GPIO_HIGH){
            	if(stopCount == 2){
                mcq->enqueue(MOTORDIRECTIONBITMAP|STOP);
            	} else {
            		stopCount++;
            	}
            }
        } else {
        	if (leftValue == GPIO::GPIO_HIGH && rightValue == GPIO::GPIO_HIGH && centerValue == GPIO::GPIO_HIGH) {
        		mcq->enqueue(MOTORDIRECTIONBITMAP|STOP);
        	}
        }
    }
}
}
