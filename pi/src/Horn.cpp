#include "Horn.h"
#include "GPIO.h"
#include "NetworkCommands.h"
#include <unistd.h>
using namespace se3910RPi;
/**
* This is the default constructor.
* @para, queue This is the queue that this task is to read from.
* @param gpioPin This is the GPIO pin that the buzzer is attached to.
* @param threadName This is the name that is to be given to the executing thread as a string.
* @param taskRate This is the rate at which the thread re-executes.
*/
Horn::Horn(CommandQueue *queue, int gpioPin, std::string threadName, uint32_t taskRate):
PeriodicTask(threadName, taskRate) {
    myqueue = queue;
    hornPin = new GPIO(gpioPin, GPIO::GPIO_OUT);
    hornPin->setValue(GPIO::GPIO_LOW);
}
/**
* This is the default destructor that will clean up from the class.
*/
Horn::~Horn() {
    delete myqueue;
    delete hornPin;
}
/**
* This method will cause the horn to sound continuously until stopped.
*/
void Horn::soundHorn() {
    length = 0;
    repetitionTime = 1;
    hornPin->setValue(GPIO::GPIO_HIGH);
}
/**
* This method will pulse the horn.  It will be on for length ms and then off for the remainder of time until the period is reached.
* @param length This is the length of the on time, given in ms.
* @param period This is the period for the horn, given in ms.
*/
void Horn::pulseHorn(int length, int period) {
    printf("%d\n", hornCount);
    if (hornCount == 0) {
        hornPin->setValue(GPIO::GPIO_HIGH);
    } else if (hornCount >= repetitionTime) {
        hornCount = -125;
    } else if (hornCount >= length) {
        hornPin->setValue(GPIO::GPIO_LOW);
    }
    hornCount = hornCount + 125;
}
/**
* This method will silence the horn so that it no longer is making any audio noise.
*/
void Horn::silenceHorn() {
    length = 0;
    repetitionTime = 1;
    hornPin->setValue(GPIO::GPIO_LOW);
}
/**
* This is the task method for the class.
*/
void Horn::taskMethod() {
    if (myqueue->hasItem()) {
        int item = myqueue->dequeue();
        int type = item & 0xF0000000;
        if (type == HORN_MUTE_COMMAND) {
            silenceHorn();
        } else if (type == HORN_SOUND_COMMAND) {
            soundHorn();
        } else if (type == HORN_PULSE_COMMAND) {
            hornCount = 0;
            this->length = item & 0x00000FFF;
            this->repetitionTime = (item >> 3) & 0x00000FFF;
            printf("%d\n", repetitionTime);
            printf("%d\n", length);
        }
    }
    if (this->length > 0) {
        pulseHorn(this->length, this->repetitionTime);
    }
}
