#include "CollisionSensor.h"
#include "DistanceSensor.h"
#include "NetworkCommands.h"
using namespace std;
namespace se3910RPi {
/**
 * This is the constructor for this class.
 * @param mcq This is the command queue used by the motor for control purposes.
 * @param hq This is the queue used to communicate with the Horn controller.
 * @param dsi This is the instance of the distance sensor.  It will read how far it is to a given object.
 * @param threadName This is the name of the thread that is to periodically be invoked.
 * @param period This is the period for the task.  This period is given in microseconds.
 */
CollisionSensor::CollisionSensor(CommandQueue* mcqi, CommandQueue *hqi, se3910RPiHCSR04::DistanceSensor* dsi, std::string threadName, uint32_t period):
PeriodicTask(threadName, period) {
    mcq = mcqi;
    hq = hqi;
    ds = dsi;
}
/**
 * This is the default destructor which will clean up from the distance sensor.
 */
CollisionSensor::~CollisionSensor() {
    delete mcq;
    delete hq;
    delete ds;
}
/**
 * This method will set the minimum distance.  If two reads in a row are below this value, then a stop command will be sent.
 * The stop command, however, is to be sent exactly once.
 * @param mad This is the minimum distance to the wall or other item that is acceptable before stopping.  This must be a positive number or no change will occur.
 */
void CollisionSensor::setMinimumAcceptableDistance(int mad) {
    minimumAcceptableDistance = mad;
}
/**
 * This is the task method.  The task method will be invoked periodically every taskPeriod
 * units of time.
 */
void CollisionSensor::taskMethod() {
    int curr = ds->getCurrentDistance();
    if (curr < minimumAcceptableDistance) {
        madViolationCount = madViolationCount + 1;
    } else {
        madViolationCount = 0;
    }
    if (madViolationCount == 2) {
    	hq->enqueue(HORN_SOUND_COMMAND);
        mcq->enqueue(MOTORDIRECTIONBITMAP|STOP);
    } else if (madViolationCount == 3) {
        hq->enqueue(HORN_SOUND_COMMAND);
        isAlerting = 1;
    } else if (madViolationCount <= 1 && isAlerting == 1) {
        hq->enqueue(HORN_MUTE_COMMAND);
        isAlerting = 0;
    }
}
}
