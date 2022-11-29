/**
 * @file DistanceSensor.cpp
 * @author  Walter Schilling (schilling@msoe.edu)
 * @version 1.0
 *
 * @section LICENSE
 *
 *
 * This code is developed as part of the MSOE SE3910 Real Time Systems course,
 * but can be freely used by others.
 *
 * SE3910 Real Time Systems is a required course for students studying the
 * discipline of software engineering.
 *
 * This Software is provided under the License on an "AS IS" basis and
 * without warranties of any kind concerning the Software, including
 * without limitation merchantability, fitness for a particular purpose,
 * absence of defects or errors, accuracy, and non-infringement of
 * intellectual property rights other than copyright. This disclaimer
 * of warranty is an essential part of the License and a condition for
 * the grant of any rights to this Software.
 *
 * @section DESCRIPTION
 *      This file defines the interface to be used to communicate with a
 *      HC-SR04 Ultrasonic distance sensor.  The sensor will give the
 * line of site distance to a remote object via ultrasonic detection.
 */

#include "DistanceSensor.h"
#include <cmath>

namespace se3910RPiHCSR04 {

/**
 * This method will instantiate a new instance of the distance sensor.  The algorithm is as follows:
 * @param trigPin This is the trigger pin, which is the pin that determines when to start an ultrasonic measurement.
 * @param echoPin This is the echo pin.  It will indicate the distance to the remote device.
 * @param threadName This is the name of the thread that is to periodically be invoked.
 * @param period This is the period for the task.  This period is given in microseconds.
 */
DistanceSensor::DistanceSensor(int trigPin, int echoPin, std::string threadName,
		uint32_t period) :
		PeriodicTask(threadName, period) {
	/**
	 * 1.0 Instantiate an instance of the trigger pin as a GPIO pin.
	 */
	this->tPin = new GPIO(trigPin, GPIO::GPIO_OUT, GPIO::GPIO_LOW);

	/**
	 * 2.0 Instantiate the echo pin as an input pin.
	 */
	this->ePin = new GPIO(echoPin, GPIO::GPIO_IN, GPIO::GPIO_LOW);

	/**
	 * 3.0 Enable interrupts on both rising and falling edges for the echo pin.
	 */
	this->ePin->enableEdgeInterrupt(GPIO::GPIO_BOTH);
}

/**
 * This is the default destructor which will clean up from the distance sensor.
 * The method simply deletes allocated objects.
 */
DistanceSensor::~DistanceSensor() {
	/**
	 * 1.0 Delete any allocated objects.
	 */
	delete tPin;
	delete ePin;
}

/**
 * This is the task method.  The task method will be invoked periodically every taskPeriod
 * units of time.
 */
void DistanceSensor::taskMethod() {
	struct timespec startTime;
	struct timespec endTime;
	struct timespec deltaTime;

	/**
	 * 1.0 Start by incrementing the read count by 1 and setting the trigger high for at least 10us.
	 */
	this->readCount++;
	this->tPin->setValue(GPIO::GPIO_HIGH);

	/**
	 * 2.0 Do a quick nano sleep for 10 microseconds.  You'll need to study the nanosleep method.
	 */
	usleep(10);

	/**
	 * 3.0 Now set the trigger pin low.
	 */
	this->tPin->setValue(GPIO::GPIO_LOW);

	/**
	 * 4.0 Now, wait for the edge to go high.  Wait no more than 10 ms.  If you wait longer than that, you've probably missed the edge, which can happen.
	 * Record the status, which is returned from waiting for the edge.  If it is negative, ignore the time capture, as this run will not be accurate.
	 */
	int startEdgeResult = this->ePin->waitForEdge(8);

	/**
	 * 5.0 Check the status returned from the wait call to make sure it is not zero.
	 */
	if(startEdgeResult == 0){
		/**
		 * 5.0.1 If the status indicated a valid recording of the timestamp, grab the timestamp for the transition from the GPIO library.
		 */
		startTime = this->ePin->getRisingISRTimestamp();
	}


	/**
	 * 6.0 Now wait for the pulse to go low and capture that timestamp.  Wait no longer than 11ms, as this is the bound of accuracy on the high end for this device.
	 */
	int endEdgeResult = this->ePin->waitForEdge(8);

	/**
	 * 7.0 Check the status returned from the second wait call.
	 */
	if(endEdgeResult == 0){
		/**
		 * 7.0.1 If the status is value, get the end timestamp.
		 */
		endTime = this->ePin->getFallingISRTimestamp();
	}


	/**
	 * 8.0 If both edges have been properly detected, then measure the distance.
	 */
	if(endEdgeResult == 0 && startEdgeResult == 0){
		/**
		 * 8.0.1 Determine the delta between the two timestamps.  This is the duration of the pulse.
		 */
	    if ((endTime.tv_nsec - startTime.tv_nsec) < 0) {
	    	//if the number of nanoseconds are less for the endtime we need to borrow a second to avoid negative nanoseconds
	        deltaTime.tv_sec = endTime.tv_sec - startTime.tv_sec - 1;
	        deltaTime.tv_nsec = endTime.tv_nsec - startTime.tv_nsec + 1000000000;
	    } else {
	    	deltaTime.tv_sec = endTime.tv_sec - startTime.tv_sec;
	        deltaTime.tv_nsec = endTime.tv_nsec - startTime.tv_nsec;
	    }

		/**
		 * 8.0.2 Convert the delta in time into a distance in millimeters.
		 * Place the result into an integer by rounding the result.
		 */
		double totalSeconds = (deltaTime.tv_nsec/1000000000.0) + deltaTime.tv_sec;
		int distance = 1000 * (344 * (totalSeconds/2.0));

		/**
		 * 8.0.3 Validate the validity of the measured distance, making certain it is in range.
		 */
		if(0 <= distance && distance <= 2000){
			/**
			 * 8.0.3.1 If the distance measured is between 0 and 2000 mm, it is considered to be valid.
			 */
			this->currentDistance = distance;

			/**
			 * 8.0.3.2 Update the count of distances read and the total of distances read.
			 */
			this->distanceRecordingCount++;
			this->totalOfAllDistances += this->currentDistance;

			/**
			 * 8.0.3.3 Compare the currently recorded distance with the max and min values,
			 * If it is less than the min value make this new reading the min value.
			 * If it is greater than the max value, make it the max value.
			 */
			if(this->currentDistance > this->maxDistance){
				this->maxDistance = this->currentDistance;
			} else if(this->currentDistance < this->minDistance){
				this->minDistance = this->currentDistance;
			}

			/**
			 * 8.0.3.4 Increment the valid read count variable.
			 */
			this->validReadCount++;
		}

	}
}

/**
 * This method will obtain the maximum read distance.
 * @return The max distance read will be returned in units of mm.
 */
int DistanceSensor::getMaxDistance() {
	return this->maxDistance;
}

/**
 * This method will obtain the minimum read distance.
 * @return The min distance read will be returned in units of mm.
 */
int DistanceSensor::getMinDistance() {
	return this->minDistance;
}

/**
 * This method will obtain the current read distance.
 * @return The last valid distance read will be returned in units of mm.
 */
int DistanceSensor::getCurrentDistance() {
	return this->currentDistance;
}

/**
 * This method will return the read cpount, or the number of times the periodic task is invoked..
 * @return The valid count will be returned.
 */
uint32_t DistanceSensor::getReadCount() {
	return readCount;
}

/**
 * This method will return the valid count or the number of times a valid read has been obtained..
 * @return The valid count will be returned.
 */
uint32_t DistanceSensor::getValidReadCount() {
	return validReadCount;
}

/**
 * This method will reset the max and min values to their default ranges, allowing new ranges to be captured.
 */
void DistanceSensor::resetDistanceRanges() {
	maxDistance = 0;
	minDistance = 2500;
	totalOfAllDistances = 0;
	distanceRecordingCount = 0;
	readCount = 0;
	validReadCount = 0;
}

/**
 * This method will obtain the average read distance.
 * @return The last average distance read will be returned.
 */
int DistanceSensor::getAverageDistance() {
	/**
	 * 1.0 Set the return value to zero.
	 */
	int averageDistance = 0;
	/**
	 * 2.0 If there are more than 0 distance readings.
	 */

	if(this->distanceRecordingCount > 0){
		/**
		 * 2.1 Return the sum of all distance readings divided by the number as the average.
		 */
		averageDistance = this->totalOfAllDistances/distanceRecordingCount;
	}

	return averageDistance;
}

/**
 * This method will obtain the number of times that a distance has been measured since the count was reset.
 * @return the Number of samples captured will be returned.
 */
int DistanceSensor::getDistanceCount() {
	return this->distanceRecordingCount;
}

}
