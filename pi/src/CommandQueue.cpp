# include "CommandQueue.h"
#include <queue>
#include <mutex>
#include <semaphore.h>

using namespace std;

CommandQueue::CommandQueue(){
	this->commandQueueContents = queue<int>();
	sem_init(&queueCountSemaphore, 0, 0);
}

int CommandQueue::dequeue(){
	sem_wait(&queueCountSemaphore);
	lock_guard<mutex> lock(queueMutex);
	int val = commandQueueContents.front();
	commandQueueContents.pop();
	return val;
}

void CommandQueue::enqueue(int value){
	lock_guard<mutex> lock(queueMutex);
	commandQueueContents.push(value);
	sem_post(&queueCountSemaphore);
}

bool CommandQueue::hasItem(){
	lock_guard<mutex> lock(queueMutex);
	if(commandQueueContents.size() == 0){
		return false;
	}
	return true;
}

CommandQueue::~CommandQueue(){

}
