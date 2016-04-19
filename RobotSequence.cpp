/*
 * RobotSequence.cpp
 *
 *  Created on: Apr 18, 2016
 *      Author: Jacob
 */

#include "RobotSequence.h"
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

RobotSequence* RobotSequence::pInstance;
RobotSequence::RobotSequence(const char* const task) {
	pTask = NULL;
	bSequenceRunning = false;
	taskname = task;
}

RobotSequence::~RobotSequence() {

}

void RobotSequence::StartSequence(){
	if(bSequenceRunning)return;
	printf("started sequence\n");
	pTask = new Task(taskname, &RobotSequence::StartTask, this);
	wpi_assert(pTask);
}

void RobotSequence::SendMessage(const char* szQueueName, RobotMessage* message){

	int iPipeXmt;

	iPipeXmt = open(szQueueName, O_WRONLY);
	wpi_assert(iPipeXmt > 0);

	write(iPipeXmt, (char*) message, sizeof(RobotMessage));
	close(iPipeXmt);
}
