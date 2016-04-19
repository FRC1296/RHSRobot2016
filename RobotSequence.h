/*
 * RobotSequence.h
 *
 *  Created on: Apr 18, 2016
 *      Author: Jacob
 */

#ifndef ROBOTSEQUENCE_H_
#define ROBOTSEQUENCE_H_
#include "WPILib.h"
#include "RobotMessage.h"

class RobotSequence {
public:
	RobotSequence(const char* const task);
	virtual ~RobotSequence();
	virtual void Run()=0;
	bool IsRunning(){return bSequenceRunning;}

	static void *StartTask(void *pThis)
	{
		pInstance = ((RobotSequence *)pThis);
		pInstance->bSequenceRunning = true;
		((RobotSequence *)pThis)->Run();
		pInstance->bSequenceRunning = false;
		return(NULL);
	}
	void StartSequence();
	void SendMessage(const char* szQueueName, RobotMessage* message);
private:
	static RobotSequence* pInstance;

	bool bSequenceRunning;
	Task* pTask;
	const char* taskname;

};

#endif /* ROBOTSEQUENCE_H_ */
