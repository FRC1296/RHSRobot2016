/*
 * DriveTalon.h
 *
 *  Created on: Mar 4, 2016
 *      Author: Jacob
 */

#ifndef SRC_DRIVETALON_H_
#define SRC_DRIVETALON_H_

#include "WPILib.h"
#include <string>

class DriveTalon : public CANTalon{
public:
	DriveTalon(int canid);
	virtual ~DriveTalon();

	static void *StartTask(void *pThis)
	{
		((DriveTalon *)pThis)->Run();
		return(NULL);
	}
	void ResetCurrentTimeout();

private:
	void Run();

	Timer* pCurrentTimer;
	Task* pTask;
	const float maxCurrent = 60.0;
	const float maxCurrentTime = 0.5;
	int cand;
	float lastCurrent=0;
	std::string path = "/home/lvuser/driveCurrent";
};

#endif /* SRC_DRIVETALON_H_ */
