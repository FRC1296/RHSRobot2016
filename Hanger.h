/*
 * Hanger.h
 *
 *  Created on: Feb 27, 2016
 *      Author: Jacob
 */

#ifndef HANGER_H_
#define HANGER_H_
#include "WPILib.h"
#include <ComponentBase.h>

class Hanger : public ComponentBase{
public:
	Hanger();
	virtual ~Hanger();

	static void *StartTask(void *pThis)
	{
		((Hanger *)pThis)->DoWork();
		return(NULL);
	}

private:
	enum HangerState{
		NOT_DEPLOYED,
		DEPLOYED_AND_WAITING,
		RAISING
	};
	CANTalon* pHangerMotor;
	Timer* pHangTimer;

	HangerState currentState;

	void OnStateChange();
	void Run();
	void Hang();

	const float fPinoutTime = .1f;
	const float fAirTime = 1.5f;
	const float fRaiseSpeed = 1.0;
	const float fPulloutSpeed = 1.0;
	const double fTeleopTime = 135.0;
	const double fActivateTimeLeft = 20;

	bool bHasStarted = false;
};

#endif /* HANGER_H_ */
