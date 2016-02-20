/*
 * Shooter.h
 *
 *  Created on: Feb 19, 2016
 *      Author: Jacob
 */

#ifndef SHOOTER_H_
#define SHOOTER_H_

#include "WPILib.h"
#include <ComponentBase.h>

class Shooter : public ComponentBase{
public:
	Shooter();
	virtual ~Shooter();

	static void *StartTask(void *pThis)
	{
		((Shooter *)pThis)->DoWork();
		return(NULL);
	}

private:
	Timer* pTimer;
	Compressor* pCompressor;
	Solenoid* pSolenoid1;
	Solenoid* pSolenoid2;
	Solenoid* pSolenoid3;
	Solenoid* pSolenoid4;

	const float fOpenTime = .5f;

	void OnStateChange();
	void Run();
	void Shoot();
};

#endif /* SHOOTER_H_ */
