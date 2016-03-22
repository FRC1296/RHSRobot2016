/*
 * Shooter.h
 *
 *  Created on: Feb 19, 2016
 *      Author: Jacob
 */

#ifndef SHOOTER_H_
#define SHOOTER_H_

#include "WPILib.h"
#include "ComponentBase.h"
#include "ShooterSolenoid.h"
#include "JawSolenoid.h"

const float clawOpenDelay = 0.0;
const float preShootDelay = 1.5;
const float postShootDelay = 0.5;
const float clawCloseDelay = 0.5;

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
	Compressor* pCompressor;

	ShooterSolenoid* shooters;
	JawSolenoid* jaw;

	void OnStateChange();
	void Run();
	void Shoot();
};

#endif /* SHOOTER_H_ */
