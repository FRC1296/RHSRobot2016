/*
 * Arm.h
 *
 *  Created on: Feb 16, 2016
 *      Author: Jacob
 */

#ifndef ARM_H_
#define ARM_H_
#include "WPILib.h"
#include <ComponentBase.h>
#include <CanArmTalon.h>

const int closeEncoderPos = 2930;
const int farEncoderPos = 3600; //3685
const int bottomEncoderPos = 300; //300
const int intakeEncoderPos = -300;
const float shootDelay = 1.0f;
const float clawDelay = .3f;
const float centerDelay = .2f;

class Arm : public ComponentBase
{
public:
	Arm();
	~Arm();

	static void *StartTask(void *pThis)
	{
		pInstance = ((Arm *)pThis);
		((Arm *)pThis)->DoWork();
		return(NULL);
	}
	static int GetPulseWidthPosition();
	static int GetEncTarget();

private:
	CanArmTalon* pArmLeverMotor;
	CANTalon* pArmCenterMotor;
	CANTalon* pArmIntakeMotor;
	PIDController* pArmPID;
	static Arm* pInstance;
	bool bIsIntaking = false;
	Timer* pShootTimer;
	Solenoid* claw;

	const float fIntakeInSpeed 		= 0.4f;
	const float fIntakeOutSpeed 	= -1.0f;
	const float fCenterSpeed 		= -0.6f;
	const float fMaxCenterCurrent 	= 100.0f;
	const float fIntakeIdleSpeed 	= 0.1f;
	const float fCenterOutSpeed 	= 0.1f;

	const float fAutoIntakeTimeout 	= 5.0f;
	const float fAutoThrowupTime	= 0.5f;
	const float fAutoTimeToArm		= 1.0f;

	void OnStateChange();
	void Run();
	void Close();
	void Far();
	void Intake(bool direction);
	void AutoIntake();
	void Throwup();
	void AutoPos();
	void IntakeShoot();

};

#endif /* ARM_H_ */
