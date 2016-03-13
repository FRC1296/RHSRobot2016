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

const int closeEncoderPos = 0;
const int farEncoderPos = 0; //1166
const int bottomEncoderPos = -2600;
const int intakeEncoderPos = -3400; // -2670
const float shootDelay = 3.0f;  // how long t0 wait before lowering arm

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
	CANTalon* pArmIntakeMotor;
	PIDController* pArmPID;
	static Arm* pInstance;
	bool bIsIntaking = false;
	Timer* pShootTimer;
	//Solenoid* claw;
	Relay* pLED;

	const float fIntakeInSpeed 		= 0.4f;
	const float fIntakeOutSpeed 	= -1.0f;
	const float fIntakeIdleSpeed 	= 0.25f;

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
