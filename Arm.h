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

const int closeEncoderPos = -743;
const int farEncoderPos = -743; //1166
const int bottomEncoderPos = -2700;
const int intakeEncoderPos = -3000; // -2670
const int afterShootEncoderPos = -1870;
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
	bool bIntakePressedLastFrame = false;
	Timer* pShootTimer;
	//Solenoid* claw;
	Relay* pLED;
	Relay* pSpare1;
	Relay* pSpare2;
	Relay* pSpare3;

	const float fIntakeInSpeed 		= -1.0f;
	const float fIntakeOutSpeed 	= 1.0f;
	const float fIntakeIdleSpeed 	= 0.0f;

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
