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

const int closeEncoderPos = 1500;  //
const int farEncoderPos = 2200; // -2116 actual position
const int bottomEncoderPos = 260;
const int intakeEncoderPos = -150; // -53 actual position
const int afterShootEncoderPos = 810;



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
	static int GetEncPosition();
	static int GetEncTarget();
	static double GetIntakeCurrent();
	static void StopIntake();

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

	const float fIntakeInSpeed 		= -1.0f;
	const float fIntakeOutSpeed 	= 1.0f;
	const float fIntakeIdleSpeed 	= 0.0f;
	const float fIntakeOutShootingSpeed = 0.50;  // to let ball settle

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
