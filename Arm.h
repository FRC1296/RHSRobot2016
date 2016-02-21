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

const int closeEncoderPos = 2320;
const int farEncoderPos = 2000;
const int bottomEncoderPos = -450;
const int intakeEncoderPos = -930;

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

private:
	CanArmTalon* pArmLeverMotor;
	CANTalon* pArmCenterMotor;
	CANTalon* pArmIntakeMotor;
	PIDController* pArmPID;
	static Arm* pInstance;
	bool bIsIntaking = false;

	const float fIntakeInSpeed = .4f;
	const float fIntakeOutSpeed = -1.0f;
	const float fCenterSpeed = -.6f;
	const float fMaxIntakeCurrent = 1.0f;


	int targetEncPos = 0; // Used when lowering the arm
	void OnStateChange();
	void Run();
	void Close();
	void Far();
	void Intake(bool direction);
	void AutoIntake();
	void Throwup();
	void AutoPos();
};

#endif /* ARM_H_ */
