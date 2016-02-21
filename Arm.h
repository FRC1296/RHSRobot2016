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

class Arm : public ComponentBase
{
public:
	Arm();
	~Arm();

	static void *StartTask(void *pThis)
	{
		((Arm *)pThis)->DoWork();
		return(NULL);
	}

private:
	CanArmTalon* pArmLeverMotor;
	CANTalon* pArmCenterMotor;
	CANTalon* pArmIntakeMotor;
	PIDController* pArmPID;
	bool bIsIntaking = false;

	const float fIntakeInSpeed = .4f;
	const float fIntakeOutSpeed = -1.0f;
	const float fCenterSpeed = -.6f;
	const int topEncoderPos = 2320;
	const int bottomEncoderPos = -450;
	const int intakeEncoderPos = -930;

	int targetEncPos = 0; // Used when lowering the arm
	void OnStateChange();
	void Run();
	void Raise();
	void Lower();
	void Intake(bool direction);
};

#endif /* ARM_H_ */
