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
	enum ArmState{
		ARM_LOWERING,
		ARM_BOTTOM,
		ARM_TOP,
		ARM_RAISING,
		ARM_FLOOR
	};

	ArmState armState;
	CANTalon* pArmLeverMotor;
	CANTalon* pArmCenterMotor;
	CANTalon* pArmIntakeMotor;

	const float fIntakeSpeed = .5f;
	const float fArmSpeed = .1f;
	const int topEncoderPos = 1024;
	const int bottomEncoderPos = 0;
	const int intakeEncoderPos = -100;

	int targetEncPos = 0; // Used when lowering the arm
	void OnStateChange();
	void Run();
	void Raise();
	void Lower();
	void Intake(bool direction);
};

#endif /* ARM_H_ */
