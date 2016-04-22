/*
 * PIDSearchOutput.cpp
 *
 *  Created on: Feb 5, 2016
 *      Author: fmj
 */

#include <PIDSearchOutput.h>
#include "Drivetrain.h"
#include "RobotParams.h"

PIDSearchOutput::PIDSearchOutput(CANTalon* leftOneMotor,
		CANTalon* rightOneMotor,  bool servo) {
pLeftOneMotor = leftOneMotor;
pRightOneMotor = rightOneMotor;
bUnderServoControl = servo;
}

PIDSearchOutput::~PIDSearchOutput() {
	delete pLeftOneMotor;
	delete pRightOneMotor;
}

void PIDSearchOutput::PIDWrite(float output){
	ABLIMIT(output,.2f);
	if(bUnderServoControl)
	{
		pLeftOneMotor->Set(-output * FULLSPEED_FROMTALONS);

		pRightOneMotor->Set(-output * FULLSPEED_FROMTALONS);

	}
	else
	{
		pLeftOneMotor->Set(-output);

		pRightOneMotor->Set(-output);

	}
}
