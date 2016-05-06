/*
 * HangerSequence.cpp
 *
 *  Created on: Apr 25, 2016
 *      Author: Jacob
 */

#include "HangerSequence.h"
#include "RobotMessage.h"
#include "RobotParams.h"

HangerSequence::HangerSequence() : RobotSequence(HANGER_SEQ_TASKNAME){

}

HangerSequence::~HangerSequence() {
}

void HangerSequence::Run(){
	RobotMessage mc;

	mc.command = COMMAND_ARM_CLOSE;
	SendMessage(ARM_QUEUE, &mc);
	Wait(3.0);

	mc.command = COMMAND_HANGER_SOLENOID_ENABLE;
	SendMessage(HANGER_QUEUE, &mc);
	Wait(5.0);
	mc.command = COMMAND_ARM_FAR;
	SendMessage(ARM_QUEUE, &mc);
	Wait(2.0);
	mc.command = COMMAND_SHOOTER_JAW_OPEN;
	SendMessage(SHOOTER_QUEUE, &mc);
	Wait(2.0);
}
