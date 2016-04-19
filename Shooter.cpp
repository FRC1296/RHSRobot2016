/*
 * Shooter.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: Jacob
 */

#include "Shooter.h"
#include <RobotParams.h>
#include "Arm.h"
Shooter::Shooter() : ComponentBase(SHOOTER_TASKNAME, SHOOTER_QUEUE, SHOOTER_PRIORITY){

	shooters = new ShooterSolenoid(CAN_PCM_SHOOTER);
	jaw = new JawSolenoid(CAN_PCM_JAW);

	pCompressor = new Relay(0, Relay::kForwardOnly);
	pCompressor->Set(Relay::kOn);// switch 5
	pSwitch = new DigitalInput(5);

	pTask = new Task(SHOOTER_TASKNAME, &Shooter::StartTask, this);
	wpi_assert(pTask);

}

Shooter::~Shooter() {
	delete pCompressor;
	delete shooters;
	delete pTask;
}

void Shooter::Run(){
	pCompressor->Set(pSwitch->Get()==false?Relay::kOn:Relay::kOff);

	switch(localMessage.command) {
	case COMMAND_SHOOTER_SHOOT:
			Shoot();
		break;
	case COMMAND_SHOOTER_SHOOTER_OPEN:
		shooters->Open();
		break;
	case COMMAND_SHOOTER_SHOOTER_CLOSE:
		shooters->Close();
		break;
	case COMMAND_SHOOTER_JAW_OPEN:
		jaw->Open();
		break;
	case COMMAND_SHOOTER_JAW_CLOSE:
		jaw->Close();
		break;

	case COMMAND_AUTONOMOUS_SHOOT:
			Shoot();
			SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
		break;
	default:

		break;
	}

}

void Shooter::Shoot(){
	int pos = Arm::GetEncTarget();

	if(pos > bottomEncoderPos)
	{
		Wait(clawOpenDelay);
		jaw->Open();
		Wait(preShootDelay);
		shooters->Open();
		Wait(postShootDelay);
		shooters->Close();
		Wait(clawCloseDelay);
		jaw->Close();
		ClearMessages(); // make sure shoot command is not processed twice
	}
}

void Shooter::OnStateChange(){
	switch(localMessage.command) {
	case COMMAND_ROBOT_STATE_AUTONOMOUS:

		break;

	case COMMAND_ROBOT_STATE_TEST:

		break;

	case COMMAND_ROBOT_STATE_TELEOPERATED:

		break;

	case COMMAND_ROBOT_STATE_DISABLED:

		break;

	case COMMAND_ROBOT_STATE_UNKNOWN:

		break;

	default:

		break;
	}
}
