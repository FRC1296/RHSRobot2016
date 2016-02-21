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
	pTimer = new Timer();

	/*
	pSolenoid1 = new Solenoid(SOL_SHOOTER_1);
	pSolenoid2 = new Solenoid(SOL_SHOOTER_2);
	pSolenoid3 = new Solenoid(SOL_SHOOTER_3);
	pSolenoid4 = new Solenoid(SOL_SHOOTER_4);
	*/

	shooters = new ShooterSolenoid(CAN_PCM);
	claw = new Solenoid(SOL_SHOOTER_CLAW);

	pCompressor = new Compressor();
	pCompressor->Start();

	pTask = new Task(SHOOTER_TASKNAME, &Shooter::StartTask, this);
	wpi_assert(pTask);

}

Shooter::~Shooter() {
	delete pTimer;
}

void Shooter::Run(){

	switch(localMessage.command) {
	case COMMAND_SHOOTER_SHOOT:
			Shoot();
		break;
	case COMMAND_AUTONOMOUS_SHOOT:
			Shoot();
			SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
		break;
	default:

		break;
	}
/*
		pSolenoid1->Set(false);
		pSolenoid2->Set(false);
		pSolenoid3->Set(false);
		pSolenoid4->Set(false);*/
}

void Shooter::Shoot(){
	int pos = Arm::GetPulseWidthPosition();
	if(pos > bottomEncoderPos){
		/*
		pSolenoid1->Set(true);
		pSolenoid2->Set(true);
		pSolenoid3->Set(true);
		pSolenoid4->Set(true);*/
		claw->Set(true);
		Wait(.3);
		shooters->Open();
		Wait(.3);
		shooters->Close();
		claw->Set(false);
		ClearMessages(); // this is so that shoot command isnt send twice
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
