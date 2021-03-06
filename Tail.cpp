/*
 * Tail.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: Jacob
 */

#include <Tail.h>
#include <RobotParams.h>

Tail::Tail() : ComponentBase(TAIL_TASKNAME, TAIL_QUEUE, TAIL_PRIORITY){
	pTailTimer = new Timer();
	pTailTimer->Stop();
	pTailTimer->Reset();

	pTailMotor = new CANTalon(CAN_TAIL_MOTOR);
	pTailMotor->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);
	pTailMotor->SetControlMode(CANTalon::kPercentVbus);

	pTask = new Task(TAIL_TASKNAME, &Tail::StartTask, this);
	wpi_assert(pTask);
}

Tail::~Tail() {
	delete pTailMotor;
	delete pTask;
}

void Tail::Run(){
	switch(localMessage.command) {
	case COMMAND_TAIL_RAISE:
		Raise();
		break;
	case COMMAND_TAIL_LOWER:
		Lower();
		break;
	default:
		//pTailMotor->Set(fIdlePower);
		break;
	}

	if(pTailTimer->Get()>fTailUpMotorTime && isRaising){
		pTailMotor->Set(fIdlePower);
	}
	if(pTailTimer->Get()>fTailDownMotorTime && isRaising){
		pTailMotor->Set(0);
	}

	if(pTailTimer->Get()>fTailDownTime && !isRaising){
		Raise();
	}
}

void Tail::Raise(){
	pTailTimer->Stop();
	pTailTimer->Reset();
	pTailTimer->Start();
	isRaising = true;
	pTailMotor->Set(fTailPower);
}

void Tail::Lower(){
	pTailTimer->Stop();
	pTailTimer->Reset();
	pTailTimer->Start();
	isRaising = false;
	pTailMotor->Set(-fTailPower);
}

void Tail::OnStateChange(){
	switch(localMessage.command) {
	case COMMAND_ROBOT_STATE_AUTONOMOUS:
		Raise();
		break;

	case COMMAND_ROBOT_STATE_TEST:

		break;

	case COMMAND_ROBOT_STATE_TELEOPERATED:
		Lower();
		break;

	case COMMAND_ROBOT_STATE_DISABLED:

		break;

	case COMMAND_ROBOT_STATE_UNKNOWN:

		break;

	default:

		break;
	}
}
