/*
 * Tail.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: Jacob
 */

#include <Tail.h>
#include <RobotParams.h>

Tail::Tail() : ComponentBase(TAIL_TASKNAME, TAIL_QUEUE, TAIL_PRIORITY){
	pTailMotor = new CANTalon(CAN_TAIL_MOTOR);
	pTailMotor->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);
	pTailMotor->SetControlMode(CANTalon::kPercentVbus);

	pTask = new Task(TAIL_TASKNAME, &Tail::StartTask, this);
	wpi_assert(pTask);
}

Tail::~Tail() {

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
		pTailMotor->Set(fIdleVoltage);
		break;
	}
}

void Tail::Raise(){
	pTailMotor->Set(fRaiseVoltage); // TODO check direction
}

void Tail::Lower(){
	pTailMotor->Set(-fLowerVoltage); // TODO check direction
}

void Tail::OnStateChange(){
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
