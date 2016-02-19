/*
 * Arm.cpp
 *
 *  Created on: Feb 16, 2016
 *      Author: Jacob
 */

#include "Arm.h"
#include "RobotParams.h"
Arm::Arm() : ComponentBase(ARM_TASKNAME, ARM_QUEUE, ARM_PRIORITY){

	pArmLeverMotor = new CANTalon(CAN_ARM_LEVER_MOTOR);
	pArmLeverMotor->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);
	pArmLeverMotor->SetFeedbackDevice(CANTalon::QuadEncoder);

	pArmCenterMotor = new CANTalon(CAN_ARM_CENTER_MOTOR);
	pArmCenterMotor->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);

	pArmIntakeMotor = new CANTalon(CAN_ARM_INTAKE_MOTOR);
	pArmIntakeMotor->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);

	wpi_assert(pArmLeverMotor->IsAlive());
	wpi_assert(pArmCenterMotor->IsAlive());
	wpi_assert(pArmIntakeMotor->IsAlive());

	pTask = new Task(ARM_TASKNAME, &Arm::StartTask, this);
	wpi_assert(pTask);
}

Arm::~Arm() {

	delete pTask;
	delete pArmLeverMotor;
	delete pArmCenterMotor;
	delete pArmIntakeMotor;

}

void Arm::Run(){
	pArmIntakeMotor->Set(0); // This is how the intake motor stops

	switch(localMessage.command) {
	case COMMAND_ARM_RAISE:
		armState = ARM_RAISING;
		break;
	case COMMAND_ARM_LOWER:
		armState = ARM_LOWERING;
		break;
	case COMMAND_ARM_INTAKE:
		Intake(localMessage.params.armParams.direction);
		break;
	default:
		break;
	}

	if(localMessage.command != COMMAND_ARM_INTAKE && armState == ARM_FLOOR){
		armState = ARM_RAISING;
	}

	switch(armState){
	case ARM_RAISING:
		Raise();
		break;
	case ARM_LOWERING:
		targetEncPos = bottomEncoderPos;
		Lower();
		break;
	case ARM_TOP:

		break;
	case ARM_BOTTOM:

		break;
	case ARM_FLOOR:

		break;
	default:
		break;
	}



}

void Arm::Raise(){
	if(pArmLeverMotor->GetEncPosition() >= topEncoderPos){ // TODO check enc values
		armState = ARM_TOP;
		pArmLeverMotor->Set(0);
		return;
	}
	pArmLeverMotor->Set(fArmSpeed); // TODO check direction
}


void Arm::Lower(){
	if(pArmLeverMotor->GetEncPosition() <= targetEncPos){ // TODO check enc values
		if(targetEncPos == intakeEncoderPos){
			armState = ARM_FLOOR;
		}else if(targetEncPos == bottomEncoderPos){
			armState = ARM_BOTTOM;
		}else{
			// Should never get here
		}

		pArmLeverMotor->Set(0);
		return;
	}
	pArmLeverMotor->Set(-fArmSpeed); // TODO check direction
}


void Arm::Intake(bool direction){

	if(direction){
		pArmIntakeMotor->Set(-fIntakeSpeed); // TODO check direction
	}else{
		pArmIntakeMotor->Set(fIntakeSpeed); // TODO check direction
	} // TODO Ask aren about the lower part, whether the intake rolls if in ARM_TOP

	if(armState == ARM_BOTTOM || armState == ARM_LOWERING){
		targetEncPos = intakeEncoderPos;
		armState = ARM_LOWERING; // TODO do more lower code

	}
}

void Arm::OnStateChange(){
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

