/*
 * Arm.cpp
 *
 *  Created on: Feb 16, 2016
 *      Author: Jacob
 */

#include "Arm.h"
#include "Drivetrain.h"
#include "RobotParams.h"
Arm::Arm() : ComponentBase(ARM_TASKNAME, ARM_QUEUE, ARM_PRIORITY){

	pArmLeverMotor = new CanArmTalon(CAN_ARM_LEVER_MOTOR);
	pArmLeverMotor->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);
	pArmLeverMotor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
	pArmPID = new PIDController(.0004,0,0,pArmLeverMotor, pArmLeverMotor, .05);
	//pArmLeverMotor->SetPID(.0001,0,0);

	pArmLeverMotor->SetInverted(true);
	pArmLeverMotor->SetIzone(TALON_IZONE);
	pArmLeverMotor->SetCloseLoopRampRate(TALON_MAXRAMP);
	pArmLeverMotor->SetControlMode(CANTalon::kPercentVbus);



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
	bIsIntaking = false;

	printf("output error %f \n", pArmPID->GetAvgError());
	SmartDashboard::PutNumber("arm encoder", pArmLeverMotor->GetPulseWidthPosition());

	switch(localMessage.command) {
	case COMMAND_ARM_RAISE:
		Raise();
		break;
	case COMMAND_ARM_LOWER:
		Lower();
		break;
	case COMMAND_ARM_INTAKE:
		bIsIntaking = true;
		Intake(localMessage.params.armParams.direction);
		break;
	default:
		break;
	}

	if(!bIsIntaking){
		pArmIntakeMotor->Set(0); // This is how the intake motor stops
		pArmCenterMotor->Set(0);
		if(pArmLeverMotor->GetPulseWidthPosition()<bottomEncoderPos){
			Lower();
		}
	}


}

void Arm::Raise(){
	pArmPID->SetSetpoint(topEncoderPos);
}


void Arm::Lower(){
	pArmPID->SetSetpoint(bottomEncoderPos);
}


// do we want to let the drivers run the intake and decide when to lower the arm?

void Arm::Intake(bool direction){

	if(direction){ // intaking
		pArmIntakeMotor->Set(fIntakeInSpeed);
		pArmCenterMotor->Set(fCenterSpeed);

		pArmPID->SetSetpoint(intakeEncoderPos);
	}else{ // throwing up
		pArmIntakeMotor->Set(fIntakeOutSpeed);
	}

}

void Arm::OnStateChange(){
	switch(localMessage.command) {
	case COMMAND_ROBOT_STATE_AUTONOMOUS:
		pArmPID->Disable();
		break;

	case COMMAND_ROBOT_STATE_TEST:
		pArmPID->Disable();
		break;

	case COMMAND_ROBOT_STATE_TELEOPERATED:
		pArmPID->Enable();
		break;

	case COMMAND_ROBOT_STATE_DISABLED:
		pArmPID->Disable();
		break;

	case COMMAND_ROBOT_STATE_UNKNOWN:
		pArmPID->Disable();
		break;

	default:
		pArmPID->Disable();
		break;
	}
}

