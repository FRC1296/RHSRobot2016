/*
 * Arm.cpp
 *
 *  Created on: Feb 16, 2016
 *      Author: Jacob
 */

#include "Arm.h"
#include "Drivetrain.h"
#include "RobotParams.h"



Arm* Arm::pInstance;
Arm::Arm() : ComponentBase(ARM_TASKNAME, ARM_QUEUE, ARM_PRIORITY){

	pArmLeverMotor = new CanArmTalon(CAN_ARM_LEVER_MOTOR);
	pArmLeverMotor->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);
	pArmLeverMotor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
	pArmPID = new PIDController(.0005,0,0,pArmLeverMotor, pArmLeverMotor, .05);
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

	Close();
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
	case COMMAND_ARM_FAR:
		Far();
		break;
	case COMMAND_ARM_CLOSE:
		Close();
		break;
	case COMMAND_ARM_INTAKE:
		bIsIntaking = true;
		Intake(localMessage.params.armParams.direction);
		break;
	case COMMAND_AUTONOMOUS_INTAKE:
		AutoIntake();
		break;
	case COMMAND_AUTONOMOUS_THROWUP:
		Throwup();
		break;
	case COMMAND_AUTONOMOUS_SHOOT:
		AutoPos();
		break;
	default:
		break;
	}

	if(!bIsIntaking){
		if(pArmIntakeMotor->GetOutputVoltage()!=0){
			pArmIntakeMotor->Set(0); // This is how the intake motor stops
			pArmCenterMotor->Set(0);
			pArmPID->SetSetpoint(bottomEncoderPos);
		}
	}


}

void Arm::Close(){
	pArmPID->SetSetpoint(closeEncoderPos);
}

void Arm::Far(){
	pArmPID->SetSetpoint(farEncoderPos);
}

int Arm::GetPulseWidthPosition(){
	return pInstance->pArmLeverMotor->GetPulseWidthPosition();
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

void Arm::AutoIntake(){
	pArmIntakeMotor->Set(fIntakeInSpeed);
	pArmCenterMotor->Set(fCenterSpeed);
	pArmPID->SetSetpoint(intakeEncoderPos);
	Timer* timeout = new Timer();
	timeout->Start();
	while(pArmIntakeMotor->GetOutputCurrent()<fMaxIntakeCurrent && timeout->Get()<5){
		Wait(.0005);
	}
	delete timeout;
	pArmIntakeMotor->Set(0);
	pArmCenterMotor->Set(0);
	pArmPID->SetSetpoint(bottomEncoderPos);
	SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
}

void Arm::Throwup(){
	pArmIntakeMotor->Set(fIntakeOutSpeed);
	Wait(.5);
	SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
}

void Arm::AutoPos(){
	pArmPID->SetSetpoint(farEncoderPos);
	Wait(1);
	SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
}

void Arm::OnStateChange(){
	switch(localMessage.command) {
	case COMMAND_ROBOT_STATE_AUTONOMOUS:
		pArmPID->Enable();
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

