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
	pShootTimer = new Timer();

	pLED = new Relay(1,Relay::kBothDirections);

	pArmLeverMotor = new CanArmTalon(CAN_ARM_LEVER_MOTOR);
	pArmLeverMotor->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);
	pArmLeverMotor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
	pArmPID = new PIDController(.0010,0,0,pArmLeverMotor, pArmLeverMotor, .05);

	pArmLeverMotor->SetInverted(true);
	pArmLeverMotor->SetIzone(TALON_IZONE);
	pArmLeverMotor->SetCloseLoopRampRate(TALON_MAXRAMP);
	pArmLeverMotor->SetControlMode(CANTalon::kPercentVbus);

	pArmIntakeMotor = new CANTalon(CAN_ARM_INTAKE_MOTOR);
	pArmIntakeMotor->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);

	wpi_assert(pArmLeverMotor->IsAlive());
	wpi_assert(pArmIntakeMotor->IsAlive());

	pTask = new Task(ARM_TASKNAME, &Arm::StartTask, this);
	wpi_assert(pTask);
	pArmLeverMotor->SetEncPosition(0);
	pArmPID->SetSetpoint(bottomEncoderPos);
	//Far();
}

Arm::~Arm() {
	delete pTask;
	delete pArmLeverMotor;
	delete pArmIntakeMotor;
	delete pArmPID;
	delete pInstance;
	delete pShootTimer;
	delete pLED;
}

void Arm::Run(){

	//printf("output error %f \n", pArmPID->GetAvgError());
	SmartDashboard::PutNumber("arm encoder", pArmLeverMotor->GetPulseWidthPosition());

	if(localMessage.command != COMMAND_AUTONOMOUS_INTAKE){

	}

	//bIsIntaking = false;
	switch(localMessage.command) {
	case COMMAND_ARM_FAR:
		Far();
		break;
	case COMMAND_ARM_MOVE_INTAKE:
		pArmPID->SetSetpoint(intakeEncoderPos);
	break;
	case COMMAND_ARM_MOVE_RIDE:
		pArmPID->SetSetpoint(bottomEncoderPos);
	break;
	case COMMAND_ARM_CLOSE:
		Close();
		break;
	case COMMAND_ARM_INTAKE:
		//bIsIntaking = true;
		//Intake(localMessage.params.armParams.direction);

		if(!bIntakePressedLastFrame){
			bIsIntaking = !bIsIntaking;
			bIntakePressedLastFrame = true;
			Intake(localMessage.params.armParams.direction);
		}
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

	case COMMAND_ARM_ENABLE:
		SmartDashboard::PutBoolean("ARM CURRENT", true);
		pArmLeverMotor->Enable();
		break;
	case COMMAND_ARM_SHOOT:
		IntakeShoot();
		break;

	case COMMAND_ARM_LEDOFF:
		pLED->Set(Relay::kOff);
		//printf("LED off\n");
		break;

	case COMMAND_ARM_LEDWHITE:
		pLED->Set(Relay::kForward);
		//printf("LED white\n");
		break;

	case COMMAND_ARM_LEDCOLOR:
		pLED->Set(Relay::kReverse);
		//printf("LED color\n");
		break;

	default:
		bIntakePressedLastFrame = false;
		break;
	}



	// For intake rollers
	if(!bIsIntaking){
		if(pArmPID->GetSetpoint() == farEncoderPos || pArmPID->GetSetpoint() == closeEncoderPos){
			pArmIntakeMotor->Set(0);
		}else{
			pArmIntakeMotor->Set(fIntakeIdleSpeed);
		}

		if(pArmPID->GetSetpoint() == intakeEncoderPos && !bIntakePressedLastFrame){
			pArmPID->SetSetpoint(bottomEncoderPos);
		}

	}

	// For shooting timing
	if(pShootTimer->Get()>shootDelay){
		//claw->Set(false);
		pShootTimer->Stop();
		pShootTimer->Reset();
		pArmPID->SetSetpoint(bottomEncoderPos);
	}

	if(pArmPID->GetSetpoint()==farEncoderPos){
		pLED->Set(Relay::kForward);
	}else{
		pLED->Set(Relay::kOff);
	}

}

void Arm::Close(){
	pArmPID->SetSetpoint(closeEncoderPos);
}

void Arm::Far(){
	pArmPID->SetSetpoint(farEncoderPos);
}

void Arm::IntakeShoot(){
	//claw->Set(true);
	Far();
	pShootTimer->Start();
}
int Arm::GetPulseWidthPosition(){
	return pInstance->pArmLeverMotor->GetPulseWidthPosition();
}
int Arm::GetEncPosition(){
	return pInstance->pArmLeverMotor->GetEncPosition();
}
int Arm::GetEncTarget(){
	return pInstance->pArmPID->GetSetpoint();
}

double Arm::GetIntakeCurrent(){
	return pInstance->pArmIntakeMotor->GetOutputCurrent();
}
void Arm::StopIntake(){
	pInstance->bIsIntaking = false;
}
// do we want to let the drivers run the intake and decide when to lower the arm?

void Arm::Intake(bool direction){
	if(direction){ // intaking
		if(pArmPID->GetSetpoint()<=bottomEncoderPos){
			pArmPID->SetSetpoint(intakeEncoderPos);
		}
		pArmIntakeMotor->Set(fIntakeInSpeed);

		//pArmPID->SetSetpoint(intakeEncoderPos);
	}else{ // throwing up
		pArmIntakeMotor->Set(fIntakeOutSpeed);
	}

}

void Arm::AutoIntake(){
	Intake(true);
	pArmIntakeMotor->Set(0);
	pArmPID->SetSetpoint(bottomEncoderPos);
	SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
}

void Arm::Throwup(){
	Intake(false);
	Wait(fAutoThrowupTime);
	pArmIntakeMotor->Set(0);
	SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
}

void Arm::AutoPos(){
	Far();
	Wait(fAutoTimeToArm);
	SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
}

void Arm::OnStateChange(){
	switch(localMessage.command) {
	case COMMAND_ROBOT_STATE_AUTONOMOUS:
		pArmPID->Enable();
		pArmPID->SetSetpoint(GetEncPosition());
		break;

	case COMMAND_ROBOT_STATE_TEST:
		pArmPID->Disable();
		break;

	case COMMAND_ROBOT_STATE_TELEOPERATED:
		pArmPID->Enable();
		pArmPID->SetSetpoint(bottomEncoderPos);
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

