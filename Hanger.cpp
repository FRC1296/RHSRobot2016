/*
 * Hanger.cpp
 *
 *  Created on: Feb 27, 2016
 *      Author: Jacob
 */

#include <Hanger.h>
#include <RobotParams.h>
Hanger::Hanger() : ComponentBase(HANGER_TASKNAME, HANGER_QUEUE, HANGER_PRIORITY){
	pHangerMotor = new CANTalon(CAN_HANGER_MOTOR);
	pHangerMotor->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);
	pHangerMotor->SetControlMode(CANTalon::kPercentVbus);

	hs = new HangerSequence();

	solenoid = new Solenoid(CAN_PCM_JAW,SOL_HANGER);
	//currentState = NOT_DEPLOYED; // This is what it should start with
	currentState = RAISING; // TODO change this back
	pHangTimer = new Timer();

	pTask = new Task(HANGER_TASKNAME, &Hanger::StartTask, this);
}

Hanger::~Hanger() {
	delete pHangerMotor;
}

void Hanger::Run(){
	switch(localMessage.command) {
	case COMMAND_HANGER_HANG:
		Hang();
		break;
	case COMMAND_HANGER_SOLENOID_ENABLE:
		solenoid->Set(true);
		break;
	case COMMAND_HANGER_SOLENOID_DISABLE:
		solenoid->Set(false);
		break;

	default:
		if(currentState == RAISING){
			pHangerMotor->Set(0);
		}
		break;
	}

	switch(currentState){
	case NOT_DEPLOYED:
		// do nothing
		break;
	case DEPLOYED_AND_WAITING:
		if(!hs->IsRunning()){
			pHangTimer->Start();
		}
		if(pHangTimer->Get()>fAirTime){
			currentState = RAISING;
		}
		break;
	case RAISING:
		// do nothing
		break;
	default:
		break;
	}

}

void Hanger::Hang(){
	if(DriverStation::GetInstance().GetMatchTime()-fTeleopTime+fActivateTimeLeft<0){
		//return; // TODO test this
	}

	switch(currentState){
	case NOT_DEPLOYED:
		hs->StartSequence();
		pHangTimer->Stop();
		pHangTimer->Reset();

		currentState = DEPLOYED_AND_WAITING;
		break;
	case DEPLOYED_AND_WAITING:
		// do nothing
		break;
	case RAISING:
		pHangerMotor->Set(fRaiseSpeed);
		break;
	default:
		break;
	}
}

void Hanger::OnStateChange(){
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
