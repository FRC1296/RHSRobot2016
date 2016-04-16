/** \file
 * Implementation of class to drive the pallet jack.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control the pallet jack's wheels.
 *
 * The task receives messages form the main robot class and runs the wheels.
 * Special commands use a pGyro and quadrature encoder to drive straight X feet
 * or to turn X degrees.
 *
 * Motor orientations:
 * left +
 * right -
 */

#include <math.h>
#include <assert.h>
#include <ComponentBase.h>

#include <string>
#include <iostream>
#include <algorithm>

#include "Drivetrain.h"			//For the local header file
#include "RobotParams.h"
#include "Arm.h"


using namespace std;

Drivetrain::Drivetrain() :
		ComponentBase(DRIVETRAIN_TASKNAME, DRIVETRAIN_QUEUE,
				DRIVETRAIN_PRIORITY) {

	fMaxVelLeft = 0;
	fMaxVelRight = 0;

	pRunTimer = new Timer();
	pRunTimer->Start();

	pLeftOneMotor = new DriveTalon(CAN_DRIVETRAIN_LEFTONE_MOTOR);
	pRightOneMotor = new DriveTalon(CAN_DRIVETRAIN_RIGHTONE_MOTOR);
	pLeftTwoMotor = new CANTalon(CAN_DRIVETRAIN_LEFTTWO_MOTOR);
	pRightTwoMotor = new CANTalon(CAN_DRIVETRAIN_RIGHTTWO_MOTOR);
	wpi_assert(pLeftOneMotor && pRightOneMotor && pLeftTwoMotor && pRightTwoMotor);

	pAPixy = new AnalogPixy(0,4,0);
	pFrontPixy = new AnalogPixy(1,3,0);

	//pCamera = new PixyCam();
	//wpi_assert(pCamera);

	// setup for closed loop operation with VP encoders
	pLeftOneMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
	pLeftOneMotor->ConfigEncoderCodesPerRev(TALON_COUNTSPERREV);
	pLeftOneMotor->SelectProfileSlot(0);
	pLeftOneMotor->SetPID(TALON_PTERM_L, TALON_ITERM_L, TALON_DTERM_L, TALON_FTERM_L);		// PIDF
	pLeftOneMotor->SetIzone(TALON_IZONE);
	pLeftOneMotor->SetCloseLoopRampRate(TALON_MAXRAMP);
	pLeftOneMotor->SetInverted(false);
	pLeftOneMotor->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);
	pLeftOneMotor->SetControlMode(CANTalon::kPercentVbus);

	pLeftTwoMotor->SetControlMode(CANSpeedController::kFollower);
	pLeftTwoMotor->Set(CAN_DRIVETRAIN_LEFTONE_MOTOR);

	// setup for closed loop operation with VP encoders

	pRightOneMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
	pRightOneMotor->ConfigEncoderCodesPerRev(TALON_COUNTSPERREV);
	pRightOneMotor->SelectProfileSlot(0);
	pRightOneMotor->SetPID(TALON_PTERM_R, TALON_ITERM_R, TALON_DTERM_R, TALON_FTERM_R);
	pRightOneMotor->SetIzone(TALON_IZONE);
	pRightOneMotor->SetCloseLoopRampRate(TALON_MAXRAMP);
	pRightOneMotor->SetInverted(false);
	pRightOneMotor->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);
	pRightOneMotor->SetControlMode(CANTalon::kPercentVbus);

	pRightTwoMotor->SetControlMode(CANSpeedController::kFollower);
	pRightTwoMotor->Set(CAN_DRIVETRAIN_RIGHTONE_MOTOR);

	wpi_assert(pLeftOneMotor->IsAlive());
	wpi_assert(pRightOneMotor->IsAlive());
	wpi_assert(pLeftTwoMotor->IsAlive());
	wpi_assert(pRightTwoMotor->IsAlive());
	bUnderServoControl = false;

	pGyro = new PoofGyro(SPI::kOnboardCS3);
	wpi_assert(pGyro);

	pAutoTimer = new Timer();
	wpi_assert(pAutoTimer);
	pAutoTimer->Start();

	pCheezy = new CheezyLoop();

	pLaserReturn = new DigitalInput(DIO_DRIVETRAIN_RED_SENSOR);

	pTask = new Task(DRIVETRAIN_TASKNAME, &Drivetrain::StartTask, this);
	wpi_assert(pTask);

	pTurnPIDOutput = new PIDSearchOutput(pLeftOneMotor,
			pRightOneMotor, true);
	//wpi_assert(pSearchPIDOutput);
	//pSearchPID = new PIDController(.075, 0.001, .025, pCamera, pSearchPIDOutput, .05f);
	pTurnPID = new PIDController(3, 0.0, 0.7, pGyro, pTurnPIDOutput, .05f);// .1 0 .1
	pTurnPID->SetTolerance(2.0);
	//pSearchPID = new PIDController(.18,0,.7, pCamera, pSearchPIDOutput, .05f);
	//wpi_assert(pSearchPID);

}

Drivetrain::~Drivetrain()			//Destructor
{
	delete (pTask);
	delete pLeftOneMotor;
	delete pRightOneMotor;
	delete pLeftTwoMotor;
	delete pRightTwoMotor;
	delete pGyro;
	//delete pSearchPID;
	//delete pSearchPIDOutput;
	delete pTurnPID;
	delete pTurnPIDOutput;
}

void Drivetrain::OnStateChange()			//Handles state changes
{
	fMaxVelLeft = 0;
	fMaxVelRight = 0;

	switch(localMessage.command) {
		case COMMAND_ROBOT_STATE_AUTONOMOUS: // we use tank drive in auto, talons close the loop
			bSearching = false;
			bUnderServoControl = true;
			pLeftOneMotor->SetControlMode(CANTalon::kSpeed);
			pRightOneMotor->SetControlMode(CANTalon::kSpeed);
			pLeftOneMotor->Set(0.0);
			pRightOneMotor->Set(0.0);
			ZeroGyro();
			break;

		case COMMAND_ROBOT_STATE_TEST:
			bUnderServoControl = false;
			pLeftOneMotor->SetControlMode(CANTalon::kPercentVbus);
			pRightOneMotor->SetControlMode(CANTalon::kPercentVbus);
			pLeftOneMotor->Set(0.0);
			pRightOneMotor->Set(0.0);
			break;

		case COMMAND_ROBOT_STATE_TELEOPERATED:  // we use cheezy drive in teleop, it closes the loop
			bUnderServoControl = false;
			pLeftOneMotor->SetControlMode(CANTalon::kPercentVbus);
			pRightOneMotor->SetControlMode(CANTalon::kPercentVbus);
			pLeftOneMotor->Set(0.0);
			pRightOneMotor->Set(0.0);
			break;

		case COMMAND_ROBOT_STATE_DISABLED:
			bUnderServoControl = false;
			pLeftOneMotor->SetControlMode(CANTalon::kPercentVbus);
			pRightOneMotor->SetControlMode(CANTalon::kPercentVbus);
			pLeftOneMotor->Set(0.0);
			pRightOneMotor->Set(0.0);
			break;

		case COMMAND_ROBOT_STATE_UNKNOWN:
		default:
			bUnderServoControl = false;
			pLeftOneMotor->SetControlMode(CANTalon::kPercentVbus);
			pRightOneMotor->SetControlMode(CANTalon::kPercentVbus);
			pLeftOneMotor->Set(0.0);
			pRightOneMotor->Set(0.0);
			break;
	}
}

///fNextLeft + , fNextRight -
void Drivetrain::Run() {
	//float fCentroid;
 	//SmartDashboard::PutBoolean("On Target", pCamera->GetCentroid(fCentroid));
  	//SmartDashboard::PutNumber("Centroid", fCentroid);

 	SmartDashboard::PutNumber("travelenc", pRightOneMotor->GetEncPosition());
 	SmartDashboard::PutNumber("distenc", fStraightDriveDistance * (TALON_COUNTSPERREV * REVSPERFOOT));
 	SmartDashboard::PutNumber("pixycam", pAPixy->Get());

 	SmartDashboard::PutNumber("velocity Right", pRightOneMotor->GetSpeed());
 	SmartDashboard::PutNumber("velocity Left", pLeftOneMotor->GetSpeed());
	SmartDashboard::PutNumber("Gyro", pGyro->GetAngle());

	SmartDashboard::PutBoolean("Red Sensor", pLaserReturn->Get());

	float avgAmp = 0;
	avgAmp+=pLeftOneMotor->GetOutputCurrent();
	avgAmp+=pLeftTwoMotor->GetOutputCurrent();
	avgAmp+=pRightOneMotor->GetOutputCurrent();
	avgAmp+=pRightTwoMotor->GetOutputCurrent();
	avgAmp/=4;

	SmartDashboard::PutNumber("ave drivetrain Amps", avgAmp);

	SmartDashboard::PutNumber("left raw", -pLeftOneMotor->GetEncPosition());
	SmartDashboard::PutNumber("right raw", pRightOneMotor->GetEncPosition());


 	if(abs(pRightOneMotor->GetSpeed())>abs(fMaxVelRight))
 		fMaxVelRight = pRightOneMotor->GetSpeed();

 	if(abs(pLeftOneMotor->GetSpeed())>abs(fMaxVelLeft))
 		fMaxVelLeft = pLeftOneMotor->GetSpeed();

 	SmartDashboard::PutNumber("Max V Left", fMaxVelLeft);
 	SmartDashboard::PutNumber("Max V Right", fMaxVelRight);

 	switch(localMessage.command) {
	case COMMAND_DRIVETRAIN_DRIVE_TANK:
		bDrivingStraight = false;
		bTurning = false;

		if(bUnderServoControl)
		{
			pLeftOneMotor->Set(-pow(localMessage.params.tankDrive.left, 3.0) * FULLSPEED_FROMTALONS);
			pRightOneMotor->Set(pow(localMessage.params.tankDrive.right, 3.0) * FULLSPEED_FROMTALONS);
		}
		else
		{
			pLeftOneMotor->Set(-pow(localMessage.params.tankDrive.left,3.0));
			pRightOneMotor->Set(pow(localMessage.params.tankDrive.right, 3.0));
		}
		break;

	case COMMAND_DRIVETRAIN_AUTO_MOVE:
		bDrivingStraight = false;
		bTurning = false;

		if(bUnderServoControl)
		{
			pLeftOneMotor->Set(localMessage.params.tankDrive.left * FULLSPEED_FROMTALONS);
			pRightOneMotor->Set(localMessage.params.tankDrive.right * FULLSPEED_FROMTALONS);
		}
		else
		{
			pLeftOneMotor->Set(localMessage.params.tankDrive.left);
			pRightOneMotor->Set(localMessage.params.tankDrive.right);
		}
		break;

		case COMMAND_DRIVETRAIN_DRIVE_SPLITARCADE:
			bTurning = false;
			bDrivingStraight = false;

			RunSplitArcade(localMessage.params.splitArcadeDrive.wheel,
					localMessage.params.splitArcadeDrive.throttle,
					localMessage.params.splitArcadeDrive.spin);

			// contribute to the cheezy Kalmanfilter

			if(fabs(localMessage.params.splitArcadeDrive.spin) > 0.05)
			{
				RunCheezyDrive(false, localMessage.params.splitArcadeDrive.wheel, 0.0, false);
			}
			else
			{
				RunCheezyDrive(false, localMessage.params.splitArcadeDrive.wheel,
						localMessage.params.splitArcadeDrive.throttle, false);
			}
			break;

	case COMMAND_DRIVETRAIN_DRIVE_CHEEZY:
		bTurning = false;
		bDrivingStraight = false;

		if(localMessage.params.cheezyDrive.throttle < 0.1 && localMessage.params.cheezyDrive.throttle > -0.1){

			bSearchLastFrame = false;
			pLeftOneMotor->ResetCurrentTimeout();
			pRightOneMotor->ResetCurrentTimeout();
			if(bSearching&&pAPixy->Get()!=5&&(pAPixy->Get()>=.02||pAPixy->Get()<=-.02)){
				if((int)(pRunTimer->Get()*2)%2){
					pLeftOneMotor->Set(pow(fabs(pAPixy->Get()),1.0/3.0)*.6*(pAPixy->Get()<0?-1:1));
					pRightOneMotor->Set(0);
				}else{
					pLeftOneMotor->Set(0);
					pRightOneMotor->Set(pow(fabs(pAPixy->Get()),1.0/3.0)*.6*(pAPixy->Get()<0?-1:1));
				}

			}else{
				if(!bRedSensing){
				RunCheezyDrive(true, localMessage.params.cheezyDrive.wheel,
						localMessage.params.cheezyDrive.throttle, localMessage.params.cheezyDrive.bQuickturn);
				}
			}


		}else{
			bRedSensing = false;
			if(!bRedSensing){
			RunCheezyDrive(true, localMessage.params.cheezyDrive.wheel,
					localMessage.params.cheezyDrive.throttle, localMessage.params.cheezyDrive.bQuickturn);
			}
		}

		if(localMessage.params.cheezyDrive.bQuickturn){
			bRedSensing = false;
		}


		break;

	case COMMAND_DRIVETRAIN_MSTRAIGHT:
 		bMeasuredMove = true;
 		bTurning = false;
 		StartStraightDrive(localMessage.params.autonomous.driveSpeed,
 				15.0, localMessage.params.autonomous.driveDistance);
 		RunCheezyDrive(false, 0.0, localMessage.params.autonomous.driveSpeed, false);
 		break;

	case COMMAND_DRIVETRAIN_STRAIGHT:
		bMeasuredMove = false;
		bTurning = false;
		StartStraightDrive(localMessage.params.autonomous.driveSpeed,
		 				localMessage.params.autonomous.timeout, 54.0);
		RunCheezyDrive(false, 0.0, localMessage.params.autonomous.driveSpeed, false);
		break;

	case COMMAND_DRIVETRAIN_TURN:
		bDrivingStraight = false;
		StartTurn(localMessage.params.autonomous.turnAngle,localMessage.params.autonomous.timeout);

		// contribute to cheezy Kalman filter

		if(localMessage.params.autonomous.turnAngle > 0.0)
		{
			RunCheezyDrive(false, 0.5, 0.0, false);
		}
		else
		{
			RunCheezyDrive(false, -0.5, 0.0, false);
		}
		break;

	case COMMAND_DRIVETRAIN_STOP:
		bDrivingStraight = false;
		bTurning = false;
		fNextLeft = 0.0;
		fNextRight = 0.0;
		pLeftOneMotor->Set(0.0);
		pRightOneMotor->Set(0.0);
		RunCheezyDrive(false, 0.0, 0.0, false);
		break;

	case COMMAND_SYSTEM_CONSTANTS:
		fBatteryVoltage = localMessage.params.system.fBattery;
		break;

	case COMMAND_AUTONOMOUS_SEARCHGOAL:
		bSearching = localMessage.params.armParams.direction;
		if(ISAUTO){
			printf("is auto\n");
			bSearching = true;
			Search();
			bSearching = false;
		}

		break;

	case COMMAND_AUTONOMOUS_SEARCHBALL:
			BallSearch();
		break;
	case COMMAND_DRIVETRAIN_REDSENSE:
		pAutoTimer->Reset();
		pAutoTimer->Start();
		bRedSensing = true;
		break;

	case COMMAND_SYSTEM_MSGTIMEOUT:
		bSearchLastFrame = false;
		break;

	default:

		break;
	}


	if(bTurning)
	{
		IterateTurn();
	}

	if(bRedSensing){
		RedSense();
	}
}

void Drivetrain::ArcadeDrive(float x, float y) {
	//TODO: add speed reduction
	if(bUnderServoControl)
	{
		pLeftOneMotor->Set(-(y + x / 2 * FULLSPEED_FROMTALONS));
		pRightOneMotor->Set((y - x / 2 * FULLSPEED_FROMTALONS));
	}
	else
	{
		pLeftOneMotor->Set(-(y + x / 2));
		pRightOneMotor->Set((y - x / 2));
	}
}

void Drivetrain::StartStraightDrive (float speed, float time, float distance)
{
	pAutoTimer->Reset();
	pAutoTimer->Start();
	//DO NOT RESET THE GYRO EVER. only zeroing.
	//pGyro->Zero();		//DO NOT RESET THE GYRO EVER. only zeroing.
	pLeftOneMotor->SetEncPosition(0);
	while(pRightOneMotor->GetEncPosition()!=0){
		pRightOneMotor->SetEncPosition(0);
	}


	fStraightDriveSpeed = speed;
	fStraightDriveTime = time;
	bDrivingStraight = true;
	fStraightDriveDistance = (distance-.928)/7.22/1024*TALON_COUNTSPERREV;  //TODO need to calculate the stop distance more carefully
	bTurning = false;

	IterateStraightDrive();
}

void Drivetrain::IterateStraightDrive(void)
{
	if(bMeasuredMove)
	{
		while(true)
		{
			if ((pAutoTimer->Get() < fStraightDriveTime) && ISAUTO)
			{
				//SmartDashboard::PutNumber("travelenc", pRightOneMotor->GetEncPosition());
				//SmartDashboard::PutNumber("distenc", fStraightDriveDistance * (TALON_COUNTSPERREV * REVSPERFOOT));
				//SmartDashboard::PutNumber("velocity Right", pRightOneMotor->GetSpeed());
				//SmartDashboard::PutNumber("velocity Left", pLeftOneMotor->GetSpeed());

				if(pRightOneMotor->GetEncPosition() < (int)(fStraightDriveDistance * (TALON_COUNTSPERREV * REVSPERFOOT))
						&& pRightOneMotor->GetEncPosition() > -(int)(fStraightDriveDistance * (TALON_COUNTSPERREV * REVSPERFOOT)))
				{
					StraightDriveLoop(fStraightDriveSpeed);
					Wait(.005);
				}
				else
				{
					printf("reached limit traveled %d , needed %d \n", pRightOneMotor->GetEncPosition(),
							(int)(fStraightDriveDistance * (TALON_COUNTSPERREV * REVSPERFOOT)));
					break;
				}
			}
			else
			{
				printf("not auto or timed out \n");
				break;
			}
		}

		bDrivingStraight = false;
		bMeasuredMove = false;
		pLeftOneMotor->Set(0.0);
		pRightOneMotor->Set(0.0);
		RunCheezyDrive(true, 0.0, 0.0, false);	// contribute to cheezy drive Kalman filter
		SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
	}
	else
	{
		if ((pAutoTimer->Get() < fStraightDriveTime) && ISAUTO)
		{
			StraightDriveLoop(fStraightDriveSpeed);
		}
		else
		{
			bDrivingStraight = false;
			fNextLeft = 0.0;
			fNextRight = 0.0;
			pLeftOneMotor->Set(0.0);
			pRightOneMotor->Set(0.0);
			RunCheezyDrive(true, 0.0, 0.0, false);  // contribute to cheezy drive Kalman filter
		}
	}
}

void Drivetrain::BallSearch(){
	Timer* timer = new Timer();
	while(timer->Get()>.5){
		if(pFrontPixy->Get()==5){
			printf("Cant see the ball\n");
			// Cant see ball
			break;
		}
		if(Arm::GetIntakeCurrent()>3){
			RunCheezyDrive(true,0,0,false);
			timer->Start();
		}else{
			RunCheezyDrive(true,pFrontPixy->Get()*3,.3,false);
			timer->Stop();
			timer->Reset();
		}

	}
	delete timer;
}

void Drivetrain::Search(){
	MessageCommand command = COMMAND_AUTONOMOUS_RESPONSE_OK;
	float timer = pRunTimer->Get();
	while(true){
		if(!ISAUTO){
			return;
		}
	if(pAPixy->Get()!=5&&(pAPixy->Get()>=.02||pAPixy->Get()<=-.02)&&(pRunTimer->Get()-timer)<.25){
		timer = pRunTimer->Get();
		if((int)(pRunTimer->Get()*2)%2){
			pLeftOneMotor->Set(pow(fabs(pAPixy->Get()),1.0/3.0)*.6*(pAPixy->Get()<0?-1:1)*FULLSPEED_FROMTALONS);
			pRightOneMotor->Set(0);
		}else{
			pLeftOneMotor->Set(0);
			pRightOneMotor->Set(pow(fabs(pAPixy->Get()),1.0/3.0)*.6*(pAPixy->Get()<0?-1:1)*FULLSPEED_FROMTALONS);
		}

	}else{
		pLeftOneMotor->Set(0);
		pRightOneMotor->Set(0);
		if((pRunTimer->Get()-timer)<.25){
			break;
		}

	}
	}
	if(ISAUTO){
		SendCommandResponse(command);
	}
}

//void Drivetrain::

void Drivetrain::StartTurn(float angle, float time)
{
	pAutoTimer->Reset();
	pAutoTimer->Start();

	if(pGyro->GetAngle()>0){
		while(pGyro->GetAngle()>180){
			pGyro->SetAngle(pGyro->GetAngle()-360);
		}
	}else{
		while(pGyro->GetAngle()<-180){
			pGyro->SetAngle(pGyro->GetAngle()+360);
		}
	}

	fTurnAngle = angle;
	//pGyro->Zero();
	fTurnTime = time;
	bDrivingStraight = false;
	bTurning = true;
	printf("setpoint %lf \n", fTurnAngle/45.0);
	pTurnPID->SetSetpoint(fTurnAngle/45.0);
	pTurnPID->Enable();

}

void Drivetrain::IterateTurn(void)
{
	if ((pTurnPID->OnTarget() == false) && (pAutoTimer->Get() < fTurnTime) && ISAUTO)
	{
		RunCheezyDrive(false, 0.0, 0.0, false);	// contribute to cheezy drive Kalman filter
	}
	else
	{
		bTurning = false;
		pTurnPID->Disable();
		SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
	}

}

void Drivetrain::StraightDrive(float speed, float time) {
	MessageCommand command = COMMAND_AUTONOMOUS_RESPONSE_OK;
	pAutoTimer->Reset();
	//DO NOT RESET THE GYRO EVER. only zeroing.
	fTurnAngle = pGyro->GetAngle();

	while ((pAutoTimer->Get() < time)
			&& ISAUTO)
	{
		StraightDriveLoop(speed);
		Wait(0.01);
	}

	fNextLeft = 0.0;
	fNextRight = 0.0;
	if(bUnderServoControl)
	{
		pLeftOneMotor->Set(0.0);
		pRightOneMotor->Set(0.0);
	}
	else
	{
		pLeftOneMotor->Set(0.0);
		pRightOneMotor->Set(0.0);
	}

	SendCommandResponse(command);
}

void Drivetrain::ZeroGyro(){
	pGyro->Zero();
}

void Drivetrain::RedSense(){
	if(!pLaserReturn->Get()){
		bRedSensing = false;
		pLeftOneMotor->Set(0);
		pRightOneMotor->Set(0);
		RunCheezyDrive(false, 0.0, 0.0, false);	// contribute to cheezy drive Kalman filter

		if(ISAUTO){
			SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
		}
	}else{

		int time = pAutoTimer->Get()*2;
		if(time%2==0){
			pLeftOneMotor->Set(0.0);
			pRightOneMotor->Set(.3f);
		}else{
			pLeftOneMotor->Set(0.22f);
			pRightOneMotor->Set(0.0);
		}

		RunCheezyDrive(false, 0.0, 0.0, false);	// contribute to cheezy drive Kalman filter
	}

}

void Drivetrain::StraightDriveLoop(float speed)
{

	float offset = (pGyro->GetAngle()-fTurnAngle)/45;
	pLeftOneMotor->Set(-(speed-(offset+fAccum)) * FULLSPEED_FROMTALONS);
	pRightOneMotor->Set((speed+(offset+fAccum)) * FULLSPEED_FROMTALONS);

#if 0
	fAccum += offset;
	fAccum /= 2;
#endif
	//RunCheezyDrive(true, -(pGyro->GetAngle()-fTurnAngle)/45, speed, false);

}

bool Drivetrain::GetGyroAngle()
{
	return pGyro->GetAngle();
}

void Drivetrain::RunSplitArcade(float fWheel, float fThrottle, float fSpin)
{
	float fLeftQ;
	float fRightQ;
	float fWheelQ = fWheel;
	float fThrottleQ = fThrottle;
	float fSpinQ = fSpin;
	float fScale = 1.0;

	// establish dead zones

	if(fabs(fThrottleQ) < 0.05)
	{
		fThrottleQ = 0.0;
	}
	else
	{
		fThrottleQ = pow(fThrottleQ, 5.0);
	}

	if(fabs(fWheelQ) < 0.05)
	{
		fWheelQ = 0.0;
	}

	if(fabs(fSpinQ) < 0.05)
	{
		fSpinQ = 0.0;
	}


	if(fSpinQ)
	{
		// spin in place modes

		fLeftQ = fSpinQ * 0.5;
		fRightQ = -fSpinQ * 0.5;
	}
	else
	{
		// larger delta at lower speeds

		fScale  = (float)sin(3.141519 / 2.0 * fabs((double)fWheelQ));

		// which quadrant are we operating in?

		if(fThrottleQ >= 0.0)
		{
			if(fWheelQ >= 0.0)
			{
				// forward and turn right

				fLeftQ = fThrottleQ;
				fRightQ = fThrottleQ - fScale * (fWheelQ * fThrottleQ);
			}
			else
			{
				// forward and turn left

				fLeftQ = fThrottleQ + fScale * (fWheelQ * fThrottleQ);
				fRightQ = fThrottleQ;
			}
		}
		else
		{
			if(fWheelQ >= 0.0)
			{
				// backward and turn right

				fLeftQ = fThrottleQ;
				fRightQ = fThrottleQ - fScale * (fWheelQ * fThrottleQ);
			}
			else
			{
				// backward and turn left

				fLeftQ = fThrottleQ + fScale * (fWheelQ * fThrottleQ);
				fRightQ = fThrottleQ;
			}
		}
	}

	if(bUnderServoControl)
	{
		pLeftOneMotor->Set(-fLeftQ * FULLSPEED_FROMTALONS);
		pRightOneMotor->Set(fRightQ * FULLSPEED_FROMTALONS);
	}
	else
	{
		pLeftOneMotor->Set(-fLeftQ);
		pRightOneMotor->Set(fRightQ);
	}


}
void Drivetrain::RunCheezyDrive(bool bEnabled, float fWheel, float fThrottle, bool bQuickturn)
{
    struct DrivetrainGoal Goal;
    struct DrivetrainPosition Position;
    struct DrivetrainOutput Output;
    struct DrivetrainStatus Status;


    Goal.steering = ((bSearching)&&pAPixy->Get()!=5?pAPixy->Get()*(abs(pLeftOneMotor->GetSpeed())>150?.75:2):(bQuickturn?-pow(fWheel,3):-fWheel));   // not sure why
    Goal.throttle = fThrottle;
    Goal.quickturn = bQuickturn;
    Goal.control_loop_driving = false;
    Goal.highgear = false;
    Goal.left_velocity_goal = 0.0;
    Goal.right_velocity_goal = 0.0;
    Goal.left_goal = 0.0;
    Goal.right_goal = 0.0;

    Position.left_encoder = -pLeftOneMotor->GetEncPosition() * METERS_PER_COUNT;
    Position.right_encoder = pRightOneMotor->GetEncPosition() * METERS_PER_COUNT;
    Position.gyro_angle = pGyro->GetAngle() * 3.141519 / 180.0;
    Position.gyro_velocity = pGyro->GetRate() * 3.141519 / 180.0;
    Position.battery_voltage = fBatteryVoltage;
    Position.left_shifter_position = true;
    Position.right_shifter_position = false;

	SmartDashboard::PutNumber("Battery", fBatteryVoltage);
	SmartDashboard::PutNumber("angle rate", Position.gyro_velocity);
	SmartDashboard::PutNumber("angle", Position.gyro_angle);
	SmartDashboard::PutNumber("left encoder", Position.left_encoder);
	SmartDashboard::PutNumber("right encoder", Position.right_encoder);

    if(bEnabled)
    {
    	// if enabled and normal operation

    	pCheezy->Update(Goal, Position, Output, Status, true);
        pLeftOneMotor->Set(-Output.left_voltage / 12.0);
        pRightOneMotor->Set(Output.right_voltage / 12.0);
    }
    else
    {
        // if the robot is not running

    	pCheezy->Update(Goal, Position, Output, Status, false);
    }
}

CheezyLoop::CheezyLoop()
{
	bOutputEnabled = false;
	CheezyInit1296();  // initialize the cheezy drive code base

	pTask = new Task("tCheezy", &CheezyLoop::Run, this);
}

void CheezyLoop::Run(CheezyLoop *pInstance)
{
	 while(true)
	 {
		 Wait(0.005);

		 if(pInstance->bOutputEnabled)
		{
			 std::lock_guard<priority_recursive_mutex> sync(pInstance->mutexData);
			 CheezyIterate1296(&pInstance->currentGoal,
					 &pInstance->currentPosition,
					 &pInstance->currentOutput,
					 &pInstance->currentStatus);
		}
		else
		{
			 std::lock_guard<priority_recursive_mutex> sync(pInstance->mutexData);
			 CheezyIterate1296(&pInstance->currentGoal,
					 &pInstance->currentPosition,
					 NULL,
					 &pInstance->currentStatus);
		}
	 }
}



void CheezyLoop::Update(const DrivetrainGoal &goal,
    const DrivetrainPosition &position,
    DrivetrainOutput &output,
    DrivetrainStatus &status,
	bool bEnabled)
{
	std::lock_guard<priority_recursive_mutex> sync(mutexData);

	bOutputEnabled = bEnabled;
	currentGoal = goal;
	currentPosition = position;
	output = currentOutput;
	status = currentStatus;
}

CheezyLoop::~CheezyLoop(){
	delete pTask;
}
