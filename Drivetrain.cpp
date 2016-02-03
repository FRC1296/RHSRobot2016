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

using namespace std;

Drivetrain::Drivetrain() :
		ComponentBase(DRIVETRAIN_TASKNAME, DRIVETRAIN_QUEUE,
				DRIVETRAIN_PRIORITY) {

	pLeftMotor = new CANTalon(CAN_DRIVETRAIN_LEFT_MOTOR);
	pRightMotor = new CANTalon(CAN_DRIVETRAIN_RIGHT_MOTOR);

	wpi_assert(pLeftMotor && pRightMotor);

	// setup for closed loop operation with VP encoders

	pLeftMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
	pLeftMotor->ConfigEncoderCodesPerRev(TALON_COUNTSPERREV);
	pLeftMotor->SelectProfileSlot(0);
	pLeftMotor->SetPID(TALON_PTERM, TALON_ITERM, TALON_DTERM, TALON_FTERM);		// PIDF
	pLeftMotor->SetIzone(TALON_IZONE);
	pLeftMotor->SetCloseLoopRampRate(TALON_MAXRAMP);
	pLeftMotor->SetInverted(false);
	pLeftMotor->SetControlMode(CANTalon::kSpeed);
	//pLeftMotor->SetControlMode(CANTalon::kPercentVbus);

	// setup for closed loop operation with VP encoders

	pRightMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
	pRightMotor->ConfigEncoderCodesPerRev(TALON_COUNTSPERREV);
	pRightMotor->SelectProfileSlot(0);
	pRightMotor->SetPID(TALON_PTERM, TALON_ITERM, TALON_DTERM, TALON_FTERM);	// PIDF
	pRightMotor->SetIzone(TALON_IZONE);
	pRightMotor->SetCloseLoopRampRate(TALON_MAXRAMP);
	pRightMotor->SetInverted(false);
	pRightMotor->SetControlMode(CANTalon::kSpeed);
	//pRightMotor->SetControlMode(CANTalon::kPercentVbus);

	wpi_assert(pLeftMotor->IsAlive());
	wpi_assert(pRightMotor->IsAlive());
	//bUnderServoControl = false;
	bUnderServoControl = true;

	pGyro = new ADXRS453Z;
	wpi_assert(pGyro);

	pCamera = new PixyCam();
	wpi_assert(pCamera);

	pAutoTimer = new Timer();
	wpi_assert(pAutoTimer);
	pAutoTimer->Start();

	pTask = new Task(DRIVETRAIN_TASKNAME, &Drivetrain::StartTask, this);
	wpi_assert(pTask);
}

Drivetrain::~Drivetrain()			//Destructor
{
	delete (pTask);
	delete pLeftMotor;
	delete pRightMotor;
	delete pGyro;
}

void Drivetrain::OnStateChange()			//Handles state changes
{
	switch(localMessage.command) {
	case COMMAND_ROBOT_STATE_AUTONOMOUS:
		if(bUnderServoControl)
		{
			pLeftMotor->Set(fNextLeft * FULLSPEED_FROMTALONS);
			pRightMotor->Set(fNextRight * FULLSPEED_FROMTALONS);
		}
		else
		{
			pLeftMotor->Set(fNextLeft);
			pRightMotor->Set(fNextRight);
		}
		break;

	case COMMAND_ROBOT_STATE_TEST:
		if(bUnderServoControl)
		{
			pLeftMotor->Set(0.0);
			pRightMotor->Set(0.0);
		}
		else
		{
			pLeftMotor->Set(0.0);
			pRightMotor->Set(0.0);
		}
		break;

	case COMMAND_ROBOT_STATE_TELEOPERATED:
		if(bUnderServoControl)
		{
			pLeftMotor->Set(0.0);
			pRightMotor->Set(0.0);
		}
		else
		{
			pLeftMotor->Set(0.0);
			pRightMotor->Set(0.0);
		}
		break;

	case COMMAND_ROBOT_STATE_DISABLED:
		if(bUnderServoControl)
		{
			pLeftMotor->Set(0.0);
			pRightMotor->Set(0.0);
		}
		else
		{
			pLeftMotor->Set(0.0);
			pRightMotor->Set(0.0);
		}
		break;

	case COMMAND_ROBOT_STATE_UNKNOWN:
		if(bUnderServoControl)
		{
			pLeftMotor->Set(0.0);
			pRightMotor->Set(0.0);
		}
		else
		{
			pLeftMotor->Set(0.0);
			pRightMotor->Set(0.0);
		}
		break;

	default:
		if(bUnderServoControl)
		{
			pLeftMotor->Set(0.0);
			pRightMotor->Set(0.0);
		}
		else
		{
			pLeftMotor->Set(0.0);
			pRightMotor->Set(0.0);
		}
		break;
	}
}

///fNextLeft + , fNextRight -
void Drivetrain::Run() {
	switch(localMessage.command) {
	case COMMAND_DRIVETRAIN_DRIVE_TANK:
		bDrivingStraight = false;
		bTurning = false;

		if((iLoop++ % 100) == 0)
		{
			SmartDashboard::PutNumber("Left Velocity", pLeftMotor->GetEncVel());
			SmartDashboard::PutNumber("Right Velocity", pRightMotor->GetEncVel());
		}

		if(bUnderServoControl)
		{
			pLeftMotor->Set(pow(localMessage.params.tankDrive.left, 3.0) * FULLSPEED_FROMTALONS);
			pRightMotor->Set(-pow(localMessage.params.tankDrive.right, 3.0) * FULLSPEED_FROMTALONS);
		}
		else
		{
			pLeftMotor->Set(pow(localMessage.params.tankDrive.left,3.0));
			pRightMotor->Set(-pow(localMessage.params.tankDrive.right, 3.0));
		}
		break;

	case COMMAND_DRIVETRAIN_DRIVE_CHEEZY:
		bTurning = false;
		bDrivingStraight = false;
		RunCheezyDrive(localMessage.params.cheezyDrive.wheel,
				localMessage.params.cheezyDrive.throttle,
				localMessage.params.cheezyDrive.spin);
		break;

	case COMMAND_DRIVETRAIN_STRAIGHT:
		bTurning = false;
		StartStraightDrive(localMessage.params.autonomous.driveSpeed, localMessage.params.autonomous.timeout);
		break;

	case COMMAND_DRIVETRAIN_TURN:
		bDrivingStraight = false;
		StartTurn(localMessage.params.autonomous.turnAngle,localMessage.params.autonomous.timeout);
		break;

	case COMMAND_DRIVETRAIN_STOP:
		bDrivingStraight = false;
		bTurning = false;
		fNextLeft = 0.0;
		fNextRight = 0.0;
		if(bUnderServoControl)
		{
			pLeftMotor->Set(fNextLeft * FULLSPEED_FROMTALONS);
			pRightMotor->Set(fNextRight * FULLSPEED_FROMTALONS);
		}
		else
		{
			pLeftMotor->Set(fNextLeft);
			pRightMotor->Set(fNextRight);
		}
		break;

	case COMMAND_SYSTEM_MSGTIMEOUT:
	default:
		break;
	}

	if(bDrivingStraight)
	{
		IterateStraightDrive();
	}

	if(bTurning)
	{
		IterateTurn();
	}
}

void Drivetrain::ArcadeDrive(float x, float y) {
	//TODO: add speed reduction
	if(bUnderServoControl)
	{
		pLeftMotor->Set(y + x / 2 * FULLSPEED_FROMTALONS);
		pRightMotor->Set(-(y - x / 2 * FULLSPEED_FROMTALONS));
	}
	else
	{
		pLeftMotor->Set(y + x / 2);
		pRightMotor->Set(-(y - x / 2));
	}
}
void Drivetrain::MeasuredMove(float speed, float targetDist) {
#if 0
	pGyro->Zero();
	encoder->Reset();
	bool isFinished = false;
	while(!isFinished)
	{
		float coveredDist = encoder->GetDistance();
		float remainingDist = targetDist - coveredDist;
		float adjustment = pGyro->GetAngle() / recoverStrength;
		//glorified arcade drive
		if(targetDist > 0 && remainingDist > distError)
		{
			//if headed in positive direction
			fNextLeft = (1.0 + adjustment) * speed;
			fNextRight = (-1.0 + adjustment) * speed;
		}
		else if(targetDist < 0 && remainingDist < -distError)
		{
			//if headed in negative direction
			fNextLeft = (-1.0 + adjustment) * speed;
			fNextRight = (1.0 + adjustment) * speed;
		}
		else
		{
			fNextLeft = 0.0;
			fNextRight = 0.0;
			isFinished = true;
		}
		if(bUnderServoControl)
		{
			pLeftMotor->Set(fNextLeft * FULLSPEED_FROMTALONS);
			pRightMotor->Set(fNextRight * FULLSPEED_FROMTALONS);
		}
		else
		{
			pLeftMotor->Set(fNextLeft);
			pRightMotor->Set(fNextRight);
		}

		SmartDashboard::PutNumber("Covered Distance", coveredDist);
		SmartDashboard::PutNumber("Remaining Distance", remainingDist);
		SmartDashboard::PutNumber("Angle Adjustment", adjustment);
	}

	SmartDashboard::PutString("Covered Distance", "Not operating");
	SmartDashboard::PutString("Remaining Distance", "Not operating");
	SmartDashboard::PutString("Angle Adjustment", "Not operating");
	printf("Finished moving %f inches", targetDist);
#endif
}
void Drivetrain::StartStraightDrive(float speed, float time)
{
	pAutoTimer->Reset();
	//DO NOT RESET THE GYRO EVER. only zeroing.
	pGyro->Zero();

	fStraightDriveSpeed = speed;
	fStraightDriveTime = time;
	bDrivingStraight = true;
	bTurning = false;
}

void Drivetrain::IterateStraightDrive(void)
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
		if(bUnderServoControl)
		{
			pLeftMotor->Set(0.0);
			pRightMotor->Set(0.0);
		}
		else
		{
			pLeftMotor->Set(0.0);
			pRightMotor->Set(0.0);
		}
	}
}

void Drivetrain::StartTurn(float angle, float time)
{
	pAutoTimer->Reset();
	//DO NOT RESET THE GYRO EVER. only zeroing.
	pGyro->Zero();

	fTurnAngle = angle + pGyro->GetAngle();
	fTurnTime = time;
	bDrivingStraight = false;
	bTurning = true;
}

void Drivetrain::IterateTurn(void)
{
	float motorValue;
	float degreesLeft;

	if ((pAutoTimer->Get() < fTurnTime) && ISAUTO)
	{
		//if you don't disable this during non-auto, it will keep trying to turn during teleop. Not fun.
		degreesLeft = fTurnAngle - pGyro->GetAngle();

		if ((degreesLeft < angleError) && (degreesLeft > -angleError))
		{
			bTurning = false;
			motorValue = 0.0;
		}
		else
		{
			motorValue = degreesLeft * turnAngleSpeedMultiplyer;
			ABLIMIT(motorValue, turnSpeedLimit);
		}
	}
	else
	{
		bTurning = false;
		motorValue = 0.0;
	}

	if(bUnderServoControl)
	{
		pLeftMotor->Set(motorValue * FULLSPEED_FROMTALONS);
		pRightMotor->Set(motorValue * FULLSPEED_FROMTALONS);
	}
	else
	{
		pLeftMotor->Set(motorValue);
		pRightMotor->Set(motorValue);
	}

	SmartDashboard::PutNumber("Angle Error", 0.0);
	SmartDashboard::PutNumber("Turn Speed", 0.0);
}

void Drivetrain::StraightDrive(float speed, float time) {
	MessageCommand command = COMMAND_AUTONOMOUS_RESPONSE_OK;
	pAutoTimer->Reset();
	//DO NOT RESET THE GYRO EVER. only zeroing.
	pGyro->Zero();

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
		pLeftMotor->Set(0.0);
		pRightMotor->Set(0.0);
	}
	else
	{
		pLeftMotor->Set(0.0);
		pRightMotor->Set(0.0);
	}

	SendCommandResponse(command);
}


void Drivetrain::StraightDriveLoop(float speed) {
	float adjustment = pGyro->GetAngle() * recoverStrength;
	//glorified arcade drive
	if (speed > 0.0)
	{
		/* if headed in positive direction
		 * +angle requires more power on the fNextRight to fix
		 * -angle, fNextLeft
		 */
		fNextLeft = (1.0 - adjustment) * speed;
		fNextRight = (-1.0 - adjustment) * speed;
	}
	else if (speed < 0.0)
	{
		/* if headed in negative direction
		 * +angle requires more power on the fNextLeft to fix
		 * -angle, fNextRight
		 */
		fNextLeft = (1.0 + adjustment) * speed;
		fNextRight = (-1.0 + adjustment) * speed;
	}
	else
	{
		fNextLeft = 0.0;
		fNextRight = 0.0;
	}

	ABLIMIT(fNextLeft, 1.0);
	ABLIMIT(fNextRight, 1.0);

	if(bUnderServoControl)
	{
		pLeftMotor->Set(fNextLeft * FULLSPEED_FROMTALONS);
		pRightMotor->Set(fNextRight * FULLSPEED_FROMTALONS);
	}
	else
	{
		pLeftMotor->Set(fNextLeft);
		pRightMotor->Set(fNextRight);
	}
}

bool Drivetrain::GetGyroAngle()
{
	return pGyro->GetAngle();
}

void Drivetrain::RunCheezyDrive(float fWheel, float fThrottle, float fSpin)
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
		fThrottleQ = pow(fThrottleQ, 3.0);
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
		pLeftMotor->Set(fLeftQ * FULLSPEED_FROMTALONS);
		pRightMotor->Set(-fRightQ * FULLSPEED_FROMTALONS);
	}
	else
	{
		pLeftMotor->Set(fLeftQ);
		pRightMotor->Set(-fRightQ);
	}
}

