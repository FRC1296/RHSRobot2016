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

#include "RobotParams.h"
#include "Drivetrain.h"

Drivetrain::Drivetrain() :
		ComponentBase(DRIVETRAIN_TASKNAME, DRIVETRAIN_QUEUE,
				DRIVETRAIN_PRIORITY) {

	pLeftMotor = new CANTalon(CAN_DRIVETRAIN_LEFT_MOTOR);
	pLeftMotorFollow = new CANTalon(CAN_DRIVETRAIN_LEFT_MOTOR_FOLLOW);
	pRightMotor = new CANTalon(CAN_DRIVETRAIN_RIGHT_MOTOR);
	pRightMotorFollow = new CANTalon(CAN_DRIVETRAIN_RIGHT_MOTOR_FOLLOW);

	wpi_assert(pLeftMotor && pRightMotor && pLeftMotorFollow && pRightMotorFollow);

	// setup for closed loop operation with VP encoders

	pLeftMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
	pLeftMotor->ConfigEncoderCodesPerRev(TALON_COUNTSPERREV);
	pLeftMotor->SelectProfileSlot(0);
	pLeftMotor->SetPID(TALON_PTERM, TALON_ITERM, TALON_DTERM, TALON_FTERM);		// PIDF
	pLeftMotor->SetIzone(TALON_IZONE);
	pLeftMotor->SetCloseLoopRampRate(TALON_MAXRAMP);
	pLeftMotor->SetInverted(false);
	pLeftMotor->ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
	pLeftMotor->SetControlMode(CANTalon::kPercentVbus);

	// setup for closed loop operation with VP encoders

	pRightMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
	pRightMotor->ConfigEncoderCodesPerRev(TALON_COUNTSPERREV);
	pRightMotor->SelectProfileSlot(0);
	pRightMotor->SetPID(TALON_PTERM, TALON_ITERM, TALON_DTERM, TALON_FTERM);	// PIDF
	pRightMotor->SetIzone(TALON_IZONE);
	pRightMotor->SetCloseLoopRampRate(TALON_MAXRAMP);
	pRightMotor->SetInverted(false);
	pRightMotor->ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
	pRightMotor->SetControlMode(CANTalon::kPercentVbus);

	wpi_assert(pLeftMotor->IsAlive());
	wpi_assert(pRightMotor->IsAlive());

	pLeftMotorFollow->SetControlMode(CANTalon::kFollower);
	pLeftMotorFollow->Set(CAN_DRIVETRAIN_LEFT_MOTOR);
	pRightMotorFollow->SetControlMode(CANTalon::kFollower);
	pRightMotorFollow->Set(CAN_DRIVETRAIN_RIGHT_MOTOR);

	bUnderServoControl = false;

	pGyro = new ADXRS453Z;
	wpi_assert(pGyro);

	pCamera = new PixyCam();
	wpi_assert(pCamera);

	pAutoTimer = new Timer();
	wpi_assert(pAutoTimer);
	pAutoTimer->Start();

	pCheezy = new CheezyLoop();

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
		bUnderServoControl = true;
		pLeftMotor->SetControlMode(CANTalon::kSpeed);
		pRightMotor->SetControlMode(CANTalon::kSpeed);
		pLeftMotor->Set(0.0);
		pRightMotor->Set(0.0);
		break;

	case COMMAND_ROBOT_STATE_TEST:
		bUnderServoControl = false;
		pLeftMotor->SetControlMode(CANTalon::kPercentVbus);
		pRightMotor->SetControlMode(CANTalon::kPercentVbus);
		pLeftMotor->Set(0.0);
		pRightMotor->Set(0.0);
		break;

	case COMMAND_ROBOT_STATE_TELEOPERATED:
		bUnderServoControl = false;
		pLeftMotor->SetControlMode(CANTalon::kPercentVbus);
		pRightMotor->SetControlMode(CANTalon::kPercentVbus);
		pLeftMotor->Set(0.0);
		pRightMotor->Set(0.0);
		break;

	case COMMAND_ROBOT_STATE_DISABLED:
		bUnderServoControl = false;
		pLeftMotor->SetControlMode(CANTalon::kPercentVbus);
		pRightMotor->SetControlMode(CANTalon::kPercentVbus);
		pLeftMotor->Set(0.0);
		pRightMotor->Set(0.0);
		break;

	case COMMAND_ROBOT_STATE_UNKNOWN:
	default:
		bUnderServoControl = false;
		pLeftMotor->SetControlMode(CANTalon::kPercentVbus);
		pRightMotor->SetControlMode(CANTalon::kPercentVbus);
		pLeftMotor->Set(0.0);
		pRightMotor->Set(0.0);
		break;
	}
}

void Drivetrain::Run() {

	float fCentroid;

	SmartDashboard::PutBoolean("On Target", pCamera->GetCentroid(fCentroid));
	SmartDashboard::PutNumber("Centroid", fCentroid);

	SmartDashboard::PutNumber("travelenc", pRightMotor->GetEncPosition());
	SmartDashboard::PutNumber("distenc", fStraightDriveDistance * (TALON_COUNTSPERREV * REVSPERFOOT));

	switch(localMessage.command) {
	case COMMAND_DRIVETRAIN_DRIVE_TANK:
		bDrivingStraight = false;
		bTurning = false;

		if(bUnderServoControl)
		{
			pLeftMotor->Set(-pow(localMessage.params.tankDrive.left, 3.0) * FULLSPEED_FROMTALONS);
			pRightMotor->Set(pow(localMessage.params.tankDrive.right, 3.0) * FULLSPEED_FROMTALONS);
		}
		else
		{
			pLeftMotor->Set(-pow(localMessage.params.tankDrive.left,3.0));
			pRightMotor->Set(pow(localMessage.params.tankDrive.right, 3.0));
		}
		break;

	case COMMAND_DRIVETRAIN_AUTO_MOVE:
		bDrivingStraight = false;
		bTurning = false;

		if(bUnderServoControl)
		{
			pLeftMotor->Set(localMessage.params.tankDrive.left * FULLSPEED_FROMTALONS);
			pRightMotor->Set(localMessage.params.tankDrive.right * FULLSPEED_FROMTALONS);
		}
		else
		{
			pLeftMotor->Set(localMessage.params.tankDrive.left);
			pRightMotor->Set(localMessage.params.tankDrive.right);
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

		// contribute to the cheezy Kalmanfilter

		RunCheezyDrive(true, localMessage.params.cheezyDrive.wheel,
				localMessage.params.cheezyDrive.throttle,
				localMessage.params.cheezyDrive.bQuickturn);
		break;

	case COMMAND_DRIVETRAIN_MSTRAIGHT:
		bMeasuredMove = true;
		bTurning = false;
		StartStraightDrive(localMessage.params.autonomous.driveSpeed,
				15.0, localMessage.params.autonomous.driveDistance);

		// contribute to the cheezy Kalmanfilter

		RunCheezyDrive(false, 0.0, localMessage.params.autonomous.driveSpeed, false);
		break;

	case COMMAND_DRIVETRAIN_STRAIGHT:
		bMeasuredMove = false;
		bTurning = false;
		StartStraightDrive(localMessage.params.autonomous.driveSpeed,
				localMessage.params.autonomous.timeout, 54.0);

		// contribute to the cheezy Kalmanfilter

		RunCheezyDrive(false, 0.0, localMessage.params.autonomous.driveSpeed, false);
		break;

	case COMMAND_DRIVETRAIN_TURN:
		bDrivingStraight = false;
		StartTurn(localMessage.params.autonomous.turnAngle,localMessage.params.autonomous.timeout);

		// contribute to the cheezy Kalmanfilter

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

		// contribute to the cheezy Kalmanfilter

		RunCheezyDrive(false, 0.0, 0.0, false);
		break;

	case COMMAND_SYSTEM_CONSTANTS:
		fBatteryVoltage = localMessage.params.system.fBattery;
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
		pLeftMotor->Set(-(y + x / 2 * FULLSPEED_FROMTALONS));
		pRightMotor->Set(-y - x / 2 * FULLSPEED_FROMTALONS);
	}
	else
	{
		pLeftMotor->Set(-(y + x / 2));
		pRightMotor->Set(-y - x / 2);
	}
}

void Drivetrain::StartStraightDrive(float speed, float time, float distance)
{
	pAutoTimer->Reset();
	pGyro->Zero();		//DO NOT RESET THE GYRO EVER. only zeroing.
	pLeftMotor->SetEncPosition(0);
	pRightMotor->SetEncPosition(0);

	fStraightDriveSpeed = speed;
	fStraightDriveTime = time;
	fStraightDriveDistance = distance - 1.0;  //TODO need to calculate the stop distance more carefully
	bDrivingStraight = true;
	bTurning = false;
}

void Drivetrain::IterateStraightDrive(void)
{
	if(bMeasuredMove)
	{
		if ((pAutoTimer->Get() < fStraightDriveDistance) && ISAUTO)
		{
			SmartDashboard::PutNumber("travelenc", pRightMotor->GetEncPosition());
			SmartDashboard::PutNumber("distenc", fStraightDriveDistance * (TALON_COUNTSPERREV * REVSPERFOOT));

			if(pRightMotor->GetEncPosition() < (int)(fStraightDriveDistance * (TALON_COUNTSPERREV * REVSPERFOOT)))
			{
				StraightDriveLoop(fStraightDriveSpeed);
			}
			else
			{
				bDrivingStraight = false;
				bMeasuredMove = false;
				fNextLeft = 0.0;
				fNextRight = 0.0;
				pLeftMotor->Set(0.0);
				pRightMotor->Set(0.0);
			}
		}
		else
		{
			bDrivingStraight = false;
			bMeasuredMove = false;
			fNextLeft = 0.0;
			fNextRight = 0.0;
			pLeftMotor->Set(0.0);
			pRightMotor->Set(0.0);
		}
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
		fNextLeft = (-1.0 - adjustment) * speed;
		fNextRight = (1.0 - adjustment) * speed;
	}
	else if (speed < 0.0)
	{
		/* if headed in negative direction
		 * +angle requires more power on the fNextLeft to fix
		 * -angle, fNextRight
		 */
		fNextLeft = (-1.0 + adjustment) * speed;
		fNextRight = (1.0 + adjustment) * speed;
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
		//fScale = 1;

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
		pLeftMotor->Set(-fLeftQ * FULLSPEED_FROMTALONS);
		pRightMotor->Set(fRightQ * FULLSPEED_FROMTALONS);
	}
	else
	{
		pLeftMotor->Set(-fLeftQ);
		pRightMotor->Set(fRightQ);
	}
}

void Drivetrain::RunCheezyDrive(bool bEnabled, float fWheel, float fThrottle, bool bQuickturn)
{
    struct DrivetrainGoal Goal;
    struct DrivetrainPosition Position;
    struct DrivetrainOutput Output;
    struct DrivetrainStatus Status;

    // TODO - select a quickturn button or do the normal spin

    Goal.steering = -fWheel;   // not sure why
    Goal.throttle = fThrottle;
    Goal.quickturn = bQuickturn;
    Goal.control_loop_driving = false;
    Goal.highgear = false;
    Goal.left_velocity_goal = 0.0;
    Goal.right_velocity_goal = 0.0;
    Goal.left_goal = 0.0;
    Goal.right_goal = 0.0;

    Position.left_encoder = -pLeftMotor->GetEncPosition() * METERS_PER_COUNT;
    Position.right_encoder = pRightMotor->GetEncPosition() * METERS_PER_COUNT;
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
        pLeftMotor->Set(-Output.left_voltage / 12.0);
        pRightMotor->Set(Output.right_voltage / 12.0);
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
