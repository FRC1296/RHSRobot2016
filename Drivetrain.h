/** \file
 * Definitions of class to control the drive train.
 *
 * This classes is derived from the standard Component base class and includes
 * definitions for the devices used to control the pallet jack wheels.
 */

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

//Robot
#include "WPILib.h"

#include <ComponentBase.h>			//For ComponentBase class
#include <pthread.h>

#include "ADXRS453Z.h"
#include "PixyCam.h"
#include "PIDSearchOutput.h"
#include "../cheezy/frc1296.h"
#include <DriveTalon.h>

// constants used to tune TALONS

// measured 546 left and 534 right on the ground,
// 641 left and 562 right on blocks on 21 March 2016

const float FULLSPEED_FROMTALONS = 	500.00;	// measured on the robot in RPMs
const float TALON_FTERM_L = 		0.21;	// From CTRE manual, section 12.4
const float TALON_PTERM_L = 		(TALON_FTERM_L / 5.0);
const float TALON_ITERM_L = 		(TALON_PTERM_L / 10.0);
const float TALON_DTERM_L = 		(TALON_PTERM_L / 5.0);
const float TALON_FTERM_R = 		0.24;	// From CTRE manual, section 12.4
const float TALON_PTERM_R = 		(TALON_FTERM_R / 5.0);
const float TALON_ITERM_R = 		(TALON_PTERM_R / 10.0);
const float TALON_DTERM_R = 		(TALON_PTERM_R / 5.0);
const float TALON_MAXRAMP =			60;		// 200ms
const float TALON_IZONE	=			128;
const float TALON_COUNTSPERREV =	1024;	// from CTRE docs
const float REVSPERFOOT = (3.141519 * 6.0 / 12.0);
const double METERS_PER_COUNT = (REVSPERFOOT * 0.3048 / (double)TALON_COUNTSPERREV);

class CheezyLoop {

 public:

 	CheezyLoop();
 	~CheezyLoop();
 	static void Run(CheezyLoop *);

 	bool bOutputEnabled;

 	void Update(const DrivetrainGoal &goal,
 	    const DrivetrainPosition &position,
 	    DrivetrainOutput &output,
 	    DrivetrainStatus &status,
 		bool bEnabled);
 private:
 	struct DrivetrainGoal currentGoal;
 	struct DrivetrainPosition currentPosition;
 	struct DrivetrainOutput currentOutput;
 	struct DrivetrainStatus currentStatus;
 	Task* pTask;

 	mutable priority_recursive_mutex mutexData;

 	void Iterate(const DrivetrainGoal *goal,
 				 const DrivetrainPosition *position,
 		         DrivetrainOutput *output,
 		         DrivetrainStatus *status);
 };



class Drivetrain : public ComponentBase
{
public:
	Drivetrain();
	~Drivetrain();
	static void *StartTask(void *pThis)
	{
		((Drivetrain *)pThis)->DoWork();
		return(NULL);
	}

	bool GetGyroAngle();
	void ZeroGyro();
private:

	DriveTalon* pLeftOneMotor;
	CANTalon* pLeftTwoMotor;
	DriveTalon* pRightOneMotor;
	CANTalon* pRightTwoMotor;
	ADXRS453Z *pGyro;
	//PixyCam *pCamera;
	Timer *pAutoTimer;
	CheezyLoop *pCheezy;
	//PIDController* pSearchPID;
	PIDController* pTurnPID;
	//PIDSearchOutput* pSearchPIDOutput;
	PIDSearchOutput* pTurnPIDOutput;
	PIDSearchOutput* pSearchPIDOutput;
	DigitalInput* pLaserReturn;

	float fStraightDriveDistance = 0.0;

	//Timer *pAutoTimer; //watches autonomous time and disables it if needed.IN COMPONENT BASE
	//stores motor values during autonomous
	float fNextLeft = 0.0;
	float fNextRight = 0.0;
	float fStraightDriveSpeed = 0.0;
	float fStraightDriveTime = 0.0;
	float fTurnAngle = 0.0;
	float fTurnTime = 0.0;

	float fBatteryVoltage = 12.0;
	bool bDrivingStraight = false;
	bool bTurning = false;
	bool bUnderServoControl = false;
	bool bMeasuredMove = false;
	bool bRedSensing = false;

	///how strong direction recovery is in straight drive, higher = stronger
	const float recoverStrength = .03;
	const float fMaxRecoverSpeed = .35;
	const float fMaxRecoverAngle = 30.0; 		//used to keep straight drive recovery from becoming to violent

	///how far from goal the robot can be before stopping
	const float distError = 1.0;				//inches
	const float angleError = 2.0;				//degrees
	const float turnAngleSpeedMultiplyer = .002;

	//angle * mult = speed to be reduced by limit
	const float turnSpeedLimit = .20;
	const float fEncoderRatio = 0.023009;

	//Auto search variables
	const float fSearchAccuracy = 0.02f;
	const float fSearchMotorSpeed = .055f;
	const float fSearchTimeout = 10; //in seconds

	float fMaxVelLeft;
	float fMaxVelRight;
	//diameter*pi/encoder_resolution : 1.875 * 3.14 / 256

	void OnStateChange();
	void Run();
	void ArcadeDrive(float, float);
	void StraightDrive(float, float);
	void RunSplitArcade(float, float, float);
	void RunCheezyDrive(bool, float, float, bool);
	void Search();
	void RedSense();

	void StartStraightDrive(float, float, float);
	void IterateStraightDrive(void);
	void StraightDriveLoop(float);

	void StartTurn(float, float);
	void IterateTurn(void);

};

#endif			//DRIVETRAIN_H
