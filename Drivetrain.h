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

using namespace std;

#include "../cheezy/frc1296.h"

// constants used to tune TALONS

const float FULLSPEED_FROMTALONS = 	2800.00;							// measured on the robot
const float TALON_PTERM = 			0.15;
const float TALON_ITERM = 			0.0;
const float TALON_DTERM = 			0.0;
const float TALON_FTERM = 			(1023.0/FULLSPEED_FROMTALONS);		// full scale divided by measured max speed
const float TALON_MAXRAMP =			60;									// 200ms
const float TALON_IZONE	=			128;
const float TALON_COUNTSPERREV =	1024;								// from CTRE docs

const float REVSPERFOOT = (3.141519 * 6.0 / 12.0);						// pi x d / 12 inch per foot, d for 6" tires

const double METERS_PER_COUNT = (REVSPERFOOT * 0.3048 / 4096.0);

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
private:

	CANTalon* pLeftMotor;
	CANTalon* pLeftMotorFollow;
	CANTalon* pRightMotor;
	CANTalon* pRightMotorFollow;
	ADXRS453Z *pGyro;
	PixyCam *pCamera;
	Timer *pAutoTimer;

	//Timer *pAutoTimer; //watches autonomous time and disables it if needed.IN COMPONENT BASE
	//stores motor values during autonomous
	float fNextLeft = 0.0;
	float fNextRight = 0.0;
	float fStraightDriveSpeed = 0.0;
	float fStraightDriveTime = 0.0;
	float fStraightDriveDistance = 0.0;
	float fTurnAngle = 0.0;
	float fTurnTime = 0.0;
	float fBatteryVoltage = 12.0;

	bool bDrivingStraight = false;
	bool bTurning = false;
	bool bUnderServoControl = false;
	bool bMeasuredMove = false;

	///how strong direction recovery is in straight drive, higher = stronger
	const float recoverStrength = .09;
	const float fMaxRecoverSpeed = .35;
	const float fMaxRecoverAngle = 30.0; 		//used to keep straight drive recovery from becoming to violent

	///how far from goal the robot can be before stopping
	const float distError = 1.0;				//inches
	const float angleError = 2.0;				//degrees
	const float turnAngleSpeedMultiplyer = .05;

	//angle * mult = speed to be reduced by limit
	const float turnSpeedLimit = .50;
	const float fEncoderRatio = 0.023009;

	//diameter*pi/encoder_resolution : 1.875 * 3.14 / 256

	void OnStateChange();
	void Run();
	void ArcadeDrive(float, float);
	void StraightDrive(float, float);
	void RunSplitArcade(float, float, float);
	void RunCheezyDrive(bool, float, float, bool);

	void StartStraightDrive(float, float, float);
	void IterateStraightDrive(void);
	void StraightDriveLoop(float);

	void StartTurn(float, float);
	void IterateTurn(void);
};

#endif			//DRIVETRAIN_H
