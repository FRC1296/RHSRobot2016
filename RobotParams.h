/** \file
 *  Defines task parameters, hardware assignments and controller button/axis assignment.
 *
 * This header contains basic parameters for the robot. All parameters must be constants with internal
 * linkage, otherwise the One Definition Rule will be violated.
 */

// TODO: please go over these items with a knowledgeable mentor and check to see what we need/don't need
#ifndef ROBOT_PARAMS_H
#define ROBOT_PARAMS_H

//Robot
#include <JoystickLayouts.h>			//For joystick layouts

//Robot Params
const char* const ROBOT_NAME =		"RhsRobot2016";	//Formal name
const char* const ROBOT_NICKNAME =  "Mittens";		//Nickname
const char* const ROBOT_VERSION =	"1.1";			//Version

//Robot Mode Macros - used to tell what mode the robot is in
#define ISAUTO			RobotBase::getInstance().IsAutonomous()
#define ISTELEOPERATED	RobotBase::getInstance().IsOperatorControl()
#define ISTEST			RobotBase::getInstance().IsTest()
#define ISENABLED		RobotBase::getInstance().IsEnabled()
#define ISDISABLED		RobotBase::getInstance().IsDisabled()

//Utility Functions - Define commonly used operations here
#define ABLIMIT(a,b)		if(a > b) a = b; else if(a < -b) a = -b;
#define TRUNC_THOU(a)		((int)(1000 * a)) * .001
#define TRUNC_HUND(a)		((int)(100 * a)) * .01
#define PRINTAUTOERROR		printf("Early Death! %s %i \n", __FILE__, __LINE__);

//Task Params - Defines component task priorites relative to the default priority.
//EXAMPLE: const int DRIVETRAIN_PRIORITY = DEFAULT_PRIORITY -2;
const int DEFAULT_PRIORITY = 150;
const int COMPONENT_PRIORITY 	= DEFAULT_PRIORITY;
const int DRIVETRAIN_PRIORITY 	= DEFAULT_PRIORITY;
const int AUTONOMOUS_PRIORITY 	= DEFAULT_PRIORITY;
const int AUTOEXEC_PRIORITY 	= DEFAULT_PRIORITY;
const int AUTOPARSER_PRIORITY 	= DEFAULT_PRIORITY;
const int ARM_PRIORITY 			= DEFAULT_PRIORITY;
const int TAIL_PRIORITY 		= DEFAULT_PRIORITY;
const int SHOOTER_PRIORITY 		= DEFAULT_PRIORITY;
const int HANGER_PRIORITY 		= DEFAULT_PRIORITY;

//Task Names - Used when you view the task list but used by the operating system
//EXAMPLE: const char* DRIVETRAIN_TASKNAME = "tDrive";
const char* const COMPONENT_TASKNAME	= "tComponent";
const char* const DRIVETRAIN_TASKNAME	= "tDrive";
const char* const AUTONOMOUS_TASKNAME	= "tAuto";
const char* const AUTOEXEC_TASKNAME		= "tAutoEx";
const char* const AUTOPARSER_TASKNAME	= "tParse";
const char* const ARM_TASKNAME			= "tArm";
const char* const TAIL_TASKNAME			= "tTail";
const char* const SHOOTER_TASKNAME		= "tShooter";
const char* const HANGER_TASKNAME		= "tHanger";
const char* const SHOOTER_SEQ_TASKNAME	= "tShooterSeq";
const char* const HANGER_SEQ_TASKNAME	= "tSHangerSeq";
//TODO change these variables throughout the code to PIPE or whatever instead  of QUEUE
//Queue Names - Used when you want to open the message queue for any task
//NOTE: 2015 - we use pipes instead of queues
//EXAMPLE: const char* DRIVETRAIN_TASKNAME = "tDrive";
const char* const COMPONENT_QUEUE 	= "/tmp/qComp";
const char* const DRIVETRAIN_QUEUE 	= "/tmp/qDrive";
const char* const AUTONOMOUS_QUEUE 	= "/tmp/qAuto";
const char* const AUTOPARSER_QUEUE 	= "/tmp/qParse";
const char* const ARM_QUEUE 	= "/tmp/qArm";
const char* const TAIL_QUEUE 	= "/tmp/qTail";
const char* const SHOOTER_QUEUE 	= "/tmp/qShooter";
const char* const HANGER_QUEUE 	= "/tmp/qHanger";

//PWM Channels - Assigns names to PWM ports 1-10 on the Roborio
//EXAMPLE: const int PWM_DRIVETRAIN_FRONT_LEFT_MOTOR = 1;
const int PWM_DRIVETRAIN_LEFT_MOTOR = 1;
const int PWM_DRIVETRAIN_RIGHT_MOTOR = 0;

//CAN IDs - Assigns names to the various CAN IDs
//EXAMPLE: const int CAN_PDB = 0;
/** \page motorID Motor Controller IDs
 * \verbatim
0 - PDB
1 - left drive motor
2 - right drive motor
Add more as needed.
 \endverbatim
 */

const int CAN_PCM_SHOOTER = 0;
const int CAN_PCM_JAW = 17;
const int CAN_PDB = 19;
const int CAN_DRIVETRAIN_LEFTONE_MOTOR = 1;
const int CAN_DRIVETRAIN_LEFTTWO_MOTOR = 2;
const int CAN_DRIVETRAIN_RIGHTONE_MOTOR = 3;
const int CAN_DRIVETRAIN_RIGHTTWO_MOTOR = 4;

const int CAN_ARM_INTAKE_MOTOR = 5;
const int CAN_HANGER_MOTOR = 6;
const int CAN_ARM_LEVER_MOTOR = 7;
const int CAN_TAIL_MOTOR = 8;


//Relay Channels - Assigns names to Relay ports 1-8 on the Roborio
//EXAMPLE: const int RLY_COMPRESSOR = 1;

//Digital I/O - Assigns names to Digital I/O ports 1-14 on the Roborio
//EXAMPLE: const int DIO_DRIVETRAIN_BEAM_BREAK = 0;
const int DIO_DRIVETRAIN_RED_SENSOR = 0;


//Solenoid - Assigns names to Solenoid ports 1-8 on the 9403
//EXAMPLE: const int SOL_DRIVETRAIN_SOLENOID_SHIFT_IN = 1;
const int SOL_SHOOTER_1 = 0;
const int SOL_SHOOTER_2 = 1;
const int SOL_SHOOTER_3 = 2;
const int SOL_SHOOTER_4 = 3;
const int SOL_JAWCLOSE = 5;
const int SOL_JAWOPEN = 7;
const int SOL_HANGER = 6;

//I2C - Assigns names to I2C ports 1-2 on the Roborio
//EXAMPLE: const int IO2C_AUTO_ACCEL = 1;
const int IO2C_AUTO_ACCEL = 1;

//Analog I/O - Assigns names to Analog I/O ports 1-8 on Anal;og Breakout Module
//EXAMPLE: const int AIO_BATTERY = 8;

//Joystick Input Device Counts - used by the listener to watch buttons and axis
const int JOYSTICK_BUTTON_COUNT = 10;
const int JOYSTICK_AXIS_COUNT = 5;

//POV IDs - Assign names to the 9 POV positions: -1 to 7
//EXAMPLE: const int POV_STILL = -1;
const int POV_STILL = -1;

//Primary Controller Mapping - Assigns action to buttons or axes on the first joystick
#undef	USE_X3D_FOR_CONTROLLER_1
#undef	USE_XBOX_FOR_CONTROLLER_1
#define	USE_L310_FOR_CONTROLLER_1

//Secondary Controller Mapping - Assigns action to buttons or axes on the second joystick
#undef	USE_X3D_FOR_CONTROLLER_2
#undef 	USE_XBOX_FOR_CONTROLLER_2
#define USE_L310_FOR_CONTROLLER_2

#ifdef USE_XBOX_FOR_CONTROLLER_1
#endif
/** \page joysticks Joystick Layouts
 * \verbatim
 	 +++++ Controller 1 +++++
  	A Button					Toggle noodle fan
  	B Button					~~
  	X Button					Hold Cube clicker at bottom to remove totes
  	Y Button					Release Cube clicker from hold
  	Start Button				Start Cube autocycle
  	Back Button					Stop Cube autocycle
  	Left Bumper					Run Conveyor forward
  	Right Bumper				Run Conveyor backwards - to claw
  	Left Thumbstick Button		Close CanLifter claw
  	Right Thumbstick Button		Open CanLifter claw
  	Left Thumbstick				Left tank, Arcade
  	Right Thumbstick			Right tank
  	D-pad						~~
  	Left Trigger				Lower CanLifter
  	RightTrigger				Raise CanLifter

 	 +++++ Controller 2 +++++
  	A Button					~~
  	B Button					~~
  	X Button					Hold Cube clicker at bottom to remove totes
  	Y Button					Release Cube clicker from hold
  	Start Button				Start Cube autocycle
  	Back Button					Stop Cube autocycle
  	Left Bumper					~~
  	Right Bumper				~~
 	Left Thumbstick Button		~~
  	Right Thumbstick Button		~~
  	Left Thumbstick				~~
  	Right Thumbstick			Raise/lower Cube clicker
  	D-pad						~~
  	Left Trigger				~~
  	RightTrigger				~~
 \endverbatim
 */
#ifdef USE_L310_FOR_CONTROLLER_1
//ID numbers for various buttons and axis
#define TANK_DRIVE_LEFT_ID			L310_THUMBSTICK_LEFT_Y
#define TANK_DRIVE_RIGHT_ID			L310_THUMBSTICK_RIGHT_Y
#define ARCADE_DRIVE_X_ID			L310_THUMBSTICK_LEFT_X
#define ARCADE_DRIVE_Y_ID			L310_THUMBSTICK_LEFT_Y

//#define TANK_DRIVE_LEFT			pow(-Controller_1->GetRawAxis(L310_THUMBSTICK_LEFT_Y),3)
//#define TANK_DRIVE_RIGHT			pow(-Controller_1->GetRawAxis(L310_THUMBSTICK_RIGHT_Y),3)
#define TANK_DRIVE_LEFT				(Controller_1->GetRawAxis(L310_THUMBSTICK_LEFT_Y))
#define TANK_DRIVE_RIGHT			(-Controller_1->GetRawAxis(L310_THUMBSTICK_RIGHT_Y))
#define CHEEZY_DRIVE_WHEEL			(Controller_1->GetRawAxis(L310_THUMBSTICK_RIGHT_X))
#define CHEEZY_DRIVE_THROTTLE		(-Controller_1->GetRawAxis(L310_THUMBSTICK_LEFT_Y))
#define CHEEZY_DRIVE_SPIN		    (-Controller_1->GetRawAxis(L310_TRIGGER_LEFT) + Controller_1->GetRawAxis(L310_TRIGGER_RIGHT))
#define CHEEZY_DRIVE_QUICKTURN		(Controller_1->GetRawButton(L310_BUTTON_BUMPER_LEFT))
#define ARCADE_DRIVE_X				(Controller_1->GetRawAxis(L310_THUMBSTICK_LEFT_X))
#define ARCADE_DRIVE_Y				(-Controller_1->GetRawAxis(L310_THUMBSTICK_LEFT_Y))
#define ARM_INTAKE_IN				((Controller_1->GetRawButton(L310_BUTTON_BUMPER_RIGHT)) || (Controller_2->GetRawButton(L310_BUTTON_BUMPER_RIGHT)))
#define ARM_INTAKE_OUT				((Controller_1->GetRawAxis(L310_TRIGGER_RIGHT)!=0) || (Controller_2->GetRawAxis(L310_TRIGGER_RIGHT)!=0))
#define TAIL_DOWN					((Controller_1->GetRawButton(L310_BUTTON_THUMB_RIGHT)) || (Controller_2->GetRawButton(L310_BUTTON_THUMB_RIGHT)))
#define TAIL_UP						((Controller_1->GetRawButton(L310_BUTTON_THUMB_LEFT)) || (Controller_2->GetRawButton(L310_BUTTON_THUMB_LEFT)))
#define	SHOOTER_SHOOT				((Controller_1->GetRawButton(L310_BUTTON_A)) || (Controller_2->GetRawButton(L310_BUTTON_A)))
#define ARM_FAR						((Controller_1->GetPOV(0)==0) || (Controller_2->GetPOV(0)==0))
#define ARM_CLOSE					((Controller_1->GetPOV(0)==90) || (Controller_2->GetPOV(0)==90))
#define ARM_MOVE_INTAKE				((Controller_1->GetPOV(0)==180) || (Controller_2->GetPOV(0)==180))
#define ARM_MOVE_RIDE				((Controller_1->GetPOV(0)==270) || (Controller_2->GetPOV(0)==270))
//#define DRIVE_RED				(Controller_1->GetRawButton(L310_BUTTON_Y))
//#define DRIVE_SEARCHBALL				(Controller_1->GetRawButton(L310_BUTTON_Y))
#define DRIVE_SEARCHON				((Controller_1->GetRawButton(L310_BUTTON_START)) || (Controller_2->GetRawButton(L310_BUTTON_START)))
#define DRIVE_SEARCHOFF				((Controller_1->GetRawButton(L310_BUTTON_BACK)) || (Controller_2->GetRawButton(L310_BUTTON_BACK)))
#define HANGER_HANG					((Controller_1->GetRawButton(L310_BUTTON_Y)) || (Controller_2->GetRawButton(L310_BUTTON_Y)))
#define DRIVE_ZERO_GYRO				((Controller_1->GetRawButton(L310_BUTTON_B)) || (Controller_2->GetRawButton(L310_BUTTON_B)))
#define ARM_ENABLE					((Controller_1->GetRawButton(L310_BUTTON_X)) || (Controller_2->GetRawButton(L310_BUTTON_X)))

#endif // USE_L310_FOR_CONTROLLER_1

#ifdef USE_X3D_FOR_CONTROLLER_2
#endif // USE_X3D_FOR_CONTROLLER_2

#ifdef USE_XBOX_FOR_CONTROLLER_2
#endif // USE_XBOX_FOR_CONTROLLER_2

#ifdef USE_L310_FOR_CONTROLLER_2
#endif // USE_L310_FOR_CONTROLLER_2

#endif //ROBOT_PARAMS_H
