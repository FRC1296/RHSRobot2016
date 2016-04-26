/** \file
 *  Messages used for intertask communications
 */

/** Defines the messages we pass from task to task.
 *
 * The RobotMessage struct is a data structure used to pass information to the
 * robot's components. It is composed of a command that indicates the action to
 * be carried out and a union of params that contain additional data.
 */

#ifndef ROBOT_MESSAGE_H
#define ROBOT_MESSAGE_H

/**
 \msc
 arcgradient = 8;
 robot [label="Main\nRobot"],
 auto [label="Autonomous"],
 check [label="Check\nList"],
 drive [label="Drive\nTrain"],
 test [label="Component\nExample"];
 robot=>* [label="SYSTEM_MSGTIMEOUT"];
 robot=>* [label="SYSTEM_OK"];
 robot=>* [label="SYSTEM_ERROR"];
 robot=>* [label="STATE_DISABLED"];
 robot=>* [label="STATE_AUTONOMOUS"];
 robot=>* [label="STATE_TELEOPERATED"];
 robot=>* [label="STATE_TEST"];
 robot=>* [label="STATE_UNKNOWN"];
 robot=>auto [label="RUN"];
 robot=>check [label="RUN"];
 robot=>drive [label="STOP"];
 robot=>drive [label="DRIVE_TANK"];
 robot=>drive [label="DRIVE_ARCADE"];
 auto=>drive [label="DRIVE_STRAIGHT"];
 auto=>drive [label="TURN"];
 drive=>auto [label="AUTONOMOUS_RESPONSE_OK"]
 drive=>auto [label="AUTONOMOUS_RESPONSE_ERROR"]

 robot=>test[label="TEST"];
 \endmsc

 */

enum MessageCommand {
	COMMAND_UNKNOWN,					//!< COMMAND_UNKNOWN
	COMMAND_SYSTEM_MSGTIMEOUT,			//!< COMMAND_SYSTEM_MSGTIMEOUT
	COMMAND_SYSTEM_OK,					//!< COMMAND_SYSTEM_OK
	COMMAND_SYSTEM_ERROR,				//!< COMMAND_SYSTEM_ERROR
	COMMAND_SYSTEM_CONSTANTS,

	COMMAND_ROBOT_STATE_DISABLED,		//!< Tells all components that the robot is disabled
	COMMAND_ROBOT_STATE_AUTONOMOUS,		//!< Tells all components that the robot is in auto
	COMMAND_ROBOT_STATE_TELEOPERATED,	//!< Tells all components that the robot is in teleop
	COMMAND_ROBOT_STATE_TEST,			//!< Tells all components that the robot is in test
	COMMAND_ROBOT_STATE_UNKNOWN,		//!< Tells all components that the robot's state is unknown

	COMMAND_AUTONOMOUS_RUN,				//!< Tells Autonomous to run
	COMMAND_AUTONOMOUS_COMPLETE,		//!< Tells all components that Autonomous is done running the script
	COMMAND_AUTONOMOUS_RESPONSE_OK,		//!< Tells Autonomous that a command finished running successfully
	COMMAND_AUTONOMOUS_RESPONSE_ERROR,	//!< Tells Autonomous that a command had a error while running
	COMMAND_CHECKLIST_RUN,				//!< Tells CheckList to run

	COMMAND_AUTONOMOUS_SEARCHGOAL,
	COMMAND_AUTONOMOUS_SEARCHBALL,
	COMMAND_AUTONOMOUS_INTAKE,
	COMMAND_AUTONOMOUS_MOVEINTAKE,
	COMMAND_AUTONOMOUS_THROWUP,
	COMMAND_AUTONOMOUS_SHOOT,

	COMMAND_DRIVETRAIN_STOP,			//!< Tells Drivetrain to stop moving
	COMMAND_DRIVETRAIN_DRIVE_TANK,		//!< Tells Drivetrain to use tank drive
	COMMAND_DRIVETRAIN_DRIVE_ARCADE,	//!< Tells Drivetrain to use arcade drive
	COMMAND_DRIVETRAIN_AUTO_MOVE,		//!< Tells Drivetrain to move motors, used by Autonomous
	COMMAND_DRIVETRAIN_STRAIGHT,		//!< Tells Drivetrain to drive straight, used by Autonomous
	COMMAND_DRIVETRAIN_MSTRAIGHT,
	COMMAND_DRIVETRAIN_MLINE,
	COMMAND_DRIVETRAIN_TURN,			//!< Tells Drivetrain to turn, used by Autonomous
	COMMAND_DRIVETRAIN_DRIVE_SPLITARCADE,
	COMMAND_DRIVETRAIN_DRIVE_CHEEZY,	//!< Tells Drivetrain to use Cheezy drive
	COMMAND_DRIVETRAIN_REDSENSE,
	COMMAND_DRIVETRAIN_SETANGLE,

	COMMAND_ARM_FAR,
	COMMAND_ARM_CLOSE,
	COMMAND_ARM_INTAKE,
	COMMAND_ARM_INTAKE_STOP,
	COMMAND_ARM_INTAKE_OUT,
	COMMAND_ARM_SHOOT,
	COMMAND_ARM_MOVE_INTAKE,
	COMMAND_ARM_MOVE_RIDE,
	COMMAND_ARM_AUTO_MOVE_RIDE,
	COMMAND_ARM_LEDOFF,
	COMMAND_ARM_LEDWHITE,
	COMMAND_ARM_LEDCOLOR,
	COMMAND_ARM_ENABLE,
	COMMAND_ARM_MOVE_AFTERSHOOT,

	COMMAND_HANGER_HANG,
	COMMAND_HANGER_SOLENOID_ENABLE,
	COMMAND_HANGER_SOLENOID_DISABLE,

	COMMAND_TAIL_RAISE,
	COMMAND_TAIL_LOWER,

	COMMAND_SHOOTER_SHOOT,
	COMMAND_SHOOTER_SHOOTER_OPEN,
	COMMAND_SHOOTER_SHOOTER_CLOSE,
	COMMAND_SHOOTER_JAW_OPEN,
	COMMAND_SHOOTER_JAW_CLOSE,

	COMMAND_COMPONENT_TEST,				//!< COMMAND_COMPONENT_TEST

	COMMAND_LAST                      //!< COMMAND_LAST 
};
///Used to deliver joystick readings to Drivetrain
struct TankDriveParams {
	float left;
	float right;
};

///Used to deliver joystick readings to Drivetrain
struct CheezyDriveParams {
	float wheel;
	float throttle;
	bool bQuickturn;
};

///Used to deliver joystick readings to Drivetrain
struct ArcadeDriveParams {
	float x;
	float y;
};

struct ArmParams{
	bool direction;
};

struct SystemParams {
	float fBattery;
};

struct SplitArcadeDriveParams {
 	float wheel;
 	float throttle;
 	float spin;
 };

///Used to deliver autonomous values to Drivetrain
struct AutonomousParams {
	unsigned uMode;
	unsigned uDelay;
	///how long a function can run, maximum
	float timeout;
	///how long until a function performs
	float timein;

	///used by drivetrain for straight driving
	float driveSpeed;
	float driveDistance;
	float turnAngle;
	float driveTime;
};

///Contains all the parameter structures contained in a message
union MessageParams {
	TankDriveParams tankDrive;
	CheezyDriveParams cheezyDrive;
	SplitArcadeDriveParams splitArcadeDrive;
	AutonomousParams autonomous;
	ArmParams armParams;
	SystemParams system;
};

///A structure containing a command, a set of parameters, and a reply id, sent between components
struct RobotMessage {
	MessageCommand command;
	const char* replyQ;
	MessageParams params;
};

#endif //ROBOT_MESSAGE_H
